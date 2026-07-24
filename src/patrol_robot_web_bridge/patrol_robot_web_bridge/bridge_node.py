import base64
import binascii
import array
import json
import math
import queue
import re
import socket
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from nav2_msgs.msg import SpeedLimit
from nav2_msgs.srv import ClearEntireCostmap, LoadMap
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import Image, JointState, LaserScan, PointCloud2
from slam_toolbox.srv import Reset, SaveMap
from std_msgs.msg import Float32, Float64, String
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray


WAYPOINT_SAFETY_RADIUS = 0.45
GAZEBO_BOUNDARY_WALL_INSET = 0.50
WAYPOINT_BOUNDARY_CLEARANCE = (
    WAYPOINT_SAFETY_RADIUS + GAZEBO_BOUNDARY_WALL_INSET
)
# Keep generated and coarse imported maps at 0.05 m/cell whenever the
# configured size limit allows it. This matches the packaged vector-generated
# map and retains enough structure for the 0.58 m x 0.46 m footprint.
NAVIGATION_GRID_TARGET_RESOLUTION = 0.05
MAX_NAVIGATION_GRID_DIMENSION = 2000
MAX_NAVIGATION_PATH_POINTS = 300
MAPPING_OPERATION_STATES = frozenset({
    'MAPPING',
    'STARTING',
    'EXPLORING',
    'NAVIGATING',
    'SAVING',
    'RESETTING',
    'DEPLOYING',
})


def _finite_float(value, field):
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{field} 不是有效数字') from error
    if not math.isfinite(number):
        raise ValueError(f'{field} 不是有限数字')
    return number


def _with_route_headings(document):
    """Fill missing headings while preserving explicit inspection framing."""
    waypoints = document.get('waypoints', [])
    if not isinstance(waypoints, list):
        raise ValueError('巡检点列表格式无效')
    if not waypoints:
        raise ValueError('至少需要一个作为基地的巡检点')
    normalized = []
    coordinates = []
    for index, waypoint in enumerate(waypoints):
        if not isinstance(waypoint, dict):
            raise ValueError(f'第 {index + 1} 个巡检点格式无效')
        coordinates.append((
            _finite_float(waypoint.get('x'), f'巡检点 {index + 1} x'),
            _finite_float(waypoint.get('y'), f'巡检点 {index + 1} y'),
        ))
    for index, waypoint in enumerate(waypoints):
        x, y = coordinates[index]
        if waypoint.get('yaw') is not None:
            yaw = _finite_float(waypoint.get('yaw'), f'巡检点 {index + 1} yaw')
        else:
            yaw = 0.0
            for offset in range(1, len(waypoints)):
                target_x, target_y = coordinates[(index + offset) % len(waypoints)]
                if math.hypot(target_x - x, target_y - y) > 1e-6:
                    yaw = math.atan2(target_y - y, target_x - x)
                    break
        normalized.append({**waypoint, 'x': x, 'y': y, 'yaw': yaw})
    return {**document, 'waypoints': normalized}


def _occupancy_blocks_circle(grid, x, y, radius):
    origin_x, origin_y, resolution, width, height, occupied = grid
    min_column = max(0, int(math.floor((x - radius - origin_x) / resolution)))
    max_column = min(width - 1, int(math.floor((x + radius - origin_x) / resolution)))
    min_grid_y = max(0, int(math.floor((y - radius - origin_y) / resolution)))
    max_grid_y = min(height - 1, int(math.floor((y + radius - origin_y) / resolution)))
    half_cell = resolution * math.sqrt(0.5)
    for grid_y in range(min_grid_y, max_grid_y + 1):
        cell_y = origin_y + (grid_y + 0.5) * resolution
        row = height - 1 - grid_y
        for column in range(min_column, max_column + 1):
            if not occupied[row * width + column]:
                continue
            cell_x = origin_x + (column + 0.5) * resolution
            if math.hypot(cell_x - x, cell_y - y) <= radius + half_cell:
                return True
    return False


def _resample_occupancy(occupied, width, height, source_resolution):
    """Upsample a coarse image-row occupancy grid without changing its world size."""
    world_width = width * source_resolution
    world_height = height * source_resolution
    target_resolution = max(
        min(source_resolution, NAVIGATION_GRID_TARGET_RESOLUTION),
        world_width / MAX_NAVIGATION_GRID_DIMENSION,
        world_height / MAX_NAVIGATION_GRID_DIMENSION,
    )
    if target_resolution >= source_resolution - 1.0e-9:
        return source_resolution, width, height, occupied

    target_width = max(1, int(math.ceil(world_width / target_resolution - 1.0e-9)))
    target_height = max(1, int(math.ceil(world_height / target_resolution - 1.0e-9)))
    resampled = [False] * (target_width * target_height)
    for target_row in range(target_height):
        source_row = min(
            height - 1,
            int(((target_row + 0.5) * target_resolution) / source_resolution),
        )
        source_offset = source_row * width
        target_offset = target_row * target_width
        for target_column in range(target_width):
            source_column = min(
                width - 1,
                int(((target_column + 0.5) * target_resolution) / source_resolution),
            )
            resampled[target_offset + target_column] = occupied[
                source_offset + source_column
            ]
    return target_resolution, target_width, target_height, resampled


def _scenario_waypoint_issues(payload, grid):
    """Mirror the UI guard so stale or direct API clients cannot bypass it."""
    bounds = payload.get('bounds') or {}
    min_x = _finite_float(bounds.get('minX', -8.0), '地图 minX')
    min_y = _finite_float(bounds.get('minY', -6.0), '地图 minY')
    max_x = min_x + _finite_float(bounds.get('width', 16.0), '地图 width')
    max_y = min_y + _finite_float(bounds.get('height', 12.0), '地图 height')
    objects = payload.get('objects') or []
    raster_margin = grid[2] * math.sqrt(0.5)
    issues = []
    for index, waypoint in enumerate(payload.get('waypoints') or []):
        x = _finite_float(waypoint.get('x'), f'巡检点 {index + 1} x')
        y = _finite_float(waypoint.get('y'), f'巡检点 {index + 1} y')
        name = str(waypoint.get('name') or f'巡检点 {index + 1}')
        prefix = f'#{index + 1} {name}'
        if (
            x < min_x + WAYPOINT_BOUNDARY_CLEARANCE
            or x > max_x - WAYPOINT_BOUNDARY_CLEARANCE
            or y < min_y + WAYPOINT_BOUNDARY_CLEARANCE
            or y > max_y - WAYPOINT_BOUNDARY_CLEARANCE
        ):
            issues.append(
                f'{prefix}：距离边界实体墙不足 '
                f'{WAYPOINT_BOUNDARY_CLEARANCE:.2f} m，无法安全原地转向'
            )
            continue
        collision = next((item for item in objects if isinstance(item, dict) and (
            abs(x - _finite_float(item.get('x', 0.0), '场景元素 x'))
            <= max(0.1, _finite_float(item.get('width', 1.0), '场景元素 width')) / 2.0
            + WAYPOINT_SAFETY_RADIUS + raster_margin
            and abs(y - _finite_float(item.get('y', 0.0), '场景元素 y'))
            <= max(0.1, _finite_float(item.get('depth', 1.0), '场景元素 depth')) / 2.0
            + WAYPOINT_SAFETY_RADIUS + raster_margin
        )), None)
        if collision is not None:
            issues.append(
                f'{prefix}：进入“{collision.get("name", "未命名场景元素")}”的安全区'
            )
            continue
        if _occupancy_blocks_circle(grid, x, y, WAYPOINT_SAFETY_RADIUS):
            issues.append(f'{prefix}：落在栅格障碍物或其旋转安全区内')
    return issues


def _scenario_route_connectivity_issues(
    payload,
    grid,
    clearance=WAYPOINT_SAFETY_RADIUS,
):
    """Reject routes whose goals are separated by walls or unsafe gaps.

    This is a deterministic, map-only gate that runs before map_server, AMCL,
    or PatrolManager state changes. PatrolManager still performs the
    authoritative Nav2 ComputePathToPose preflight immediately before motion.
    """
    waypoints = payload.get('waypoints') or []
    if len(waypoints) < 2:
        return []

    origin_x, origin_y, resolution, width, height, occupied = grid
    free_image = np.logical_not(
        np.asarray(occupied, dtype=np.bool_).reshape((height, width))
    ).astype(np.uint8)
    clearance_image = (
        cv2.distanceTransform(free_image, cv2.DIST_L2, 5) * resolution
    )
    traversable = clearance_image >= max(float(clearance), resolution)

    cells = []
    issues = []
    for index, waypoint in enumerate(waypoints):
        x = _finite_float(waypoint.get('x'), f'巡检点 {index + 1} x')
        y = _finite_float(waypoint.get('y'), f'巡检点 {index + 1} y')
        column = int(math.floor((x - origin_x) / resolution))
        grid_y = int(math.floor((y - origin_y) / resolution))
        row = height - 1 - grid_y
        name = str(waypoint.get('name') or f'巡检点 {index + 1}')
        cells.append((row, column, name))
        if (
            row < 0 or row >= height
            or column < 0 or column >= width
            or not bool(traversable[row, column])
        ):
            issues.append(
                f'#{index + 1} {name}：所在位置没有足够的底盘通行宽度'
            )
    if issues:
        return issues

    home_row, home_column, _ = cells[0]
    visited = np.zeros((height, width), dtype=np.uint8)
    visited[home_row, home_column] = 1
    frontier = deque([(home_row, home_column)])
    while frontier:
        row, column = frontier.popleft()
        for row_offset, column_offset in (
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1),
        ):
            next_row = row + row_offset
            next_column = column + column_offset
            if (
                next_row < 0 or next_row >= height
                or next_column < 0 or next_column >= width
                or visited[next_row, next_column]
                or not traversable[next_row, next_column]
            ):
                continue
            if row_offset and column_offset and (
                not traversable[row, next_column]
                or not traversable[next_row, column]
            ):
                # Never let a diagonal step cut through an obstacle corner.
                continue
            visited[next_row, next_column] = 1
            frontier.append((next_row, next_column))

    for index, (row, column, name) in enumerate(cells[1:], start=1):
        if not visited[row, column]:
            issues.append(
                f'#{index + 1} {name}：与基地点之间不存在满足底盘安全宽度的连通路径'
            )
    return issues


class RobotWebBridge(Node):
    """HTTP gateway for safe controls, telemetry, and the RGB-D gimbal stream."""

    def __init__(self):
        super().__init__('patrol_robot_web_bridge')
        self.declare_parameter('http_host', '0.0.0.0')
        self.declare_parameter('http_port', 8765)
        self.declare_parameter('max_linear_speed', 0.6)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('manual_command_timeout', 0.5)
        self.declare_parameter('hardware_config_file', '~/.ros/patrol_robot/hardware.json')
        self.declare_parameter('map_storage_dir', '~/.ros/patrol_robot/maps')
        self.declare_parameter('require_3d_map_on_save', False)
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_stream_fps', 12.0)
        self.declare_parameter('camera_jpeg_quality', 65)
        self.declare_parameter('camera_stream_width', 640)
        self.declare_parameter('camera_enabled_at_start', False)
        self.declare_parameter('camera_pan_limit_degrees', 90.0)
        self.declare_parameter('camera_tilt_up_limit_degrees', 25.0)
        self.declare_parameter('camera_tilt_down_limit_degrees', 35.0)
        self.declare_parameter('perception_initial_mode', 'lidar')
        self.declare_parameter('camera_cloud_timeout_seconds', 5.0)
        self.declare_parameter('voxel_preview_max_points', 30000)
        self.declare_parameter('perception_fault_delay_seconds', 3.0)
        self.declare_parameter('perception_recovery_stable_seconds', 1.5)
        self.declare_parameter('simulation_origin_x', -6.0)
        self.declare_parameter('simulation_origin_y', -4.0)
        self.declare_parameter('simulation_origin_yaw', 0.0)
        self.declare_parameter('odom_pose_is_world', True)
        self.declare_parameter('ground_truth_localization', False)
        self.declare_parameter('seed_initial_pose_at_start', False)
        self.declare_parameter('patrol_route_ready_at_start', True)
        self.declare_parameter('autonomous_exploration_available', True)

        self._max_linear = float(self.get_parameter('max_linear_speed').value)
        self._max_angular = float(self.get_parameter('max_angular_speed').value)
        self._command_timeout = float(self.get_parameter('manual_command_timeout').value)
        self._simulation_origin = (
            float(self.get_parameter('simulation_origin_x').value),
            float(self.get_parameter('simulation_origin_y').value),
            float(self.get_parameter('simulation_origin_yaw').value),
        )
        self._odom_pose_is_world = bool(
            self.get_parameter('odom_pose_is_world').value
        )
        self._ground_truth_localization = bool(
            self.get_parameter('ground_truth_localization').value
        )
        self._latest_odom_pose = None
        self._latest_ground_truth_pose = None
        self._last_ground_truth_pose_update = None
        self._initial_pose_repeats = (
            3
            if bool(self.get_parameter('seed_initial_pose_at_start').value)
            else 0
        )
        patrol_route_ready = bool(
            self.get_parameter('patrol_route_ready_at_start').value
        )
        autonomous_exploration_available = bool(
            self.get_parameter('autonomous_exploration_available').value
        )
        self._expected_scene_map_id = None
        self._pending_map_status = None
        self._pending_map_route = None
        self._pending_map_payload = None
        self._active_map_payload = None
        self._mapping_base_payload = None
        self._pending_patrol_start = None
        self._map_source_mode = 'slam'
        self._pending_mapping_resume = None
        self._mapping_pose_reset_deadline = None
        self._mapping_reset_wait_deadline = None
        self._scene_ready = True
        self._scene_error = None
        self._localization_pose_ready = True
        self._last_map_initial_pose = None
        self._map_storage_dir = Path(
            str(self.get_parameter('map_storage_dir').value)
        ).expanduser()
        self._mapping_catalog_path = (
            self._map_storage_dir / 'mapping_catalog.json'
        )
        self._require_3d_map_on_save = bool(
            self.get_parameter('require_3d_map_on_save').value
        )
        self._mapping_map = None
        self._mapping_started_at = None
        self._mapping_mode = 'idle'
        self._mapping_reset_in_progress = False
        self._mapping_session_visible = False
        self._voxel_preview_max_points = max(
            1000,
            min(
                int(self.get_parameter('voxel_preview_max_points').value),
                100000,
            ),
        )
        self._voxel_snapshot = None
        self._voxel_revision = 0
        self._camera_topic = str(self.get_parameter('camera_topic').value)
        self._camera_stream_fps = max(
            1.0,
            min(float(self.get_parameter('camera_stream_fps').value), 20.0),
        )
        self._camera_jpeg_quality = max(
            35,
            min(int(self.get_parameter('camera_jpeg_quality').value), 90),
        )
        self._camera_stream_width = max(
            160,
            min(int(self.get_parameter('camera_stream_width').value), 1280),
        )
        self._camera_pan_limit = max(
            5.0,
            min(float(self.get_parameter('camera_pan_limit_degrees').value), 180.0),
        )
        self._camera_tilt_up_limit = max(
            5.0,
            min(float(self.get_parameter('camera_tilt_up_limit_degrees').value), 90.0),
        )
        self._camera_tilt_down_limit = max(
            5.0,
            min(float(self.get_parameter('camera_tilt_down_limit_degrees').value), 90.0),
        )
        self._camera_cloud_timeout = max(
            2.0,
            float(self.get_parameter('camera_cloud_timeout_seconds').value),
        )
        self._perception_fault_delay = max(
            1.0,
            float(self.get_parameter('perception_fault_delay_seconds').value),
        )
        self._perception_recovery_stable = max(
            0.5,
            float(self.get_parameter('perception_recovery_stable_seconds').value),
        )
        initial_mode = str(self.get_parameter('perception_initial_mode').value)
        self._perception_mode = (
            initial_mode if initial_mode in ('lidar', 'camera', 'fusion') else 'lidar'
        )
        self._perception_generation = 0
        self._perception_pending = 0
        self._perception_fault_active = False
        self._perception_unhealthy_since = None
        self._perception_healthy_since = None
        self._perception_hold_active = False
        self._perception_stopped_patrol = False
        self._perception_resume_requested = False
        self._patrol_speed_limit = None
        self._commands = queue.Queue()
        self._lock = threading.Condition()
        self._last_manual_command = 0.0
        self._manual_active = False
        self._manual_override = False
        self._cv_bridge = CvBridge()
        cv2.setNumThreads(1)
        self._camera_subscription = None
        self._camera_frame = None
        self._camera_sequence = 0
        self._last_camera_enqueue = 0.0
        self._last_camera_error_log = 0.0
        self._camera_frame_times = deque(maxlen=30)
        self._camera_encode_queue = queue.Queue(maxsize=1)
        self._camera_encoder_stop = threading.Event()
        self._camera_encoder_thread = threading.Thread(
            target=self._camera_encoder_loop,
            daemon=True,
        )
        self._camera_encoder_thread.start()
        self._status = {
            'connected': True, 'patrol': {'state': 'UNKNOWN', 'running': False},
            'speed': 0.0, 'angular_speed': 0.0, 'x': 0.0, 'y': 0.0,
            'battery': None, 'lidar_ok': False, 'last_scan_age': None,
            'max_linear_speed': self._max_linear,
            'navigation': {
                'path': [],
                'frame_id': 'map',
                'source': 'none',
                '_updated_monotonic': None,
            },
            'map': {
                'active_id': 'pipeline-demo',
                'active_name': '管廊综合测试区',
                'active_revision': None,
                'pending_id': None,
                'pending_name': None,
                'pending_revision': None,
                'active_payload_available': False,
                'source_resolution': None,
                'navigation_resolution': None,
                'resampled': False,
                'pending_navigation_resolution': None,
                'transitioning': False,
                'localization_ready': True,
                'patrol_route_ready': patrol_route_ready,
                'mapping_base_available': False,
                'source_mode': self._map_source_mode,
                'error': None,
                'error_map_id': None,
            },
            'mapping': {
                'state': 'IDLE',
                'detail': '等待开始新的建图任务',
                'mode': 'idle',
                'enabled': False,
                'goals_reached': 0,
                'goals_failed': 0,
                'frontier_clusters': 0,
                'blacklisted_goals': 0,
                'duration_seconds': 0,
                'map_revision': 0,
                'coverage': 0.0,
                'known_cells': 0,
                'total_cells': 0,
                'save_error': None,
                'saved_map': None,
                'autonomous_available': autonomous_exploration_available,
            },
            'perception': {
                'mode': self._perception_mode,
                'lidar_enabled': self._perception_mode in ('lidar', 'fusion'),
                'camera_enabled': self._perception_mode in ('camera', 'fusion'),
                'transitioning': False,
                'gimbal_locked': self._perception_mode == 'camera',
                'safety_ok': True,
                'camera_points': 0,
                'error': None,
            },
            'camera': {
                'enabled': False,
                'ok': False,
                'topic': self._camera_topic,
                'frames': 0,
                'width': 0,
                'height': 0,
                'fps': 0.0,
                'stream_fps': self._camera_stream_fps,
                'pan_deg': 0.0,
                'tilt_deg': 0.0,
                'pan_target_deg': 0.0,
                'tilt_target_deg': 0.0,
                'gimbal_ok': False,
            },
        }

        # This node is the only publisher allowed on the physical base topic.
        # It arbitrates smoothed Nav2 output and high-priority manual commands.
        self._cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self._on_navigation_command,
            10,
        )
        self._waypoint_publisher = self.create_publisher(
            String,
            '/patrol_manager/set_waypoints',
            10,
        )
        self._map_scenario_publisher = self.create_publisher(
            String,
            '/patrol/map_scenario',
            10,
        )
        map_source_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_source_publisher = self.create_publisher(
            String,
            '/patrol/map_source/select',
            map_source_qos,
        )
        self._reset_pose_publisher = self.create_publisher(
            String,
            '/patrol/reset_pose',
            10,
        )
        self._initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10,
        )
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        # Gazebo teleportation does not reset wheel/EKF odometry. The truth
        # pose is used only to seed AMCL after a simulated map change; it is
        # never exposed as the continuous navigation odometry source.
        self.create_subscription(
            Odometry,
            '/ground_truth/odom',
            self._on_ground_truth_odom,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            NavPath,
            '/plan_smoothed',
            self._on_navigation_path,
            10,
        )
        self.create_subscription(
            NavPath,
            '/plan',
            self._on_raw_navigation_path,
            10,
        )
        self.create_subscription(
            LaserScan,
            '/scan',
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            '/camera/points/filtered',
            self._on_camera_cloud,
            qos_profile_sensor_data,
        )
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self._on_mapping_map,
            map_qos,
        )
        self.create_subscription(
            MarkerArray,
            '/occupied_cells_vis_array',
            self._on_octomap_markers,
            map_qos,
        )
        self.create_subscription(
            String,
            '/frontier_explorer/status',
            self._on_frontier_status,
            10,
        )
        self.create_subscription(
            String,
            '/patrol_manager/status',
            self._on_patrol_status,
            10,
        )
        self.create_subscription(
            Float32,
            '/patrol_manager/speed_limit',
            self._on_patrol_speed_limit,
            10,
        )
        self.create_subscription(
            String,
            '/patrol/map_scenario_status',
            self._on_map_scenario_status,
            10,
        )
        self.create_subscription(
            JointState,
            '/joint_states',
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self._camera_pan_publisher = self.create_publisher(
            Float64,
            '/camera/gimbal/pan/command',
            10,
        )
        self._camera_tilt_publisher = self.create_publisher(
            Float64,
            '/camera/gimbal/tilt/command',
            10,
        )
        self._patrol_clients = {
            name: self.create_client(Trigger, f'/patrol_manager/{name}')
            for name in ('start', 'stop', 'reset', 'estop', 'clear_estop')
        }
        self._speed_limit_publisher = self.create_publisher(
            SpeedLimit,
            '/speed_limit',
            10,
        )
        self._local_costmap_params = self.create_client(
            SetParameters,
            '/local_costmap/local_costmap/set_parameters',
        )
        self._global_costmap_params = self.create_client(
            SetParameters,
            '/global_costmap/global_costmap/set_parameters',
        )
        self._costmap_clear_clients = (
            self.create_client(
                ClearEntireCostmap,
                '/local_costmap/clear_entirely_local_costmap',
            ),
            self.create_client(
                ClearEntireCostmap,
                '/global_costmap/clear_entirely_global_costmap',
            ),
        )
        self._map_load_client = self.create_client(
            LoadMap,
            '/map_server/load_map',
        )
        self._slam_lifecycle_client = self.create_client(
            ChangeState,
            '/slam_toolbox/change_state',
        )
        self._slam_state_client = self.create_client(
            GetState,
            '/slam_toolbox/get_state',
        )
        self._amcl_params = self.create_client(
            SetParameters,
            '/amcl/set_parameters',
        )
        self._frontier_clients = {
            action: self.create_client(
                Trigger,
                f'/frontier_explorer/{action}',
            )
            for action in ('start', 'stop', 'reset')
        }
        self._save_map_client = self.create_client(
            SaveMap,
            '/slam_toolbox/save_map',
        )
        self._slam_reset_client = self.create_client(
            Reset,
            '/slam_toolbox/reset',
        )
        self._octomap_reset_client = self.create_client(
            Empty,
            '/octomap_server/reset',
        )
        self.create_timer(0.05, self._process_commands)
        self.create_timer(0.1, self._continue_mapping_mode_switch)
        self.create_timer(0.1, self._continue_slam_reset_when_ready)
        self.create_timer(0.1, self._manual_watchdog)
        self.create_timer(1.0, self._publish_speed_limit)
        self.create_timer(1.0, self._publish_map_source_state)
        self.create_timer(0.5, self._perception_watchdog)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.2, self._update_map_pose)
        self.create_timer(0.25, self._publish_map_initial_pose)
        self.create_timer(0.05, self._publish_ground_truth_map_transform)

        host = str(self.get_parameter('http_host').value)
        port = int(self.get_parameter('http_port').value)
        handler = self._make_handler()
        self._http = ThreadingHTTPServer((host, port), handler)
        self._http.daemon_threads = True
        self._http_thread = threading.Thread(
            target=self._http.serve_forever,
            daemon=True,
        )
        self._http_thread.start()
        self.get_logger().info(f'车辆 Web 网关已监听 http://{host}:{port}')
        if bool(self.get_parameter('camera_enabled_at_start').value):
            self._set_camera_enabled(True)

    def _publish_ground_truth_map_transform(self):
        if not self._ground_truth_localization:
            return
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        if self._odom_pose_is_world:
            transform.transform.rotation.w = 1.0
        else:
            # The lightweight simulator integrates odometry from (0, 0), but
            # spawns the robot at simulation_origin in map coordinates.
            # Publish that fixed map->odom offset so Nav2 sees one consistent
            # pose instead of alternating between AMCL's initial pose and the
            # odometry origin.
            origin_x, origin_y, origin_yaw = self._simulation_origin
            transform.transform.translation.x = origin_x
            transform.transform.translation.y = origin_y
            transform.transform.rotation.z = math.sin(origin_yaw / 2.0)
            transform.transform.rotation.w = math.cos(origin_yaw / 2.0)
        self._tf_broadcaster.sendTransform(transform)

    def _make_handler(self):
        bridge = self

        class Handler(BaseHTTPRequestHandler):
            def _send(self, status, payload):
                body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
                self.send_response(status)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Private-Network', 'true')
                self.send_header('Access-Control-Allow-Headers', 'Content-Type')
                self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
                self.end_headers()
                self.wfile.write(body)

            def _send_binary(self, status, content_type, body):
                self.send_response(status)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Private-Network', 'true')
                self.end_headers()
                self.wfile.write(body)

            def _stream_camera(self):
                # The enable command is processed on the ROS thread. Allow a
                # short grace period so the browser can open the stream
                # immediately after pressing the button.
                deadline = time.monotonic() + 2.0
                while not bridge.camera_enabled() and time.monotonic() < deadline:
                    time.sleep(0.05)
                if not bridge.camera_enabled():
                    self._send(409, {'ok': False, 'message': '摄像头尚未开启'})
                    return

                self.send_response(200)
                self.send_header(
                    'Content-Type',
                    'multipart/x-mixed-replace; boundary=frame',
                )
                self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
                self.send_header('Pragma', 'no-cache')
                self.send_header('X-Accel-Buffering', 'no')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Private-Network', 'true')
                self.end_headers()
                self.connection.setsockopt(
                    socket.IPPROTO_TCP,
                    socket.TCP_NODELAY,
                    1,
                )

                sequence = -1
                try:
                    while bridge.camera_enabled():
                        frame, next_sequence = bridge.camera_frame_after(
                            sequence,
                            timeout=0.25,
                        )
                        if frame is None:
                            continue
                        sequence = next_sequence
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(
                            f'Content-Length: {len(frame)}\r\n\r\n'.encode('ascii')
                        )
                        self.wfile.write(frame)
                        self.wfile.write(b'\r\n')
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, TimeoutError):
                    return

            def do_OPTIONS(self):
                self._send(204, {})

            def do_GET(self):
                path = urlparse(self.path).path
                if path == '/api/status':
                    self._send(200, bridge.status_snapshot())
                elif path == '/api/mapping/map':
                    snapshot = bridge.mapping_map_snapshot()
                    if snapshot is None:
                        self._send(200, {
                            'ok': False,
                            'message': '等待开始新的建图任务',
                            'mapping': bridge.status_snapshot()['mapping'],
                        })
                    else:
                        self._send(200, snapshot)
                elif path == '/api/mapping/maps':
                    self._send(200, {
                        'ok': True,
                        'maps': bridge.saved_mapping_maps(),
                    })
                elif path == '/api/mapping/voxels':
                    self._send(200, bridge.mapping_voxel_snapshot())
                elif path == '/api/maps/active':
                    active_map = bridge.active_map_payload()
                    if active_map is None:
                        self._send(404, {
                            'ok': False,
                            'message': '当前地图来自旧版会话，请重新应用一次地图',
                        })
                    else:
                        self._send(200, active_map)
                elif path == '/api/health':
                    self._send(200, {'ok': True, 'service': 'patrol_robot_web_bridge'})
                elif path == '/api/camera/stream':
                    self._stream_camera()
                elif path == '/api/camera/frame':
                    frame, _sequence = bridge.camera_frame_after(-1)
                    if frame is None:
                        self._send(503, {'ok': False, 'message': '正在等待摄像头画面'})
                    else:
                        self._send_binary(200, 'image/jpeg', frame)
                else:
                    self._send(404, {'ok': False, 'message': '接口不存在'})

            def do_POST(self):
                try:
                    length = int(self.headers.get('Content-Length', '0'))
                    payload = json.loads(self.rfile.read(length) or b'{}')
                    if not isinstance(payload, dict):
                        raise ValueError('请求正文必须是 JSON 对象')
                    bridge._commands.put((urlparse(self.path).path, payload))
                    self._send(202, {'ok': True, 'message': '命令已接收'})
                except (ValueError, json.JSONDecodeError) as error:
                    self._send(400, {'ok': False, 'message': str(error)})

            def log_message(self, _format, *_args):
                return

        return Handler

    def status_snapshot(self):
        with self._lock:
            status = dict(self._status)
            last_scan = status.pop('_last_scan', None)
            camera = dict(status.get('camera', {}))
            perception = dict(status.get('perception', {}))
            map_status = dict(status.get('map', {}))
            mapping_status = dict(status.get('mapping', {}))
            navigation = dict(status.get('navigation', {}))
            last_frame = camera.pop('_last_frame', None)
            last_gimbal_state = camera.pop('_last_gimbal_state', None)
            last_camera_cloud = perception.pop('_last_camera_cloud', None)
            frame_times = tuple(self._camera_frame_times)
        navigation_updated = navigation.pop('_updated_monotonic', None)
        navigation['path_age'] = (
            None
            if navigation_updated is None
            else round(time.monotonic() - navigation_updated, 2)
        )
        status['last_scan_age'] = (
            None if last_scan is None else round(time.monotonic() - last_scan, 2)
        )
        status['lidar_ok'] = (
            status['last_scan_age'] is not None
            and status['last_scan_age'] < 1.0
        )
        camera['last_frame_age'] = (
            None if last_frame is None else round(time.monotonic() - last_frame, 2)
        )
        camera['ok'] = bool(
            camera.get('enabled')
            and camera['last_frame_age'] is not None
            and camera['last_frame_age'] < 2.0
        )
        camera['gimbal_state_age'] = (
            None
            if last_gimbal_state is None
            else round(time.monotonic() - last_gimbal_state, 2)
        )
        camera['gimbal_ok'] = bool(
            camera['gimbal_state_age'] is not None
            and camera['gimbal_state_age'] < 2.0
        )
        camera['fps'] = (
            round((len(frame_times) - 1) / (frame_times[-1] - frame_times[0]), 1)
            if len(frame_times) > 1 and frame_times[-1] > frame_times[0]
            else 0.0
        )
        perception['last_camera_cloud_age'] = (
            None
            if last_camera_cloud is None
            else round(time.monotonic() - last_camera_cloud, 2)
        )
        perception['lidar_ok'] = status['lidar_ok']
        perception['camera_ok'] = bool(
            perception['last_camera_cloud_age'] is not None
            and perception['last_camera_cloud_age'] < self._camera_cloud_timeout
        )
        perception['camera_cloud_timeout'] = self._camera_cloud_timeout
        perception['active_sources'] = [
            source
            for source, enabled, healthy in (
                ('lidar', perception.get('lidar_enabled'), perception['lidar_ok']),
                ('camera', perception.get('camera_enabled'), perception['camera_ok']),
            )
            if enabled and healthy
        ]
        if perception.get('mode') == 'lidar':
            perception['safety_ok'] = perception['lidar_ok']
        elif perception.get('mode') == 'camera':
            perception['safety_ok'] = perception['camera_ok']
        else:
            perception['safety_ok'] = (
                perception['lidar_ok'] or perception['camera_ok']
            )
        if perception.get('error'):
            perception['safety_ok'] = False
        status['camera'] = camera
        status['perception'] = perception
        status['navigation'] = navigation
        # Mapping mode intentionally runs SLAM Toolbox without Nav2's static
        # map_server. Expose that distinction so the map editor can describe a
        # saved-SLAM-map update as "save patrol points" instead of pretending
        # that a static navigation map is being reloaded.
        map_status['static_map_service_ready'] = (
            self._map_load_client.service_is_ready()
        )
        status['map'] = map_status
        if self._mapping_started_at is not None:
            mapping_status['duration_seconds'] = max(
                0,
                int(time.monotonic() - self._mapping_started_at),
            )
        status['mapping'] = mapping_status
        patrol_status = status.get('patrol') or {}
        patrol_active = bool(patrol_status.get('running'))
        mapping_active = (
            str(mapping_status.get('state', 'IDLE'))
            in MAPPING_OPERATION_STATES
        )
        map_switching = bool(
            map_status.get('transitioning')
            or str(map_status.get('source_mode', '')).startswith('switching_')
        )
        if patrol_active:
            operation_owner = 'patrol'
            operation_state = str(patrol_status.get('state') or 'RUNNING')
            operation_detail = '巡检任务正在占用车辆'
        elif mapping_active:
            operation_owner = 'mapping'
            operation_state = str(mapping_status.get('state') or 'MAPPING')
            operation_detail = '自主建图任务正在占用车辆'
        elif map_switching:
            operation_owner = 'map'
            operation_state = 'SWITCHING'
            operation_detail = '地图与定位方式正在切换'
        else:
            operation_owner = 'idle'
            operation_state = 'IDLE'
            operation_detail = '车辆可选择开始巡检或开始建图'
        status['operation'] = {
            'owner': operation_owner,
            'state': operation_state,
            'locked': operation_owner != 'idle',
            'detail': operation_detail,
        }
        return status

    def active_map_payload(self):
        with self._lock:
            payload = self._active_map_payload
            return None if payload is None else json.loads(json.dumps(payload))

    def camera_enabled(self):
        with self._lock:
            return bool(self._status['camera']['enabled'])

    def camera_frame_after(self, sequence, timeout=0.0):
        with self._lock:
            if timeout > 0.0:
                self._lock.wait_for(
                    lambda: (
                        self._camera_frame is not None
                        and self._camera_sequence != sequence
                    ) or not self._status['camera']['enabled'],
                    timeout=timeout,
                )
            if self._camera_frame is None or self._camera_sequence == sequence:
                return None, self._camera_sequence
            return self._camera_frame, self._camera_sequence

    def _on_odom(self, message):
        orientation = message.pose.pose.orientation
        odom_yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        with self._lock:
            self._latest_odom_pose = (
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
                odom_yaw,
            )
            self._status.update({
                'speed': round(message.twist.twist.linear.x, 3),
                'angular_speed': round(message.twist.twist.angular.z, 3),
                'odom_x': round(message.pose.pose.position.x, 3),
                'odom_y': round(message.pose.pose.position.y, 3),
            })

    def _on_ground_truth_odom(self, message):
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        with self._lock:
            self._latest_ground_truth_pose = (
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
                yaw,
            )
            self._last_ground_truth_pose_update = time.monotonic()

    def _on_navigation_path(self, message):
        self._store_navigation_path(message, 'plan_smoothed')

    def _on_raw_navigation_path(self, message):
        self._store_navigation_path(message, 'plan')

    def _store_navigation_path(self, message, source):
        poses = message.poses
        if not poses:
            return
        stride = max(1, int(math.ceil(len(poses) / MAX_NAVIGATION_PATH_POINTS)))
        selected = list(poses[::stride])
        if selected[-1] is not poses[-1]:
            selected.append(poses[-1])
        points = [
            {
                'x': round(float(pose.pose.position.x), 3),
                'y': round(float(pose.pose.position.y), 3),
            }
            for pose in selected
        ]
        with self._lock:
            self._status['navigation'] = {
                'path': points,
                'frame_id': message.header.frame_id or 'map',
                'source': source,
                '_updated_monotonic': time.monotonic(),
            }

    def _clear_navigation_path(self):
        with self._lock:
            self._status['navigation'] = {
                'path': [],
                'frame_id': 'map',
                'source': 'none',
                '_updated_monotonic': None,
            }

    def _on_navigation_command(self, message):
        """Relay smoothed Nav2 velocity only when manual control is released."""
        if self._manual_override:
            return
        self._publish_base_command(message)

    def _publish_base_command(self, command):
        """Publish one REP-103 command through the single physical-base output."""
        output = Twist()
        output.linear.x = command.linear.x
        output.linear.y = command.linear.y
        output.linear.z = command.linear.z
        output.angular.x = command.angular.x
        output.angular.y = command.angular.y
        # Gazebo DiffDrive already defines positive angular.z as counter-clockwise
        # (left). Inverting it here makes both manual steering and Nav2 rotate
        # away from their requested heading.
        output.angular.z = command.angular.z
        self._cmd_publisher.publish(output)

    def _update_map_pose(self):
        try:
            transform = self._tf_buffer.lookup_transform('map', 'base_footprint', Time())
            rotation = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
            )
            with self._lock:
                self._status['x'] = round(transform.transform.translation.x, 3)
                self._status['y'] = round(transform.transform.translation.y, 3)
                self._status['yaw'] = round(yaw, 4)
        except TransformException:
            pass

    def _on_scan(self, _message):
        with self._lock:
            self._status['_last_scan'] = time.monotonic()

    def _on_camera_cloud(self, message):
        with self._lock:
            perception = self._status['perception']
            perception['_last_camera_cloud'] = time.monotonic()
            perception['camera_points'] = int(message.width * message.height)

    def _on_octomap_markers(self, message):
        """Convert OctoMap CUBE_LIST markers into a compact browser payload."""
        records = []
        frame_id = 'map'
        source_voxel_count = 0
        for marker in message.markers:
            # visualization_msgs/Marker: ADD=0, DELETE=2, DELETEALL=3,
            # CUBE_LIST=6. A complete OctoMap publication contains one
            # CUBE_LIST per occupied tree depth.
            if int(marker.action) == 3:
                records.clear()
                source_voxel_count = 0
                continue
            if int(marker.action) != 0 or int(marker.type) != 6:
                continue
            if marker.header.frame_id:
                frame_id = str(marker.header.frame_id)
            size = max(
                0.001,
                float(marker.scale.x or marker.scale.y or marker.scale.z),
            )
            default_color = marker.color
            colors = marker.colors
            source_voxel_count += len(marker.points)
            for index, point in enumerate(marker.points):
                point_color = (
                    colors[index] if index < len(colors) else default_color
                )
                records.append((
                    float(point.x),
                    float(point.y),
                    float(point.z),
                    size,
                    float(point_color.r),
                    float(point_color.g),
                    float(point_color.b),
                ))

        if len(records) > self._voxel_preview_max_points:
            source_count = len(records)
            records = [
                records[min(
                    source_count - 1,
                    int(index * source_count / self._voxel_preview_max_points),
                )]
                for index in range(self._voxel_preview_max_points)
            ]

        values = array.array('f')
        for record in records:
            values.extend(record)
        if sys.byteorder != 'little':
            values.byteswap()

        self._voxel_revision += 1
        snapshot = {
            'ok': bool(records),
            'revision': self._voxel_revision,
            'frame_id': frame_id,
            'encoding': 'base64-f32le-xyzsrgb',
            'stride': 7,
            'data': base64.b64encode(values.tobytes()).decode('ascii'),
            'voxel_count': len(records),
            'source_voxel_count': source_voxel_count,
            'truncated': source_voxel_count > len(records),
        }
        with self._lock:
            self._voxel_snapshot = snapshot
            self._status['mapping'].update({
                'voxel_count': len(records),
                'voxel_source_count': source_voxel_count,
                'voxel_revision': self._voxel_revision,
            })

    def _on_mapping_map(self, message):
        with self._lock:
            if self._mapping_reset_in_progress:
                return
        runs = []
        previous = None
        count = 0
        known_cells = 0
        occupied_cells = 0
        for raw_value in message.data:
            value = int(raw_value)
            if value >= 0:
                known_cells += 1
            if value >= 65:
                occupied_cells += 1
            if previous is None:
                previous = value
                count = 1
            elif value == previous:
                count += 1
            else:
                runs.extend((previous, count))
                previous = value
                count = 1
        if previous is not None:
            runs.extend((previous, count))

        total_cells = int(message.info.width * message.info.height)
        revision = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        snapshot = {
            'ok': True,
            'frame_id': message.header.frame_id or 'map',
            'revision': revision,
            'width': int(message.info.width),
            'height': int(message.info.height),
            'resolution': float(message.info.resolution),
            'origin': {
                'x': float(message.info.origin.position.x),
                'y': float(message.info.origin.position.y),
            },
            'encoding': 'rle-int8',
            'runs': runs,
            'known_cells': known_cells,
            'occupied_cells': occupied_cells,
            'total_cells': total_cells,
            'coverage': round(
                (known_cells / total_cells * 100.0) if total_cells else 0.0,
                1,
            ),
        }
        with self._lock:
            self._mapping_map = snapshot
            if self._mapping_session_visible:
                self._status['mapping'].update({
                    'map_revision': revision,
                    'coverage': snapshot['coverage'],
                    'known_cells': known_cells,
                    'total_cells': total_cells,
                })

    def _on_frontier_status(self, message):
        with self._lock:
            if self._mapping_reset_in_progress:
                return
        try:
            explorer = json.loads(message.data)
        except json.JSONDecodeError:
            return
        explorer_state = str(explorer.get('state', 'UNKNOWN'))
        with self._lock:
            current_state = str(self._status['mapping'].get('state', 'IDLE'))
        if current_state in (
            'SAVING',
            'SAVED',
            'DEPLOYING',
            'DEPLOYED',
            'ERROR',
        ):
            public_state = current_state
        elif explorer_state in ('COMPLETED', 'COMPLETED_WITH_UNREACHABLE'):
            public_state = explorer_state
        elif bool(explorer.get('enabled')):
            public_state = 'EXPLORING'
        elif self._mapping_mode == 'manual':
            public_state = 'MAPPING'
        elif self._mapping_started_at is None:
            public_state = 'IDLE'
        else:
            public_state = 'PAUSED'
        if current_state in (
            'SAVING',
            'SAVED',
            'DEPLOYING',
            'DEPLOYED',
            'ERROR',
        ):
            with self._lock:
                public_detail = str(
                    self._status['mapping'].get('detail')
                    or self._status['mapping'].get('save_error')
                    or '建图状态已更新'
                )
        else:
            public_detail = str(
                explorer.get('detail') or '自主探索状态已更新'
            )
        if bool(explorer.get('enabled')) and self._mapping_started_at is None:
            self._mapping_started_at = time.monotonic()
            self._mapping_mode = 'autonomous'
        with self._lock:
            self._status['mapping'].update({
                'state': public_state,
                'detail': public_detail,
                'mode': self._mapping_mode,
                'enabled': bool(explorer.get('enabled')),
                'goals_reached': int(explorer.get('goals_reached') or 0),
                'goals_failed': int(explorer.get('goals_failed') or 0),
                'frontier_clusters': int(
                    explorer.get('frontier_clusters') or 0
                ),
                'blacklisted_goals': int(
                    explorer.get('blacklisted_goals') or 0
                ),
            })

    def mapping_map_snapshot(self):
        with self._lock:
            if self._mapping_map is None or not self._mapping_session_visible:
                return None
            snapshot = dict(self._mapping_map)
            snapshot['runs'] = list(self._mapping_map['runs'])
            snapshot['robot'] = {
                'x': float(self._status.get('x', 0.0)),
                'y': float(self._status.get('y', 0.0)),
                'yaw': float(self._status.get('yaw', 0.0)),
            }
            snapshot['mapping'] = dict(self._status['mapping'])
        snapshot['camera'] = self.status_snapshot()['camera']
        if self._mapping_started_at is not None:
            snapshot['mapping']['duration_seconds'] = max(
                0,
                int(time.monotonic() - self._mapping_started_at),
            )
        return snapshot

    def mapping_voxel_snapshot(self):
        with self._lock:
            if self._voxel_snapshot is None:
                return {
                    'ok': False,
                    'message': (
                        '尚未收到 OctoMap 体素；'
                        '请使用纯 SLAM 三维仿真并等待 RGB-D 数据'
                    ),
                    'voxel_count': 0,
                    'source_voxel_count': 0,
                    'robot': {
                        'x': float(self._status.get('x', 0.0)),
                        'y': float(self._status.get('y', 0.0)),
                        'yaw': float(self._status.get('yaw', 0.0)),
                    },
                }
            snapshot = dict(self._voxel_snapshot)
            snapshot['robot'] = {
                'x': float(self._status.get('x', 0.0)),
                'y': float(self._status.get('y', 0.0)),
                'yaw': float(self._status.get('yaw', 0.0)),
            }
        return snapshot

    def saved_mapping_maps(self):
        try:
            catalog = json.loads(
                self._mapping_catalog_path.read_text(encoding='utf-8')
            )
        except (OSError, json.JSONDecodeError):
            catalog = []
        if not isinstance(catalog, list):
            return []
        return [
            item
            for item in catalog
            if isinstance(item, dict)
            and item.get('id')
            and (self._map_storage_dir / f'{item["id"]}.yaml').is_file()
        ]

    def _on_patrol_status(self, message):
        try:
            patrol = json.loads(message.data)
            with self._lock:
                self._status['patrol'] = patrol
                if (
                    patrol.get('state') == 'RETRY_WAIT'
                    and (
                        int(patrol.get('retry_count') or 0) > 0
                        or int(patrol.get('similar_path_replan_count') or 0) > 0
                        or bool(patrol.get('route_replan_pending'))
                    )
                ):
                    # Do not leave the rejected/failed blue path on screen while
                    # the planner is selecting a genuinely different route.
                    self._status['navigation'] = {
                        'path': [],
                        'frame_id': 'map',
                        'source': 'none',
                        '_updated_monotonic': None,
                    }
                if not bool(patrol.get('running')):
                    self._patrol_speed_limit = None
            self._publish_speed_limit()
        except json.JSONDecodeError:
            pass

    def _on_patrol_speed_limit(self, message):
        limit = float(message.data)
        if not math.isfinite(limit) or limit <= 0.0:
            return
        self._patrol_speed_limit = max(0.05, min(limit, 1.5))
        self._publish_speed_limit()

    def _on_map_scenario_status(self, message):
        try:
            status = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if str(status.get('map_id', '')) != self._expected_scene_map_id:
            return
        ok = bool(status.get('ok')) and bool(status.get('robot_home_ready'))
        self._scene_ready = ok
        self._scene_error = None if ok else str(
            status.get('error') or 'Gazebo 场景或车辆基地重置失败'
        )
        if self._scene_error:
            self._abort_map_transition(self._scene_error)
            self.get_logger().error(self._scene_error)
            return
        self._finish_map_transition_if_ready()

    def _on_joint_state(self, message):
        positions = dict(zip(message.name, message.position))
        pan = positions.get('camera_pan_joint')
        tilt = positions.get('camera_tilt_joint')
        if pan is None and tilt is None:
            return
        with self._lock:
            camera = self._status['camera']
            if pan is not None:
                camera['pan_deg'] = round(math.degrees(float(pan)), 1)
            if tilt is not None:
                # The physical Y-axis joint is positive downward. The web UI
                # reports the more natural convention where positive is up.
                camera['tilt_deg'] = round(-math.degrees(float(tilt)), 1)
            camera['_last_gimbal_state'] = time.monotonic()

    def _process_commands(self):
        for _ in range(20):
            try:
                path, payload = self._commands.get_nowait()
            except queue.Empty:
                return
            if path.startswith('/api/mapping/'):
                self._process_mapping_command(path, payload)
            elif path.startswith('/api/patrol/'):
                action = path.rsplit('/', 1)[-1]
                client = self._patrol_clients.get(action)
                if client and not client.service_is_ready():
                    # Do not silently lose Start/Stop while patrol_manager is
                    # restarting or completing an action cancellation. Keep
                    # the command ordered at the back of the queue and retry
                    # it on the next timer tick.
                    self._commands.put((path, payload))
                    return
                if client:
                    if action == 'start':
                        with self._lock:
                            map_status = dict(self._status['map'])
                            mapping_state = str(
                                self._status['mapping'].get('state', 'IDLE')
                            )
                            mapping_base_payload = self._mapping_base_payload
                        if mapping_state in MAPPING_OPERATION_STATES:
                            self.get_logger().warning(
                                '建图任务运行中，拒绝同时启动巡检'
                            )
                            continue
                        if (
                            self._map_source_mode == 'slam'
                            and isinstance(mapping_base_payload, dict)
                            and not map_status.get('transitioning')
                            and not map_status.get('patrol_route_ready', False)
                        ):
                            # A mapping test started from an applied imported,
                            # generated, or saved map. Restore that frozen map
                            # and its route first, then requeue this same start
                            # command after AMCL initialization succeeds.
                            restore_payload = json.loads(
                                json.dumps(mapping_base_payload)
                            )
                            if payload.get('loop_count') is not None:
                                try:
                                    restore_payload['loop_count'] = max(
                                        1,
                                        min(
                                            1000,
                                            int(payload.get('loop_count')),
                                        ),
                                    )
                                except (TypeError, ValueError):
                                    self.get_logger().warning(
                                        '巡检圈数无效，恢复地图时沿用原设置'
                                    )
                            self._pending_patrol_start = dict(payload)
                            self._activate_map(restore_payload)
                            continue
                        if (
                            map_status.get('transitioning')
                            or not map_status.get('localization_ready', False)
                            or not map_status.get('patrol_route_ready', False)
                            or self._map_source_mode not in ('slam', 'static')
                        ):
                            self.get_logger().warning(
                                '地图、定位或巡检路线尚未就绪，拒绝启动巡检'
                            )
                            continue
                        perception = self.status_snapshot()['perception']
                        if (
                            perception.get('transitioning')
                            or not perception.get('safety_ok')
                        ):
                            self.get_logger().warning(
                                '感知数据尚未稳定，拒绝启动巡检'
                            )
                            continue
                        if self._manual_override:
                            self.get_logger().info(
                                '人工控制已释放，自动导航重新取得底盘控制权'
                            )
                        self._perception_hold_active = False
                        self._perception_stopped_patrol = False
                        self._perception_resume_requested = False
                        self._manual_override = False
                        # Frontier exploration and a patrol route both use the
                        # same NavigateToPose server. Patrol start explicitly
                        # wins so two autonomous owners never compete.
                        frontier_stop = self._frontier_clients['stop']
                        if frontier_stop.service_is_ready():
                            frontier_stop.call_async(Trigger.Request())
                    elif action in ('stop', 'reset'):
                        # An explicit user stop always wins over automatic
                        # recovery and must never be followed by an auto-resume.
                        self._perception_hold_active = False
                        self._perception_stopped_patrol = False
                        self._perception_resume_requested = False
                        # Keep late navigation commands blocked while the
                        # asynchronous action cancellation is completing.
                        self._publish_manual(0.0, 0.0)
                        self._clear_navigation_path()
                    client.call_async(Trigger.Request())
            elif path == '/api/navigation/waypoints':
                try:
                    route = _with_route_headings(payload)
                except ValueError as error:
                    self.get_logger().error(f'拒绝无效巡检路线：{error}')
                else:
                    with self._lock:
                        active_map_id = self._status['map'].get('active_id')
                        active_revision = self._status['map'].get('active_revision')
                        active_map_payload = self._active_map_payload
                    route_map_id = str(route.get('map_id', '')).strip()
                    route_revision = str(route.get('map_revision', '')).strip() or None
                    if route_map_id and route_map_id != active_map_id:
                        self.get_logger().error(
                            f'拒绝巡检路线：路线属于 {route_map_id}，'
                            f'车辆当前地图为 {active_map_id}'
                        )
                        continue
                    if (
                        route_revision
                        and active_revision
                        and route_revision != active_revision
                    ):
                        self.get_logger().error(
                            '拒绝巡检路线：路线版本与车辆当前地图不一致'
                        )
                        continue
                    if isinstance(active_map_payload, dict):
                        try:
                            self._validated_scenario_grid({
                                **active_map_payload,
                                'waypoints': route['waypoints'],
                            })
                        except (ValueError, TypeError, binascii.Error) as error:
                            with self._lock:
                                self._status['map'].update({
                                    'patrol_route_ready': False,
                                    'error': f'巡检路线无效：{error}',
                                    'error_map_id': active_map_id,
                                })
                            self.get_logger().error(
                                f'拒绝不连通的巡检路线：{error}'
                            )
                            continue
                    message = String()
                    message.data = json.dumps(route, ensure_ascii=False)
                    self._waypoint_publisher.publish(message)
                    with self._lock:
                        self._status['map']['patrol_route_ready'] = True
            elif path == '/api/maps/activate':
                self._activate_map(payload)
            elif path == '/api/config/speed':
                self._set_speed_limit(
                    float(payload.get('linear', self._max_linear)),
                    float(payload.get('angular', self._max_angular)),
                )
            elif path == '/api/control/manual':
                self._publish_manual(
                    float(payload.get('linear', 0.0)),
                    float(payload.get('angular', 0.0)),
                )
            elif path == '/api/control/emergency-stop':
                self._perception_hold_active = False
                self._perception_stopped_patrol = False
                self._perception_resume_requested = False
                self._publish_manual(0.0, 0.0)
                client = self._patrol_clients['estop']
                if not client.service_is_ready():
                    self._commands.put((path, payload))
                    return
                client.call_async(Trigger.Request())
            elif path == '/api/control/emergency-reset':
                client = self._patrol_clients['clear_estop']
                if not client.service_is_ready():
                    self._commands.put((path, payload))
                    return
                client.call_async(Trigger.Request())
            elif path == '/api/camera/enable':
                self._set_camera_enabled(bool(payload.get('enabled', False)))
            elif path == '/api/camera/gimbal':
                self._set_camera_gimbal(
                    float(payload.get('pan', 0.0)),
                    float(payload.get('tilt', 0.0)),
                )
            elif path == '/api/perception/mode':
                self._set_perception_mode(str(payload.get('mode', 'fusion')))
            elif path == '/api/config/hardware':
                self._set_hardware_config(payload)

    def _process_mapping_command(self, path, payload):
        action = path.rsplit('/', 1)[-1]
        if action in ('start', 'explore'):
            with self._lock:
                patrol_running = bool(
                    self._status.get('patrol', {}).get('running')
                )
                map_transitioning = bool(
                    self._status.get('map', {}).get('transitioning')
                )
            if patrol_running:
                with self._lock:
                    self._status['mapping'].update({
                        'state': 'IDLE',
                        'detail': '巡检任务正在运行；请先停止或等待巡检完成',
                        'mode': 'idle',
                        'enabled': False,
                    })
                self.get_logger().warning(
                    '巡检任务运行中，拒绝同时启动建图'
                )
                return
            if map_transitioning:
                with self._lock:
                    self._status['mapping'].update({
                        'state': 'IDLE',
                        'detail': '地图与定位方式正在切换，请等待应用完成',
                        'mode': 'idle',
                        'enabled': False,
                    })
                return
        if (
            action in ('start', 'explore')
            and self._prepare_slam_mapping_mode(path, payload)
        ):
            return
        if action == 'start':
            self._stop_patrol_for_mapping()
            self._mapping_started_at = time.monotonic()
            self._mapping_mode = 'manual'
            self._mapping_session_visible = True
            self._call_frontier('stop')
            with self._lock:
                self._status['map']['patrol_route_ready'] = False
                self._status['mapping'].update({
                    'state': 'MAPPING',
                    'detail': 'SLAM 正在更新，可使用网页摇杆手动扫图',
                    'mode': 'manual',
                    'enabled': True,
                    'save_error': None,
                    'saved_map': None,
                })
        elif action == 'explore':
            self._stop_patrol_for_mapping()
            self._mapping_started_at = time.monotonic()
            self._mapping_mode = 'autonomous'
            self._mapping_session_visible = True
            self._manual_override = False
            self._call_frontier('reset')
            with self._lock:
                self._status['map']['patrol_route_ready'] = False
                self._status['mapping'].update({
                    'state': 'STARTING',
                    'detail': '正在启动前沿探索并选择首个未知边界',
                    'mode': 'autonomous',
                    'enabled': True,
                    'goals_reached': 0,
                    'goals_failed': 0,
                    'save_error': None,
                    'saved_map': None,
                })
        elif action == 'stop':
            self._call_frontier('stop')
            self._publish_manual(0.0, 0.0)
            self._mapping_mode = 'idle'
            with self._lock:
                self._status['mapping'].update({
                    'state': 'PAUSED',
                    'detail': '建图已暂停，当前地图仍保留在内存中',
                    'enabled': False,
                })
        elif action == 'discard':
            self._call_frontier('stop')
            self._publish_manual(0.0, 0.0)
            self._discard_mapping_session()
        elif action == 'finish':
            self._save_mapping_map(str(payload.get('name', '')).strip())
        elif action == 'deploy':
            self._deploy_mapping_map(str(payload.get('id', '')).strip())
        elif action == 'delete':
            self._delete_mapping_map(str(payload.get('id', '')).strip())

    def _stop_patrol_for_mapping(self):
        """Give manual/autonomous mapping exclusive ownership of Nav2."""
        stop_client = self._patrol_clients['stop']
        if stop_client.service_is_ready():
            stop_client.call_async(Trigger.Request())
        self._clear_navigation_path()

    def _prepare_slam_mapping_mode(self, path, payload):
        """Switch a frozen navigation map back to a fresh live SLAM session."""
        if self._map_source_mode == 'slam':
            return False
        if self._pending_mapping_resume is not None:
            return True
        if self._map_source_mode != 'static':
            with self._lock:
                self._status['mapping'].update({
                    'state': 'STARTING',
                    'detail': '正在切换到实时 SLAM，请稍候',
                    'enabled': False,
                })
            return True
        if not self._amcl_params.service_is_ready():
            self._set_mapping_source_error('AMCL 参数服务尚未就绪，无法开始建图')
            return True
        if not self._slam_lifecycle_client.service_is_ready():
            self._set_mapping_source_error('SLAM 生命周期服务尚未就绪，无法开始建图')
            return True
        if not self._slam_state_client.service_is_ready():
            self._set_mapping_source_error('SLAM 状态服务尚未就绪，无法开始建图')
            return True

        self._stop_patrol_for_mapping()
        self._call_frontier('stop')
        self._publish_manual(0.0, 0.0)
        with self._lock:
            active_payload = self._active_map_payload
        if isinstance(active_payload, dict):
            self._mapping_base_payload = json.loads(
                json.dumps(active_payload)
            )
            with self._lock:
                self._status['map']['mapping_base_available'] = True
        self._pending_mapping_resume = (path, payload)
        self._map_source_mode = 'switching_slam'
        with self._lock:
            self._status['map'].update({
                'source_mode': 'switching_slam',
                'patrol_route_ready': False,
                'localization_ready': False,
                'error': None,
                'error_map_id': None,
            })
            self._status['mapping'].update({
                'state': 'STARTING',
                'detail': '正在退出静态定位并启动新的 SLAM 会话',
                'mode': 'idle',
                'enabled': False,
                'save_error': None,
            })

        request = SetParameters.Request()
        request.parameters = [self._boolean_parameter('tf_broadcast', False)]
        future = self._amcl_params.call_async(request)
        future.add_done_callback(self._on_amcl_disabled_for_mapping)
        return True

    def _on_amcl_disabled_for_mapping(self, future):
        try:
            response = future.result()
            failures = [
                result.reason or 'AMCL 拒绝参数'
                for result in response.results
                if not bool(result.successful)
            ]
            if failures:
                raise RuntimeError('；'.join(failures))
        except Exception as error:
            self._pending_mapping_resume = None
            self._select_map_source('static')
            self._set_mapping_source_error(f'无法暂停 AMCL 定位：{error}')
            return

        with self._lock:
            self._status['mapping']['detail'] = (
                '静态定位已暂停，正在归位并统一建图坐标轴'
            )
            active_payload = self._active_map_payload
        start_x, start_y = self._simulation_origin[:2]
        if isinstance(active_payload, dict):
            waypoints = active_payload.get('waypoints') or []
            if waypoints and isinstance(waypoints[0], dict):
                try:
                    candidate_x = float(waypoints[0].get('x'))
                    candidate_y = float(waypoints[0].get('y'))
                    if math.isfinite(candidate_x) and math.isfinite(candidate_y):
                        start_x, start_y = candidate_x, candidate_y
                except (TypeError, ValueError):
                    pass
        pose_message = String()
        pose_message.data = json.dumps({
            'x': start_x,
            'y': start_y,
            # Every fresh SLAM session starts north-up/east-forward. Reusing
            # an applied route's HOME yaw would rotate simulated scans while
            # odometry restarts at yaw=0, producing a tilted occupancy map.
            'yaw': 0.0,
        })
        self._reset_pose_publisher.publish(pose_message)
        self._mapping_pose_reset_deadline = time.monotonic() + 0.8

    def _continue_mapping_mode_switch(self):
        if self._mapping_pose_reset_deadline is None:
            return
        if self._pending_mapping_resume is None:
            self._mapping_pose_reset_deadline = None
            return
        if time.monotonic() < self._mapping_pose_reset_deadline:
            return
        self._mapping_pose_reset_deadline = None
        if not self._slam_state_client.service_is_ready():
            self._restore_static_after_mapping_switch_error(
                '车辆已归位，但 SLAM 状态服务不可用'
            )
            return
        with self._lock:
            self._status['mapping']['detail'] = (
                '车辆已归位，正在检查 SLAM 生命周期状态'
            )
        future = self._slam_state_client.call_async(GetState.Request())
        future.add_done_callback(self._on_slam_state_for_mapping)

    def _on_slam_state_for_mapping(self, future):
        try:
            state = future.result().current_state
            state_id = int(state.id)
            state_label = str(state.label)
        except Exception as error:
            self._restore_static_after_mapping_switch_error(
                f'无法读取 SLAM 生命周期状态：{error}'
            )
            return
        if state_id == State.PRIMARY_STATE_ACTIVE:
            self._reset_slam_for_mapping()
            return
        if state_id != State.PRIMARY_STATE_INACTIVE:
            self._restore_static_after_mapping_switch_error(
                f'SLAM 当前状态为 {state_label or state_id}，无法开始建图'
            )
            return
        with self._lock:
            self._status['mapping']['detail'] = (
                '正在重新激活 SLAM Toolbox'
            )
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_ACTIVATE
        future = self._slam_lifecycle_client.call_async(request)
        future.add_done_callback(self._on_slam_activated_for_mapping)

    def _on_slam_activated_for_mapping(self, future):
        try:
            response = future.result()
            transition_succeeded = bool(response.success)
        except Exception as error:
            self._restore_static_after_mapping_switch_error(
                f'无法重新启动 SLAM：{error}'
            )
            return
        if transition_succeeded:
            self._reset_slam_for_mapping()
            return

        # Lifecycle transitions are not queued. A concurrent activation can
        # therefore make this request report false even though SLAM is already
        # active. Re-read the state before treating that harmless race as a
        # failed map-mode switch.
        if not self._slam_state_client.service_is_ready():
            self._restore_static_after_mapping_switch_error(
                'SLAM 激活请求未成功，且状态服务不可用'
            )
            return
        state_future = self._slam_state_client.call_async(GetState.Request())
        state_future.add_done_callback(
            self._on_slam_activation_rechecked_for_mapping
        )

    def _on_slam_activation_rechecked_for_mapping(self, future):
        try:
            state = future.result().current_state
            state_id = int(state.id)
            state_label = str(state.label)
        except Exception as error:
            self._restore_static_after_mapping_switch_error(
                f'SLAM 激活后状态复核失败：{error}'
            )
            return
        if state_id == State.PRIMARY_STATE_ACTIVE:
            self._reset_slam_for_mapping()
            return
        self._restore_static_after_mapping_switch_error(
            f'SLAM 无法进入 active 状态，当前为 {state_label or state_id}'
        )

    def _reset_slam_for_mapping(self):
        # Clear the mux's old live-map cache without leaving the currently
        # selected static map. The first post-reset SLAM map will then be the
        # only map eligible for publication.
        if not self._slam_reset_client.service_is_ready():
            now = time.monotonic()
            if self._mapping_reset_wait_deadline is None:
                self._mapping_reset_wait_deadline = now + 5.0
            if now >= self._mapping_reset_wait_deadline:
                self._mapping_reset_wait_deadline = None
                self._restore_static_after_mapping_switch_error(
                    'SLAM 已激活，但重置服务在 5 秒内仍未就绪'
                )
                return
            with self._lock:
                self._status['mapping']['detail'] = (
                    'SLAM 已激活，正在等待重置服务注册'
                )
            return
        self._mapping_reset_wait_deadline = None
        self._clear_live_voxels()
        with self._lock:
            self._status['mapping']['detail'] = (
                'SLAM 已就绪，正在清空上一张实时地图'
            )
        message = String()
        message.data = 'clear_slam'
        self._map_source_publisher.publish(message)
        request = Reset.Request()
        request.pause_new_measurements = False
        future = self._slam_reset_client.call_async(request)
        future.add_done_callback(self._on_slam_reset_for_mapping)

    def _clear_live_voxels(self):
        with self._lock:
            self._voxel_snapshot = None
            self._status['mapping'].update({
                'voxel_count': 0,
                'voxel_source_count': 0,
                'voxel_revision': 0,
            })
        if self._octomap_reset_client.service_is_ready():
            self._octomap_reset_client.call_async(Empty.Request())

    def _continue_slam_reset_when_ready(self):
        if self._mapping_reset_wait_deadline is None:
            return
        if self._pending_mapping_resume is None:
            self._mapping_reset_wait_deadline = None
            return
        self._reset_slam_for_mapping()

    def _on_slam_reset_for_mapping(self, future):
        self._mapping_reset_wait_deadline = None
        try:
            response = future.result()
            if int(response.result) != 0:
                raise RuntimeError(f'SLAM 重置返回错误码 {response.result}')
        except Exception as error:
            self._restore_static_after_mapping_switch_error(
                f'无法清空上一张 SLAM 地图：{error}'
            )
            return

        pending = self._pending_mapping_resume
        self._pending_mapping_resume = None
        self._mapping_map = None
        self._mapping_started_at = None
        self._mapping_session_visible = False
        self._select_map_source('slam')
        with self._lock:
            self._active_map_payload = None
            self._status['map'].update({
                'active_id': None,
                'active_name': '新的实时 SLAM 会话',
                'active_revision': None,
                'active_payload_available': False,
                'mapping_base_available': (
                    self._mapping_base_payload is not None
                ),
                'source_resolution': None,
                'navigation_resolution': None,
                'resampled': False,
                'localization_ready': True,
                'patrol_route_ready': False,
                'error': None,
                'error_map_id': None,
            })
            self._status['mapping'].update({
                'state': 'IDLE',
                'detail': '新的 SLAM 会话已就绪',
                'mode': 'idle',
                'enabled': False,
                'duration_seconds': 0,
                'map_revision': 0,
                'coverage': 0.0,
                'known_cells': 0,
                'total_cells': 0,
            })
        if pending is not None:
            self._process_mapping_command(*pending)

    def _restore_static_after_mapping_switch_error(self, error):
        self._mapping_pose_reset_deadline = None
        self._mapping_reset_wait_deadline = None
        self._pending_mapping_resume = None
        if not self._slam_lifecycle_client.service_is_ready():
            self._enable_amcl_after_mapping_switch_error(error)
            return
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_DEACTIVATE
        future = self._slam_lifecycle_client.call_async(request)
        future.add_done_callback(
            lambda _completed:
                self._enable_amcl_after_mapping_switch_error(error)
        )

    def _enable_amcl_after_mapping_switch_error(self, error):
        if not self._amcl_params.service_is_ready():
            self._select_map_source('static')
            self._set_mapping_source_error(error)
            return
        request = SetParameters.Request()
        request.parameters = [self._boolean_parameter('tf_broadcast', True)]
        future = self._amcl_params.call_async(request)
        future.add_done_callback(
            lambda _completed: self._finish_mapping_switch_rollback(error)
        )

    def _finish_mapping_switch_rollback(self, error):
        self._select_map_source('static')
        with self._lock:
            self._status['map']['localization_ready'] = True
        self._set_mapping_source_error(error)

    def _set_mapping_source_error(self, error):
        with self._lock:
            self._status['mapping'].update({
                'state': 'ERROR',
                'detail': str(error),
                'mode': 'idle',
                'enabled': False,
                'save_error': str(error),
            })

    def _call_frontier(self, action):
        client = self._frontier_clients[action]
        if not client.service_is_ready():
            with self._lock:
                self._status['mapping'].update({
                    'state': 'ERROR',
                    'detail': f'前沿探索服务尚未就绪：{action}',
                    'save_error': '自主探索服务不可用',
                })
            return False
        client.call_async(Trigger.Request())
        return True

    def _discard_mapping_session(self):
        self._mapping_started_at = None
        self._mapping_mode = 'idle'
        self._mapping_reset_in_progress = True
        self._mapping_session_visible = False
        self._clear_live_voxels()
        with self._lock:
            self._mapping_map = None
            self._status['mapping'].update({
                'state': 'RESETTING',
                'detail': '正在清空 SLAM 会话并让车辆返回原点',
                'mode': 'idle',
                'enabled': False,
                'duration_seconds': 0,
                'map_revision': 0,
                'coverage': 0.0,
                'known_cells': 0,
                'total_cells': 0,
                'goals_reached': 0,
                'goals_failed': 0,
                'frontier_clusters': 0,
                'blacklisted_goals': 0,
                'save_error': None,
                'saved_map': None,
            })
            self._status['map']['patrol_route_ready'] = False
        if not self._slam_reset_client.service_is_ready():
            self._mapping_reset_in_progress = False
            with self._lock:
                self._status['mapping'].update({
                    'state': 'ERROR',
                    'detail': 'SLAM 重置服务尚未就绪，当前地图未清空',
                    'save_error': '无法连接 /slam_toolbox/reset',
                })
            return

        request = Reset.Request()
        request.pause_new_measurements = False
        future = self._slam_reset_client.call_async(request)
        future.add_done_callback(self._on_mapping_session_reset)

    def _on_mapping_session_reset(self, future):
        error = None
        try:
            response = future.result()
            if int(response.result) != 0:
                error = f'SLAM 重置返回错误码 {response.result}'
        except Exception as exception:
            error = str(exception)
        if error:
            self._mapping_reset_in_progress = False
            with self._lock:
                self._status['mapping'].update({
                    'state': 'ERROR',
                    'detail': error,
                    'save_error': error,
                })
            return

        pose_message = String()
        pose_message.data = json.dumps({
            'x': self._simulation_origin[0],
            'y': self._simulation_origin[1],
            'yaw': self._simulation_origin[2],
        })
        self._reset_pose_publisher.publish(pose_message)
        with self._lock:
            self._status['mapping'].update({
                'state': 'IDLE',
                'detail': '本次会话已放弃，车辆已返回原点，可重新开始建图',
                'mode': 'idle',
                'enabled': False,
                'duration_seconds': 0,
                'map_revision': 0,
                'coverage': 0.0,
                'known_cells': 0,
                'total_cells': 0,
                'goals_reached': 0,
                'goals_failed': 0,
                'frontier_clusters': 0,
                'blacklisted_goals': 0,
                'save_error': None,
                'saved_map': None,
            })
        self._mapping_reset_in_progress = False

    def _save_mapping_map(self, display_name):
        if not display_name:
            with self._lock:
                self._status['mapping'].update({
                    'state': 'ERROR',
                    'detail': '地图名称不能为空',
                    'save_error': '地图名称不能为空',
                })
            return
        frontier_stop = self._frontier_clients['stop']
        if frontier_stop.service_is_ready():
            frontier_stop.call_async(Trigger.Request())
        self._publish_manual(0.0, 0.0)
        if not self._save_map_client.service_is_ready():
            with self._lock:
                self._status['mapping'].update({
                    'state': 'ERROR',
                    'detail': 'SLAM 地图保存服务尚未就绪',
                    'save_error': '无法连接 /slam_toolbox/save_map',
                })
            return

        timestamp = int(time.time() * 1000)
        safe_prefix = re.sub(
            r'[^A-Za-z0-9_-]+',
            '-',
            display_name,
        ).strip('-')[:48] or 'mapping'
        safe_id = f'{safe_prefix}-{timestamp}'
        self._map_storage_dir.mkdir(parents=True, exist_ok=True)
        target = self._map_storage_dir / safe_id
        request = SaveMap.Request()
        request.name = String(data=str(target))
        with self._lock:
            self._status['mapping'].update({
                'state': 'SAVING',
                'detail': '正在写入二维栅格地图与 SLAM 元数据',
                'enabled': False,
                'save_error': None,
            })
        future = self._save_map_client.call_async(request)
        future.add_done_callback(
            lambda completed:
                self._on_mapping_map_saved(
                    completed,
                    safe_id,
                    display_name,
                )
        )

    def _on_mapping_map_saved(self, future, map_id, display_name):
        error = None
        try:
            response = future.result()
            if int(response.result) != 0:
                error = f'SLAM 保存返回错误码 {response.result}'
        except Exception as exception:
            error = str(exception)
        yaml_path = self._map_storage_dir / f'{map_id}.yaml'
        pgm_path = self._map_storage_dir / f'{map_id}.pgm'
        if error is None and (not yaml_path.is_file() or not pgm_path.is_file()):
            error = '保存服务返回成功，但地图文件不完整'
        if error:
            with self._lock:
                self._status['mapping'].update({
                    'state': 'ERROR',
                    'detail': error,
                    'save_error': error,
                })
            return

        with self._lock:
            map_snapshot = dict(self._mapping_map or {})
            self._status['mapping'].update({
                'state': 'SAVING',
                'detail': '二维栅格已保存，正在写入三维 OctoMap 体素文件',
            })
        threading.Thread(
            target=self._finalize_mapping_map_save,
            args=(map_id, display_name, yaml_path, pgm_path, map_snapshot),
            daemon=True,
        ).start()

    @staticmethod
    def _mapping_editor_map(record, map_snapshot, has_3d, voxel_size_bytes):
        width = int(map_snapshot.get('width', 0))
        height = int(map_snapshot.get('height', 0))
        total = max(0, width * height)
        values = []
        runs = map_snapshot.get('runs') or []
        for index in range(0, len(runs) - 1, 2):
            value = int(runs[index])
            count = max(0, int(runs[index + 1]))
            values.extend([value] * min(count, max(0, total - len(values))))
            if len(values) >= total:
                break
        if len(values) < total:
            values.extend([-1] * (total - len(values)))

        packed = bytearray((total + 7) // 8)
        for image_row in range(height):
            source_row = height - 1 - image_row
            for column in range(width):
                source_index = source_row * width + column
                value = values[source_index]
                # Unknown space is kept occupied in the navigation editor. A
                # patrol point can only be placed in an actually observed free
                # cell, never in an unexplored area.
                if value < 0 or value >= 65:
                    target_index = image_row * width + column
                    packed[target_index >> 3] |= 1 << (target_index & 7)

        resolution = float(map_snapshot.get('resolution', 0.05))
        origin = map_snapshot.get('origin') or {}
        origin_x = float(origin.get('x', 0.0))
        origin_y = float(origin.get('y', 0.0))
        timestamp = record['created_at']
        return {
            'id': record['id'],
            'name': record['name'],
            'description': (
                f'自主 SLAM 地图 · 覆盖 {record["coverage"]:.1f}% · '
                + ('包含 2D 栅格与 3D OctoMap' if has_3d else '包含 2D 栅格')
            ),
            'source': 'slam',
            'bounds': {
                'minX': origin_x,
                'minY': origin_y,
                'width': width * resolution,
                'height': height * resolution,
            },
            'resolution': resolution,
            'objects': [],
            'waypoints': [],
            'occupancy': {
                'width': width,
                'height': height,
                'resolution': resolution,
                'originX': origin_x,
                'originY': origin_y,
                'data': base64.b64encode(bytes(packed)).decode('ascii'),
            },
            'voxel': {
                'available': bool(has_3d),
                'format': 'octomap-ot',
                'sizeBytes': int(voxel_size_bytes),
            },
            'createdAt': timestamp,
            'updatedAt': timestamp,
        }

    def _finalize_mapping_map_save(
        self,
        map_id,
        display_name,
        yaml_path,
        pgm_path,
        map_snapshot,
    ):
        voxel_path = self._map_storage_dir / f'{map_id}.ot'
        try:
            completed = subprocess.run(
                [
                    'ros2',
                    'run',
                    'octomap_server',
                    'octomap_saver_node',
                    '--ros-args',
                    '-p',
                    f'octomap_path:={voxel_path}',
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=20.0,
            )
            has_3d = (
                completed.returncode == 0
                and voxel_path.is_file()
                and voxel_path.stat().st_size > 0
            )
        except (OSError, subprocess.TimeoutExpired):
            has_3d = False
        if self._require_3d_map_on_save and not has_3d:
            for partial_path in (yaml_path, pgm_path, voxel_path):
                try:
                    partial_path.unlink()
                except FileNotFoundError:
                    pass
                except OSError:
                    pass
            with self._lock:
                self._status['mapping'].update({
                    'state': 'ERROR',
                    'detail': (
                        '二维地图已生成，但三维 OctoMap 未能保存；'
                        '请确认 /octomap_binary 正在发布后重试'
                    ),
                    'save_error': '三维 OctoMap 数据不可用，未创建不完整地图',
                })
            return
        voxel_size_bytes = voxel_path.stat().st_size if has_3d else 0
        size_bytes = (
            yaml_path.stat().st_size
            + pgm_path.stat().st_size
            + voxel_size_bytes
        )
        record = {
            'id': map_id,
            'name': display_name,
            'created_at': time.strftime(
                '%Y-%m-%dT%H:%M:%S%z',
                time.localtime(),
            ),
            'size_bytes': size_bytes,
            'resolution': float(map_snapshot.get('resolution', 0.05)),
            'width': int(map_snapshot.get('width', 0)),
            'height': int(map_snapshot.get('height', 0)),
            'coverage': float(map_snapshot.get('coverage', 0.0)),
            'has_2d': True,
            'has_3d': has_3d,
            'voxel_size_bytes': voxel_size_bytes,
        }
        record['editor_map'] = self._mapping_editor_map(
            record,
            map_snapshot,
            has_3d,
            voxel_size_bytes,
        )
        try:
            catalog = [
                item for item in self.saved_mapping_maps()
                if item.get('id') != map_id
            ]
            catalog.insert(0, record)
            self._mapping_catalog_path.write_text(
                json.dumps(catalog, ensure_ascii=False, indent=2),
                encoding='utf-8',
            )
        except OSError as exception:
            with self._lock:
                self._status['mapping'].update({
                    'state': 'ERROR',
                    'detail': f'地图文件已生成，但仓库索引写入失败：{exception}',
                    'save_error': str(exception),
                })
            return
        self._mapping_started_at = None
        self._mapping_mode = 'idle'
        with self._lock:
            self._status['mapping'].update({
                'state': 'SAVED',
                'detail': (
                    f'“{display_name}”的二维栅格与三维体素图已保存'
                    if has_3d
                    else f'“{display_name}”的二维栅格已保存；当前未收到三维体素流'
                ),
                'mode': 'idle',
                'duration_seconds': 0,
                'save_error': None,
                'saved_map': record,
            })

    def _delete_mapping_map(self, map_id):
        catalog = self.saved_mapping_maps()
        record = next(
            (item for item in catalog if item.get('id') == map_id),
            None,
        )
        if record is None:
            return
        try:
            for suffix in ('.yaml', '.pgm', '.ot'):
                path = self._map_storage_dir / f'{map_id}{suffix}'
                try:
                    path.unlink()
                except FileNotFoundError:
                    pass
            remaining = [
                item for item in catalog if item.get('id') != map_id
            ]
            self._mapping_catalog_path.write_text(
                json.dumps(remaining, ensure_ascii=False, indent=2),
                encoding='utf-8',
            )
        except OSError as exception:
            with self._lock:
                self._status['mapping'].update({
                    'state': 'ERROR',
                    'detail': f'删除地图失败：{exception}',
                })

    def _deploy_mapping_map(self, map_id):
        record = next(
            (
                item for item in self.saved_mapping_maps()
                if item.get('id') == map_id
            ),
            None,
        )
        if record is None:
            with self._lock:
                self._status['mapping'].update({
                    'state': 'ERROR',
                    'detail': '待部署地图不存在或文件不完整',
                })
            return
        if not self._map_load_client.service_is_ready():
            with self._lock:
                self._status['mapping'].update({
                    'state': 'ERROR',
                    'detail': '请先进入导航模式，再部署已保存地图',
                })
            return
        frontier_stop = self._frontier_clients['stop']
        if frontier_stop.service_is_ready():
            frontier_stop.call_async(Trigger.Request())
        self._publish_manual(0.0, 0.0)
        request = LoadMap.Request()
        request.map_url = str(
            self._map_storage_dir / f'{map_id}.yaml'
        )
        with self._lock:
            self._status['mapping'].update({
                'state': 'DEPLOYING',
                'detail': f'正在把“{record["name"]}”加载到导航内存',
            })
        future = self._map_load_client.call_async(request)
        future.add_done_callback(
            lambda completed:
                self._on_mapping_map_deployed(completed, record)
        )

    def _on_mapping_map_deployed(self, future, record):
        error = None
        try:
            response = future.result()
            if int(response.result) != 0:
                error = f'Nav2 加载地图返回错误码 {response.result}'
        except Exception as exception:
            error = str(exception)
        with self._lock:
            if error:
                self._status['mapping'].update({
                    'state': 'ERROR',
                    'detail': error,
                })
            else:
                self._status['mapping'].update({
                    'state': 'DEPLOYED',
                    'detail': (
                        f'“{record["name"]}”已加载；'
                        '请确认机器人初始位姿后开始导航'
                    ),
                })
                self._status['map'].update({
                    'active_id': record['id'],
                    'active_name': record['name'],
                    'localization_ready': False,
                    # Deployment only loads the saved occupancy grid. The
                    # operator still has to apply that map's audited route.
                    'patrol_route_ready': False,
                    'error': None,
                })

    @staticmethod
    def _scenario_grid(payload):
        bounds = payload.get('bounds') or {}
        origin_x = float(bounds.get('minX', -8.0))
        origin_y = float(bounds.get('minY', -6.0))
        world_width = max(2.0, min(float(bounds.get('width', 16.0)), 500.0))
        world_height = max(2.0, min(float(bounds.get('height', 12.0)), 500.0))
        requested_resolution = max(
            0.02,
            min(float(payload.get('resolution', 0.25)), 5.0),
        )
        occupancy = payload.get('occupancy')

        if isinstance(occupancy, dict) and occupancy.get('data'):
            width = max(1, min(int(occupancy.get('width', 0)), 2000))
            height = max(1, min(int(occupancy.get('height', 0)), 2000))
            resolution = max(
                0.001,
                min(float(occupancy.get('resolution', requested_resolution)), 5.0),
            )
            origin_x = float(occupancy.get('originX', origin_x))
            origin_y = float(occupancy.get('originY', origin_y))
            packed = base64.b64decode(str(occupancy['data']), validate=True)
            if len(packed) * 8 < width * height:
                raise ValueError('导入地图的占用栅格数据不完整')
            occupied = [
                bool((packed[index >> 3] >> (index & 7)) & 1)
                for index in range(width * height)
            ]
            resolution, width, height, occupied = _resample_occupancy(
                occupied,
                width,
                height,
                resolution,
            )
        else:
            resolution = max(
                min(requested_resolution, NAVIGATION_GRID_TARGET_RESOLUTION),
                world_width / MAX_NAVIGATION_GRID_DIMENSION,
                world_height / MAX_NAVIGATION_GRID_DIMENSION,
            )
            width = max(2, int(math.ceil(world_width / resolution)))
            height = max(2, int(math.ceil(world_height / resolution)))
            occupied = [False] * (width * height)

        # The Gazebo perimeter walls extend 0.50 m inward from map bounds.
        # Mark the same thickness in Nav2 instead of a single grid cell, or
        # the global planner can legally create a path through physical wall.
        perimeter_cells = max(
            1,
            int(math.ceil(GAZEBO_BOUNDARY_WALL_INSET / resolution)),
        )
        for row in range(height):
            for column in range(width):
                if (
                    row < perimeter_cells
                    or row >= height - perimeter_cells
                    or column < perimeter_cells
                    or column >= width - perimeter_cells
                ):
                    occupied[row * width + column] = True

        objects = payload.get('objects') or []
        if not isinstance(objects, list) or len(objects) > 500:
            raise ValueError('地图场景元素格式无效或数量超过 500')
        for item in objects:
            if not isinstance(item, dict):
                continue
            center_x = float(item.get('x', 0.0))
            center_y = float(item.get('y', 0.0))
            object_width = max(0.1, min(float(item.get('width', 1.0)), 50.0))
            object_depth = max(0.1, min(float(item.get('depth', 1.0)), 50.0))
            min_column = max(0, int(math.floor(
                (center_x - object_width / 2.0 - origin_x) / resolution
            )))
            max_column = min(width - 1, int(math.floor(
                (center_x + object_width / 2.0 - origin_x) / resolution
            )))
            min_grid_y = max(0, int(math.floor(
                (center_y - object_depth / 2.0 - origin_y) / resolution
            )))
            max_grid_y = min(height - 1, int(math.floor(
                (center_y + object_depth / 2.0 - origin_y) / resolution
            )))
            for grid_y in range(min_grid_y, max_grid_y + 1):
                row = height - 1 - grid_y
                for column in range(min_column, max_column + 1):
                    occupied[row * width + column] = True

        return origin_x, origin_y, resolution, width, height, occupied

    def _validated_scenario_grid(self, payload):
        grid = self._scenario_grid(payload)
        waypoint_issues = _scenario_waypoint_issues(payload, grid)
        if waypoint_issues:
            raise ValueError('不安全巡检点：' + '；'.join(waypoint_issues))
        connectivity_issues = _scenario_route_connectivity_issues(payload, grid)
        if connectivity_issues:
            raise ValueError('巡检路线不连通：' + '；'.join(connectivity_issues))
        return grid

    def _write_scenario_map(self, payload, grid=None):
        if grid is None:
            grid = self._validated_scenario_grid(payload)
        (
            origin_x, origin_y, resolution, width, height, occupied,
        ) = grid
        self._map_storage_dir.mkdir(parents=True, exist_ok=True)
        safe_id = re.sub(
            r'[^A-Za-z0-9_.-]+',
            '-',
            str(payload.get('id', 'scenario-map')),
        ).strip('-') or 'scenario-map'
        pgm_path = self._map_storage_dir / f'{safe_id}.pgm'
        yaml_path = self._map_storage_dir / f'{safe_id}.yaml'
        rows = []
        for row in range(height):
            start = row * width
            rows.append(' '.join(
                '0' if value else '254'
                for value in occupied[start:start + width]
            ))
        pgm_path.write_text(
            'P2\n'
            '# Generated by patrol_robot_web_bridge map management\n'
            f'{width} {height}\n255\n' + '\n'.join(rows) + '\n',
            encoding='ascii',
        )
        yaml_path.write_text(
            f'image: {pgm_path.name}\n'
            'mode: trinary\n'
            f'resolution: {resolution:.6f}\n'
            f'origin: [{origin_x:.6f}, {origin_y:.6f}, 0.0]\n'
            'negate: 0\n'
            'occupied_thresh: 0.65\n'
            'free_thresh: 0.25\n',
            encoding='utf-8',
        )
        return yaml_path

    def _persist_saved_slam_editor_map(self, payload):
        """Store patrol-point edits beside a saved SLAM map.

        The SLAM map catalog is the shared source for Mapping Mode and Map
        Management. Persisting the editor payload here keeps new patrol points
        after a browser refresh and makes them available when Navigation Mode
        later loads the saved YAML/OctoMap pair.
        """
        map_id = str(payload.get('id', '')).strip()
        catalog = self.saved_mapping_maps()
        record = next(
            (item for item in catalog if str(item.get('id', '')) == map_id),
            None,
        )
        if record is None:
            return None

        revision = str(payload.get('revision', '')).strip()
        previous = record.get('editor_map')
        editor_map = dict(previous) if isinstance(previous, dict) else {}
        for key in (
            'id',
            'name',
            'description',
            'source',
            'seed',
            'bounds',
            'resolution',
            'objects',
            'waypoints',
            'occupancy',
            'voxel',
            'createdAt',
        ):
            if key in payload:
                editor_map[key] = payload[key]
        editor_map['id'] = map_id
        editor_map['source'] = 'slam'
        editor_map['updatedAt'] = (
            revision
            or str(editor_map.get('updatedAt', '')).strip()
            or time.strftime('%Y-%m-%dT%H:%M:%S%z', time.localtime())
        )
        record['name'] = str(payload.get('name') or record.get('name') or map_id)
        record['editor_map'] = editor_map
        self._mapping_catalog_path.write_text(
            json.dumps(catalog, ensure_ascii=False, indent=2),
            encoding='utf-8',
        )
        return record

    def _activate_saved_slam_map_in_mapping_mode(
        self,
        payload,
        loop_count,
        source_resolution,
    ):
        """Apply route metadata while live SLAM owns /map.

        There is deliberately no /map_server/load_map service in Mapping Mode:
        SLAM Toolbox is already publishing the map used by the exploration
        planners. Reloading that same map would create two /map owners and
        invalidate the active SLAM session. In this mode "apply" therefore
        persists the patrol route and marks the live map revision active.
        """
        try:
            record = self._persist_saved_slam_editor_map(payload)
        except OSError as error:
            map_id = str(payload.get('id', '')).strip() or None
            detail = f'巡检点保存失败：{error}'
            with self._lock:
                self._status['map'].update({
                    'transitioning': False,
                    'localization_ready': True,
                    'error': detail,
                    'error_map_id': map_id,
                })
                self._status['mapping'].update({'detail': detail})
            return True
        if record is None:
            return False

        map_id = str(payload['id'])
        map_name = str(payload['name'])
        map_revision = str(payload.get('revision', '')).strip() or None
        route = {
            'frame_id': str(payload.get('frame_id', 'map')),
            'loop_count': loop_count,
            'map_id': map_id,
            'map_revision': map_revision,
            'waypoints': payload['waypoints'],
        }

        self._publish_manual(0.0, 0.0)
        self._clear_navigation_path()
        stop_client = self._patrol_clients['stop']
        if stop_client.service_is_ready():
            stop_client.call_async(Trigger.Request())
        frontier_stop = self._frontier_clients['stop']
        if frontier_stop.service_is_ready():
            frontier_stop.call_async(Trigger.Request())

        message = String()
        message.data = json.dumps(route, ensure_ascii=False)
        self._waypoint_publisher.publish(message)
        self._pending_map_status = None
        self._pending_map_route = None
        self._pending_map_payload = None
        self._expected_scene_map_id = None
        with self._lock:
            self._active_map_payload = payload
            self._status['map'].update({
                'active_id': map_id,
                'active_name': map_name,
                'active_revision': map_revision,
                'pending_id': None,
                'pending_name': None,
                'pending_revision': None,
                'active_payload_available': True,
                'source_resolution': source_resolution,
                'navigation_resolution': source_resolution,
                'resampled': False,
                'pending_navigation_resolution': None,
                'transitioning': False,
                'localization_ready': True,
                'patrol_route_ready': True,
                'error': None,
                'error_map_id': None,
            })
            self._status['mapping'].update({
                'detail': (
                    f'“{map_name}”的巡检路线已应用；'
                    '可切换到日常巡检导航并开始巡检'
                ),
            })
        self.get_logger().info(
            f'已应用 SLAM 地图“{map_name}”的 {len(route["waypoints"])} 个路线点；'
            '当前由 SLAM Toolbox 提供 /map，可直接启动巡检'
        )
        return True

    def _activate_map(self, payload):
        map_id = str(payload.get('id', '')).strip()
        map_name = str(payload.get('name', '')).strip()
        map_revision = str(payload.get('revision', '')).strip() or None
        if not map_id or not map_name:
            with self._lock:
                self._status['map'].update({
                    'transitioning': False,
                    'error': '地图 ID 或名称不能为空',
                    'error_map_id': map_id or None,
                })
            return

        with self._lock:
            if self._status['map'].get('transitioning'):
                self.get_logger().warning('当前地图尚未完成切换，已忽略新的应用请求')
                return

        # Reject stale/direct API clients before stopping the current patrol or
        # changing the active-map status. The UI performs the same validation.
        try:
            payload = _with_route_headings(payload)
            scenario_grid = self._validated_scenario_grid(payload)
            loop_count = max(1, min(1000, int(payload.get('loop_count', 1))))
        except (ValueError, TypeError, binascii.Error) as error:
            with self._lock:
                self._status['map'].update({
                    'transitioning': False,
                    'localization_ready': True,
                    'error': f'地图数据无效：{error}',
                    'error_map_id': map_id,
                })
            self.get_logger().error(f'拒绝应用地图“{map_name}”：{error}')
            return

        occupancy = payload.get('occupancy') or {}
        source_resolution = float(
            occupancy.get('resolution', payload.get('resolution', 0.25))
        )
        navigation_resolution = float(scenario_grid[2])
        was_resampled = navigation_resolution < source_resolution - 1.0e-9

        # A saved SLAM map is edited while SLAM Toolbox still owns /map. It is
        # valid to save and activate its route metadata without the static Nav2
        # map service, which only exists after switching to Navigation Mode.
        if (
            str(payload.get('source', '')).strip().lower() == 'slam'
            and not self._map_load_client.service_is_ready()
            and self._activate_saved_slam_map_in_mapping_mode(
                payload,
                loop_count,
                source_resolution,
            )
        ):
            return

        self._publish_manual(0.0, 0.0)
        self._clear_navigation_path()
        stop_client = self._patrol_clients['stop']
        if stop_client.service_is_ready():
            stop_client.call_async(Trigger.Request())
        self._expected_scene_map_id = map_id
        self._scene_ready = False
        self._scene_error = None
        self._localization_pose_ready = False
        self._last_map_initial_pose = None
        self._pending_map_status = {
            'id': map_id,
            'name': map_name,
            'revision': map_revision,
            'source_resolution': source_resolution,
            'navigation_resolution': navigation_resolution,
            'resampled': was_resampled,
        }
        self._pending_map_route = {
            'frame_id': str(payload.get('frame_id', 'map')),
            'loop_count': loop_count,
            'map_id': map_id,
            'map_revision': map_revision,
            'waypoints': payload['waypoints'],
        }
        self._pending_map_payload = payload
        with self._lock:
            self._status['map'].update({
                'pending_id': map_id,
                'pending_name': map_name,
                'pending_revision': map_revision,
                'pending_navigation_resolution': navigation_resolution,
                'transitioning': True,
                'localization_ready': False,
                'error': None,
                'error_map_id': None,
            })

        try:
            yaml_path = self._write_scenario_map(payload, scenario_grid)
            scenario_message = String()
            scenario_message.data = json.dumps(payload, ensure_ascii=False)
            self._map_scenario_publisher.publish(scenario_message)
        except (ValueError, TypeError, OSError, binascii.Error) as error:
            self._abort_map_transition(f'地图数据无效：{error}')
            return

        if not self._map_load_client.service_is_ready():
            self._abort_map_transition('地图已保存，但 Nav2 地图服务尚未就绪')
            return

        request = LoadMap.Request()
        request.map_url = str(yaml_path)
        future = self._map_load_client.call_async(request)
        future.add_done_callback(self._on_map_loaded)
        self.get_logger().warning(
            f'正在切换导航地图：{map_name} ({yaml_path})'
        )

    def _publish_map_source_state(self):
        if self._map_source_mode not in ('slam', 'static'):
            return
        message = String()
        message.data = self._map_source_mode
        self._map_source_publisher.publish(message)

    def _select_map_source(self, source, reset_slam_cache=False):
        self._map_source_mode = source
        with self._lock:
            self._status['map']['source_mode'] = source
        message = String()
        message.data = (
            'slam_reset'
            if source == 'slam' and reset_slam_cache
            else source
        )
        self._map_source_publisher.publish(message)

    def _begin_static_localization_switch(self):
        if self._map_source_mode == 'static':
            self._initial_pose_repeats = 3
            self.get_logger().info('静态地图已加载，正在重新初始化 AMCL')
            return
        if not self._slam_lifecycle_client.service_is_ready():
            self._abort_map_transition('SLAM 生命周期服务尚未就绪，无法切换静态地图')
            return
        if not self._slam_state_client.service_is_ready():
            self._abort_map_transition('SLAM 状态服务尚未就绪，无法切换静态地图')
            return
        if not self._amcl_params.service_is_ready():
            self._abort_map_transition('AMCL 参数服务尚未就绪，无法切换静态地图')
            return

        self._map_source_mode = 'switching_static'
        with self._lock:
            self._status['map']['source_mode'] = 'switching_static'
        future = self._slam_state_client.call_async(GetState.Request())
        future.add_done_callback(self._on_slam_state_for_static)
        self.get_logger().info('静态地图已加载，正在暂停 SLAM 定位输出')

    def _on_slam_state_for_static(self, future):
        try:
            state = future.result().current_state
            state_id = int(state.id)
            state_label = str(state.label)
        except Exception as error:
            self._abort_map_transition(
                f'无法读取 SLAM 生命周期状态：{error}'
            )
            return
        if state_id == State.PRIMARY_STATE_INACTIVE:
            self._enable_amcl_for_static()
            return
        if state_id != State.PRIMARY_STATE_ACTIVE:
            self._abort_map_transition(
                f'SLAM 当前状态为 {state_label or state_id}，无法切换静态地图'
            )
            return
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_DEACTIVATE
        future = self._slam_lifecycle_client.call_async(request)
        future.add_done_callback(self._on_slam_deactivated_for_static)

    def _on_slam_deactivated_for_static(self, future):
        try:
            response = future.result()
            transition_succeeded = bool(response.success)
        except Exception as error:
            self._select_map_source('slam')
            self._abort_map_transition(f'无法暂停 SLAM：{error}')
            return
        if transition_succeeded:
            self._enable_amcl_for_static()
            return

        # Accept an already-inactive node, but never enable AMCL until the
        # lifecycle state has been verified. This prevents two map->odom TF
        # publishers from becoming active together.
        if not self._slam_state_client.service_is_ready():
            self._select_map_source('slam')
            self._abort_map_transition(
                'SLAM 停用请求未成功，且状态服务不可用'
            )
            return
        state_future = self._slam_state_client.call_async(GetState.Request())
        state_future.add_done_callback(
            self._on_slam_deactivation_rechecked_for_static
        )

    def _on_slam_deactivation_rechecked_for_static(self, future):
        try:
            state = future.result().current_state
            state_id = int(state.id)
            state_label = str(state.label)
        except Exception as error:
            self._select_map_source('slam')
            self._abort_map_transition(
                f'SLAM 停用后状态复核失败：{error}'
            )
            return
        if state_id == State.PRIMARY_STATE_INACTIVE:
            self._enable_amcl_for_static()
            return
        self._select_map_source('slam')
        self._abort_map_transition(
            f'SLAM 无法进入 inactive 状态，当前为 {state_label or state_id}'
        )

    def _enable_amcl_for_static(self):
        request = SetParameters.Request()
        request.parameters = [self._boolean_parameter('tf_broadcast', True)]
        parameter_future = self._amcl_params.call_async(request)
        parameter_future.add_done_callback(self._on_amcl_enabled_for_static)

    def _on_amcl_enabled_for_static(self, future):
        try:
            response = future.result()
            failures = [
                result.reason or 'AMCL 拒绝参数'
                for result in response.results
                if not bool(result.successful)
            ]
            if failures:
                raise RuntimeError('；'.join(failures))
        except Exception as error:
            self._recover_slam_after_static_switch_failure(
                f'无法启用 AMCL 定位：{error}'
            )
            return

        self._select_map_source('static')
        self._initial_pose_repeats = 3
        self.get_logger().info(
            '已切换到 Nav2 静态地图 + AMCL，正在校准车辆初始位姿'
        )

    def _recover_slam_after_static_switch_failure(self, error):
        """Restore the live mapping TF owner when static localization fails."""
        if not self._slam_lifecycle_client.service_is_ready():
            self._select_map_source('slam')
            self._abort_map_transition(error)
            return
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_ACTIVATE
        future = self._slam_lifecycle_client.call_async(request)
        future.add_done_callback(
            lambda completed:
                self._on_slam_recovered_after_static_switch_failure(
                    completed,
                    error,
                )
        )

    def _on_slam_recovered_after_static_switch_failure(self, future, error):
        recovery_error = None
        try:
            response = future.result()
            if not bool(response.success):
                recovery_error = 'SLAM Toolbox 拒绝恢复 active 状态'
        except Exception as exception:
            recovery_error = str(exception)
        self._select_map_source('slam')
        detail = (
            f'{error}；实时 SLAM 恢复失败：{recovery_error}'
            if recovery_error
            else error
        )
        self._abort_map_transition(detail)

    def _on_map_loaded(self, future):
        error = None
        try:
            response = future.result()
            if int(response.result) != 0:
                error = f'Nav2 返回错误码 {response.result}'
        except Exception as exception:
            error = str(exception)
        error = error or self._scene_error
        if self._pending_map_status is None:
            return
        with self._lock:
            self._status['map'].update({
                'transitioning': not bool(error),
                'localization_ready': False,
                'error': error,
            })
        if error:
            self._abort_map_transition(error)
            self.get_logger().error(f'地图切换失败：{error}')
        else:
            # Static maps use the original navigation stack: map_server +
            # AMCL + the same planner/controller/patrol manager. SLAM remains
            # installed but stops publishing map->odom before AMCL takes over.
            self._begin_static_localization_switch()

    def _publish_map_initial_pose(self):
        if self._initial_pose_repeats <= 0:
            return
        with self._lock:
            odom_pose = self._latest_odom_pose
            ground_truth_pose = self._latest_ground_truth_pose
            ground_truth_updated = self._last_ground_truth_pose_update
            pending_route = self._pending_map_route
            scene_ready = self._scene_ready
        # Map loading and Gazebo scene rebuilding run independently. Never
        # consume the finite AMCL seed attempts before Gazebo confirms that the
        # model has actually reached the new HOME pose.
        if pending_route is not None and not scene_ready:
            return
        ground_truth_fresh = bool(
            ground_truth_pose is not None
            and ground_truth_updated is not None
            and time.monotonic() - ground_truth_updated <= 1.0
        )
        route_waypoints = (
            pending_route.get('waypoints', [])
            if isinstance(pending_route, dict) else []
        )
        route_home = route_waypoints[0] if route_waypoints else None
        if odom_pose is None and not ground_truth_fresh and route_home is None:
            return

        if isinstance(route_home, dict):
            # gazebo_scene_sync teleports to this exact pose before publishing
            # scene_ready. Using the shared HOME target also avoids a race with
            # the first ground-truth odometry sample after teleportation.
            map_x = float(route_home['x'])
            map_y = float(route_home['y'])
            map_yaw = float(route_home.get('yaw', 0.0))
        elif ground_truth_fresh:
            # The simulator has just teleported the model to the selected
            # map's HOME pose. Seed AMCL from that exact pose once, then let
            # wheel odometry + IMU + EKF drive all subsequent motion.
            map_x, map_y, map_yaw = ground_truth_pose
        elif self._odom_pose_is_world:
            # Gazebo OdometryPublisher reports the model's world pose even
            # though the message frame is named odom.
            odom_x, odom_y, odom_yaw = odom_pose
            map_x, map_y, map_yaw = odom_x, odom_y, odom_yaw
        else:
            # The lightweight simulator integrates odometry from zero at its
            # configured initial map pose.
            odom_x, odom_y, odom_yaw = odom_pose
            origin_x, origin_y, origin_yaw = self._simulation_origin
            cosine = math.cos(origin_yaw)
            sine = math.sin(origin_yaw)
            map_x = origin_x + cosine * odom_x - sine * odom_y
            map_y = origin_y + sine * odom_x + cosine * odom_y
            map_yaw = origin_yaw + odom_yaw

        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = map_x
        message.pose.pose.position.y = map_y
        message.pose.pose.orientation.z = math.sin(map_yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(map_yaw / 2.0)
        message.pose.covariance[0] = 0.04
        message.pose.covariance[7] = 0.04
        message.pose.covariance[35] = 0.03
        self._initial_pose_publisher.publish(message)
        self._last_map_initial_pose = (map_x, map_y, map_yaw)
        self._initial_pose_repeats -= 1

        if self._initial_pose_repeats == 0:
            self._localization_pose_ready = True
            self._finish_map_transition_if_ready()

    def _finish_map_transition_if_ready(self):
        if not self._scene_ready or not self._localization_pose_ready:
            return
        pending_status = self._pending_map_status
        pending_route = self._pending_map_route
        pending_payload = self._pending_map_payload
        if pending_status is None or pending_route is None or pending_payload is None:
            return
        with self._lock:
            map_status = self._status['map']
            if not map_status.get('transitioning') or map_status.get('error'):
                return
            map_status.update({
                'active_id': pending_status['id'],
                'active_name': pending_status['name'],
                'active_revision': pending_status['revision'],
                'pending_id': None,
                'pending_name': None,
                'pending_revision': None,
                'active_payload_available': True,
                'source_resolution': pending_status['source_resolution'],
                'navigation_resolution': pending_status['navigation_resolution'],
                'resampled': pending_status['resampled'],
                'pending_navigation_resolution': None,
                'transitioning': False,
                'localization_ready': True,
                'patrol_route_ready': True,
                'error': None,
                'error_map_id': None,
            })
            self._active_map_payload = pending_payload
            self._mapping_base_payload = json.loads(
                json.dumps(pending_payload)
            )
            map_status['mapping_base_available'] = True
        message = String()
        message.data = json.dumps(pending_route, ensure_ascii=False)
        self._waypoint_publisher.publish(message)
        self._pending_map_status = None
        self._pending_map_route = None
        self._pending_map_payload = None
        self._expected_scene_map_id = None
        self._clear_costmaps()
        if self._pending_patrol_start is not None:
            patrol_payload = self._pending_patrol_start
            self._pending_patrol_start = None
            self._commands.put(('/api/patrol/start', patrol_payload))
        pose = self._last_map_initial_pose
        if pose is None:
            self.get_logger().info('Gazebo 场景和车辆基地已就绪，代价地图已清除')
            return
        map_x, map_y, map_yaw = pose
        self.get_logger().info(
            f'地图、Gazebo 场景和车辆基地已同步：x={map_x:.2f}, '
            f'y={map_y:.2f}, yaw={map_yaw:.2f}，代价地图已清除'
        )

    def _abort_map_transition(self, error):
        pending = self._pending_map_status
        attempted_map_id = (
            pending.get('id') if isinstance(pending, dict)
            else self._expected_scene_map_id
        )
        self._pending_map_status = None
        self._pending_map_route = None
        self._pending_map_payload = None
        self._pending_patrol_start = None
        self._expected_scene_map_id = None
        self._initial_pose_repeats = 0
        with self._lock:
            self._status['map'].update({
                'pending_id': None,
                'pending_name': None,
                'pending_revision': None,
                'pending_navigation_resolution': None,
                'transitioning': False,
                'localization_ready': False,
                'error': str(error),
                'error_map_id': attempted_map_id,
            })

    @staticmethod
    def _boolean_parameter(name, value):
        return Parameter(
            name=name,
            value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL,
                bool_value=bool(value),
            ),
        )

    def _set_perception_mode(self, mode):
        if mode not in ('lidar', 'camera', 'fusion'):
            with self._lock:
                self._status['perception']['error'] = f'不支持的感知模式：{mode}'
            return

        lidar_enabled = mode in ('lidar', 'fusion')
        camera_enabled = mode in ('camera', 'fusion')
        # Hold the base at zero while costmap layers are swapped, but keep the
        # current NavigateToPose action alive. A healthy source will release the
        # hold automatically, so switching modes does not lose task progress.
        self._perception_hold_active = True
        self._perception_unhealthy_since = None
        self._perception_healthy_since = None
        self._publish_manual(0.0, 0.0)

        self._perception_generation += 1
        generation = self._perception_generation
        with self._lock:
            self._perception_mode = mode
            self._status['perception'].update({
                'mode': mode,
                'lidar_enabled': lidar_enabled,
                'camera_enabled': camera_enabled,
                'transitioning': True,
                'gimbal_locked': mode == 'camera',
                'error': None,
            })

        if mode == 'camera':
            self._set_camera_gimbal(0.0, 0.0)

        # Live sensors belong to the local collision map only. The global map
        # remains the audited static plant map, so NavFn cannot invent a detour
        # into an unapproved area around a temporary obstacle.
        clients = (self._local_costmap_params,)
        if not all(client.service_is_ready() for client in clients):
            with self._lock:
                self._status['perception'].update({
                    'transitioning': False,
                    'safety_ok': False,
                    'error': 'Nav2 本地代价地图尚未就绪，车辆保持停车',
                })
            return

        futures = []
        for client in clients:
            request = SetParameters.Request()
            request.parameters = [
                self._boolean_parameter(
                    'lidar_obstacle_layer.enabled',
                    lidar_enabled,
                ),
                self._boolean_parameter(
                    'camera_obstacle_layer.enabled',
                    camera_enabled,
                ),
            ]
            futures.append(client.call_async(request))

        self._perception_pending = len(futures)
        for future in futures:
            future.add_done_callback(
                lambda completed, current=generation: (
                    self._on_perception_parameters_done(completed, current)
                )
            )
        self.get_logger().warning(
            f'感知模式切换为 {mode}：雷达={lidar_enabled}，相机={camera_enabled}'
        )

    def _on_perception_parameters_done(self, future, generation):
        if generation != self._perception_generation:
            return
        error = None
        try:
            response = future.result()
            failures = [
                result.reason or '参数更新失败'
                for result in response.results
                if not result.successful
            ]
            if failures:
                error = '；'.join(failures)
        except Exception as exception:  # rclpy future transports service errors.
            error = str(exception)

        self._perception_pending = max(0, self._perception_pending - 1)
        with self._lock:
            perception = self._status['perception']
            if error:
                perception['error'] = error
            if self._perception_pending == 0:
                perception['transitioning'] = False
        if self._perception_pending == 0:
            self._clear_costmaps()

    def _clear_costmaps(self):
        for client in self._costmap_clear_clients:
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())

    def _perception_watchdog(self):
        now = time.monotonic()
        snapshot = self.status_snapshot()
        perception = snapshot['perception']
        if perception.get('transitioning'):
            return
        safe = bool(perception.get('safety_ok'))
        with self._lock:
            self._status['perception']['safety_ok'] = safe

        if safe:
            self._perception_unhealthy_since = None
            if self._perception_healthy_since is None:
                self._perception_healthy_since = now
                return
            if now - self._perception_healthy_since < self._perception_recovery_stable:
                return
            if self._perception_fault_active:
                self._perception_fault_active = False
                self.get_logger().info('感知数据已稳定恢复')
            self._release_perception_hold_if_ready(snapshot)
            return

        self._perception_healthy_since = None
        if self._perception_unhealthy_since is None:
            self._perception_unhealthy_since = now
            return
        if now - self._perception_unhealthy_since < self._perception_fault_delay:
            return
        if self._perception_fault_active:
            return

        with self._lock:
            was_running = bool(self._status.get('patrol', {}).get('running'))
        self._perception_fault_active = True
        # Preserve an earlier automatic-resume request if the recovered source
        # drops again while NavigateToPose cancellation is still finishing.
        self._perception_stopped_patrol = (
            self._perception_stopped_patrol or was_running
        )
        self._perception_resume_requested = (
            self._perception_resume_requested or was_running
        )
        self._publish_manual(0.0, 0.0)
        if was_running:
            stop_client = self._patrol_clients['stop']
            if stop_client.service_is_ready():
                stop_client.call_async(Trigger.Request())
        self.get_logger().error(
            f'感知模式 {perception.get("mode")} 连续 '
            f'{self._perception_fault_delay:.1f} 秒无可用数据，已安全停车'
        )

    def _release_perception_hold_if_ready(self, snapshot):
        if not self._perception_hold_active and not self._perception_resume_requested:
            return
        patrol = snapshot.get('patrol', {})
        if self._perception_stopped_patrol:
            # Wait until NavigateToPose cancellation has actually completed.
            # Calling start while patrol_manager is still CANCELLING would be
            # rejected and leave the UI looking active while the base is idle.
            if patrol.get('running'):
                return
            start_client = self._patrol_clients['start']
            if not start_client.service_is_ready():
                return
            self._manual_override = False
            start_client.call_async(Trigger.Request())
            self.get_logger().info('感知恢复，正在自动恢复原巡检任务')
        else:
            self._manual_override = False
            self.get_logger().info('感知模式切换完成，继续当前巡检任务')
        self._perception_hold_active = False
        self._perception_stopped_patrol = False
        self._perception_resume_requested = False

    def _set_camera_gimbal(self, pan_degrees, tilt_degrees):
        with self._lock:
            gimbal_locked = bool(
                self._status['perception'].get('gimbal_locked')
            )
        if gimbal_locked:
            pan_degrees = 0.0
            tilt_degrees = 0.0
        pan = max(-self._camera_pan_limit, min(pan_degrees, self._camera_pan_limit))
        tilt = max(
            -self._camera_tilt_down_limit,
            min(tilt_degrees, self._camera_tilt_up_limit),
        )

        pan_message = Float64()
        pan_message.data = math.radians(pan)
        tilt_message = Float64()
        tilt_message.data = math.radians(-tilt)
        self._camera_pan_publisher.publish(pan_message)
        self._camera_tilt_publisher.publish(tilt_message)
        with self._lock:
            self._status['camera']['pan_target_deg'] = round(pan, 1)
            self._status['camera']['tilt_target_deg'] = round(tilt, 1)

    def _set_camera_enabled(self, enabled):
        enabled = bool(enabled)
        if enabled and self._camera_subscription is None:
            self._camera_subscription = self.create_subscription(
                Image,
                self._camera_topic,
                self._on_camera,
                qos_profile_sensor_data,
            )
            self._last_camera_enqueue = 0.0
            with self._lock:
                self._camera_frame = None
                self._camera_sequence += 1
                self._camera_frame_times.clear()
                self._status['camera'].update({
                    'enabled': True,
                    'ok': False,
                    'frames': 0,
                    'width': 0,
                    'height': 0,
                    'fps': 0.0,
                    '_last_frame': None,
                    'error': None,
                })
                self._lock.notify_all()
            self.get_logger().info(
                f'网页云台摄像头已开启：{self._camera_topic}，'
                f'{self._camera_stream_fps:.1f} FPS'
            )
        elif not enabled and self._camera_subscription is not None:
            self.destroy_subscription(self._camera_subscription)
            self._camera_subscription = None
            while True:
                try:
                    self._camera_encode_queue.get_nowait()
                except queue.Empty:
                    break
            with self._lock:
                self._camera_frame = None
                self._camera_sequence += 1
                self._camera_frame_times.clear()
                self._status['camera'].update({
                    'enabled': False,
                    'ok': False,
                    'fps': 0.0,
                    '_last_frame': None,
                    'error': None,
                })
                self._lock.notify_all()
            self.get_logger().info('网页云台摄像头已关闭')

    def _on_camera(self, message):
        now = time.monotonic()
        if now - self._last_camera_enqueue < 1.0 / self._camera_stream_fps:
            return
        self._last_camera_enqueue = now

        # Never build a backlog: if encoding is still busy, discard the old
        # queued frame and keep only the newest camera image.
        try:
            self._camera_encode_queue.put_nowait((message, now))
        except queue.Full:
            try:
                self._camera_encode_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._camera_encode_queue.put_nowait((message, now))
            except queue.Full:
                pass

    def _camera_encoder_loop(self):
        while not self._camera_encoder_stop.is_set():
            try:
                message, received_at = self._camera_encode_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            self._encode_camera_frame(message, received_at)

    def _encode_camera_frame(self, message, received_at):
        if not self.camera_enabled():
            return

        try:
            frame = self._cv_bridge.imgmsg_to_cv2(
                message,
                desired_encoding='bgr8',
            )
            if frame.ndim != 3 or frame.shape[2] != 3:
                raise ValueError('彩色图像不是三通道数据')
            if frame.shape[1] > self._camera_stream_width:
                scale = self._camera_stream_width / frame.shape[1]
                frame = cv2.resize(
                    frame,
                    (self._camera_stream_width, int(frame.shape[0] * scale)),
                    interpolation=cv2.INTER_AREA,
                )
            success, encoded = cv2.imencode(
                '.jpg',
                frame,
                [cv2.IMWRITE_JPEG_QUALITY, self._camera_jpeg_quality],
            )
            if not success:
                raise ValueError('JPEG 编码失败')
        except (CvBridgeError, cv2.error, ValueError) as error:
            with self._lock:
                self._status['camera']['error'] = str(error)
            if received_at - self._last_camera_error_log >= 2.0:
                self.get_logger().error(f'网页摄像头帧处理失败：{error}')
                self._last_camera_error_log = received_at
            return

        encoded_at = time.monotonic()
        with self._lock:
            if not self._status['camera']['enabled']:
                return
            self._camera_frame = encoded.tobytes()
            self._camera_sequence += 1
            self._camera_frame_times.append(encoded_at)
            camera = self._status['camera']
            camera['frames'] = int(camera.get('frames', 0)) + 1
            camera['width'] = int(frame.shape[1])
            camera['height'] = int(frame.shape[0])
            camera['_last_frame'] = encoded_at
            camera['error'] = None
            self._lock.notify_all()

    def _set_speed_limit(self, linear, angular):
        self._max_linear = max(0.05, min(linear, 1.5))
        self._max_angular = max(0.1, min(angular, 2.0))
        with self._lock:
            self._status['max_linear_speed'] = self._max_linear
        self._publish_speed_limit()

    def _publish_speed_limit(self):
        message = SpeedLimit()
        message.percentage = False
        message.speed_limit = min(
            self._max_linear,
            self._patrol_speed_limit
            if self._patrol_speed_limit is not None else self._max_linear,
        )
        self._speed_limit_publisher.publish(message)

    def _set_hardware_config(self, payload):
        config = {
            'height_cm': max(8.0, min(float(payload.get('height_cm', 16)), 150.0)),
            'length_cm': max(20.0, min(float(payload.get('length_cm', 52)), 250.0)),
            'width_cm': max(20.0, min(float(payload.get('width_cm', 42)), 200.0)),
            'lidar_count': max(1, min(int(payload.get('lidar_count', 1)), 4)),
            'updated_at': time.time(),
        }
        path = Path(str(self.get_parameter('hardware_config_file').value)).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(config, ensure_ascii=False, indent=2), encoding='utf-8')

        # Costmap footprint can safely follow the configured chassis immediately.
        half_length = config['length_cm'] / 200.0 + 0.03
        half_width = config['width_cm'] / 200.0 + 0.02
        footprint = (
            f'[[{half_length:.3f}, {half_width:.3f}], '
            f'[{half_length:.3f}, {-half_width:.3f}], '
            f'[{-half_length:.3f}, {-half_width:.3f}], '
            f'[{-half_length:.3f}, {half_width:.3f}]]'
        )
        for client in (self._local_costmap_params, self._global_costmap_params):
            if client.service_is_ready():
                request = SetParameters.Request()
                request.parameters = [Parameter(
                    name='footprint',
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_STRING,
                        string_value=footprint,
                    ),
                )]
                client.call_async(request)
        with self._lock:
            self._status['hardware'] = config
        self.get_logger().warning(
            '底盘安全轮廓已更新；URDF 尺寸、雷达驱动和 TF 仍需停车重启后生效'
        )

    def _publish_manual(self, linear, angular):
        command = Twist()
        command.linear.x = max(-self._max_linear, min(linear, self._max_linear))
        command.angular.z = max(-self._max_angular, min(angular, self._max_angular))
        nonzero = (
            not math.isclose(command.linear.x, 0.0)
            or not math.isclose(command.angular.z, 0.0)
        )
        if nonzero and not self._manual_override:
            client = self._patrol_clients['stop']
            if client.service_is_ready():
                client.call_async(Trigger.Request())
            frontier_client = self._frontier_clients['stop']
            if frontier_client.service_is_ready():
                frontier_client.call_async(Trigger.Request())
            self.get_logger().warning('网页人工控制已接管，自动巡检正在暂停')
        # A zero command also keeps the latch. Otherwise stale Nav2 output may
        # move the robot again before asynchronous goal cancellation finishes.
        # Patrol start or a successful perception transition releases this latch.
        self._manual_override = True
        self._publish_base_command(command)
        self._last_manual_command = time.monotonic()
        self._manual_active = nonzero

    def _manual_watchdog(self):
        timed_out = (
            time.monotonic() - self._last_manual_command > self._command_timeout
        )
        if self._manual_active and timed_out:
            self._publish_manual(0.0, 0.0)

    def destroy_node(self):
        self._set_camera_enabled(False)
        self._camera_encoder_stop.set()
        self._camera_encoder_thread.join(timeout=1.0)
        self._http.shutdown()
        self._http.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotWebBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as error:
        # During a launch-wide SIGINT, Jazzy's Python binding can wake a
        # subscription after its DDS bridge has already been torn down. This
        # exact conversion error is a shutdown race, not an application fault.
        if 'Unable to convert call argument' not in str(error):
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
