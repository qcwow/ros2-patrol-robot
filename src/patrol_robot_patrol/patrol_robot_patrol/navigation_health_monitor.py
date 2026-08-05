"""Publish a single readiness gate for autonomous patrol navigation."""

from __future__ import annotations

import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from patrol_robot_patrol.footprint_geometry import (
    quaternion_to_yaw,
    rectangle_overlaps_lethal_cell,
)
from patrol_robot_patrol.navigation_motion_guard import (
    PoseStabilityGate,
    PoseStabilityStatus,
)


class NavigationHealthMonitor(Node):
    """Require live sensors, a complete TF chain and active Nav2 servers."""

    def __init__(self) -> None:
        super().__init__('navigation_health')
        self.declare_parameter('scan_timeout_seconds', 1.0)
        self.declare_parameter('odom_timeout_seconds', 1.0)
        self.declare_parameter('amcl_timeout_seconds', 2.0)
        self.declare_parameter('camera_timeout_seconds', 2.0)
        # The global OccupancyGrid is a low-rate planning snapshot, not the
        # real-time collision sensor. Allow two missed 0.5 Hz publications so
        # a loaded Gazebo host does not repeatedly stop an otherwise safe run.
        self.declare_parameter('costmap_timeout_seconds', 8.0)
        self.declare_parameter('footprint_length', 0.52)
        self.declare_parameter('footprint_width', 0.42)
        self.declare_parameter('footprint_safety_margin', 0.01)
        self.declare_parameter('lethal_cost_threshold', 100)
        self.declare_parameter('footprint_collision_confirm_seconds', 1.0)
        self.declare_parameter('require_base_driver_ready', False)
        self.declare_parameter('require_estop_released', False)
        self.declare_parameter('require_amcl_covariance', False)
        self.declare_parameter('max_position_variance', 0.25)
        self.declare_parameter('max_yaw_variance', 0.25)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('require_global_pose_stability', False)
        self.declare_parameter('global_pose_stable_seconds', 5.0)
        self.declare_parameter('global_pose_max_translation_delta', 0.08)
        self.declare_parameter('global_pose_max_yaw_delta_degrees', 5.0)
        self.declare_parameter('lifecycle_nodes', [
            'map_server',
            'amcl',
            'controller_server',
            'smoother_server',
            'planner_server',
            'behavior_server',
            'velocity_smoother',
            'bt_navigator',
            'waypoint_follower',
        ])

        self._scan_timeout = max(
            0.1, float(self.get_parameter('scan_timeout_seconds').value)
        )
        self._odom_timeout = max(
            0.1, float(self.get_parameter('odom_timeout_seconds').value)
        )
        self._amcl_timeout = max(
            0.1, float(self.get_parameter('amcl_timeout_seconds').value)
        )
        self._camera_timeout = max(
            0.1, float(self.get_parameter('camera_timeout_seconds').value)
        )
        self._costmap_timeout = max(
            0.5, float(self.get_parameter('costmap_timeout_seconds').value)
        )
        self._footprint_length = max(
            0.05, float(self.get_parameter('footprint_length').value)
        )
        self._footprint_width = max(
            0.05, float(self.get_parameter('footprint_width').value)
        )
        self._footprint_safety_margin = max(
            0.0, float(self.get_parameter('footprint_safety_margin').value)
        )
        self._lethal_cost_threshold = max(
            100, int(self.get_parameter('lethal_cost_threshold').value)
        )
        self._footprint_collision_confirm_seconds = max(
            0.0,
            float(
                self.get_parameter('footprint_collision_confirm_seconds').value
            ),
        )
        self._require_base_driver = bool(
            self.get_parameter('require_base_driver_ready').value
        )
        self._require_estop = bool(
            self.get_parameter('require_estop_released').value
        )
        self._require_amcl = bool(
            self.get_parameter('require_amcl_covariance').value
        )
        self._max_position_variance = max(
            0.001, float(self.get_parameter('max_position_variance').value)
        )
        self._max_yaw_variance = max(
            0.001, float(self.get_parameter('max_yaw_variance').value)
        )
        self._global_frame = str(self.get_parameter('global_frame').value)
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._require_global_pose_stability = bool(
            self.get_parameter('require_global_pose_stability').value
        )
        self._global_pose_stable_seconds = max(
            0.0,
            float(self.get_parameter('global_pose_stable_seconds').value),
        )
        self._global_pose_stability = PoseStabilityGate(
            self._global_pose_stable_seconds,
            float(
                self.get_parameter(
                    'global_pose_max_translation_delta'
                ).value
            ),
            math.radians(float(
                self.get_parameter(
                    'global_pose_max_yaw_delta_degrees'
                ).value
            )),
        )
        self._lifecycle_nodes = tuple(
            str(name) for name in self.get_parameter('lifecycle_nodes').value
        )

        sensor_qos = QoSProfile(depth=5)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(LaserScan, '/scan', self._on_scan, sensor_qos)
        self.create_subscription(Odometry, '/odom', self._on_odom, sensor_qos)
        self.create_subscription(
            PointCloud2,
            '/camera/points/filtered',
            self._on_camera_cloud,
            sensor_qos,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._on_amcl_pose,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self._on_global_costmap,
            10,
        )
        self.create_subscription(
            Bool,
            '/base_driver/ready',
            self._on_base_driver_ready,
            10,
        )
        self.create_subscription(
            Bool,
            '/estop/released',
            self._on_estop_released,
            10,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._last_scan = None
        self._last_odom = None
        self._last_amcl = None
        self._last_camera = None
        self._last_costmap = None
        self._global_costmap: OccupancyGrid | None = None
        self._base_driver_ready = False
        self._estop_released = False
        self._position_variance = math.inf
        self._yaw_variance = math.inf
        self._footprint_collision_started: float | None = None
        self._last_ready_state: bool | None = None
        self._lifecycle_states = {
            name: None for name in self._lifecycle_nodes
        }
        self._lifecycle_pending: set[str] = set()
        self._lifecycle_clients = {
            name: self.create_client(GetState, f'/{name}/get_state')
            for name in self._lifecycle_nodes
        }

        ready_qos = QoSProfile(depth=1)
        ready_qos.reliability = ReliabilityPolicy.RELIABLE
        ready_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._ready_publisher = self.create_publisher(Bool, '~/ready', ready_qos)
        self._status_publisher = self.create_publisher(String, '~/status', ready_qos)
        self.create_timer(0.5, self._evaluate)
        self.create_timer(1.0, self._poll_lifecycle_states)

    def _on_scan(self, _message: LaserScan) -> None:
        self._last_scan = time.monotonic()

    def _on_odom(self, _message: Odometry) -> None:
        self._last_odom = time.monotonic()

    def _on_amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        self._last_amcl = time.monotonic()
        covariance = message.pose.covariance
        # Humble may expose fixed-size ROS arrays through NumPy scalar types.
        # Convert at the message boundary so JSON status always contains
        # ordinary Python numbers and booleans.
        self._position_variance = max(
            float(covariance[0]),
            float(covariance[7]),
        )
        self._yaw_variance = float(covariance[35])

    def _on_camera_cloud(self, _message: PointCloud2) -> None:
        self._last_camera = time.monotonic()

    def _on_global_costmap(self, message: OccupancyGrid) -> None:
        self._global_costmap = message
        self._last_costmap = time.monotonic()

    def _on_base_driver_ready(self, message: Bool) -> None:
        self._base_driver_ready = bool(message.data)

    def _on_estop_released(self, message: Bool) -> None:
        self._estop_released = bool(message.data)

    def _poll_lifecycle_states(self) -> None:
        for name, client in self._lifecycle_clients.items():
            if name in self._lifecycle_pending or not client.service_is_ready():
                continue
            self._lifecycle_pending.add(name)
            future = client.call_async(GetState.Request())
            future.add_done_callback(
                lambda result, node_name=name: self._on_lifecycle_state(
                    node_name, result
                )
            )

    def _on_lifecycle_state(self, name: str, future) -> None:
        self._lifecycle_pending.discard(name)
        try:
            self._lifecycle_states[name] = int(future.result().current_state.id)
        except Exception as error:  # Service transport errors are health data.
            # Keep the last confirmed lifecycle state across a transient RPC
            # timeout. On a software-rendered VM, nine concurrent lifecycle
            # services can occasionally miss a response even though the Nav2
            # lifecycle manager still has a live bond to the active node.
            self.get_logger().debug(f'读取 {name} 生命周期失败: {error}')

    @staticmethod
    def _fresh(last_update: float | None, timeout: float, now: float) -> bool:
        return last_update is not None and now - last_update <= timeout

    def _has_transform(self, target: str, source: str) -> bool:
        try:
            self._tf_buffer.lookup_transform(target, source, Time())
            return True
        except TransformException:
            return False

    def _global_pose_status(
        self, now: float
    ) -> tuple[bool, PoseStabilityStatus]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._odom_frame,
                Time(),
            )
        except TransformException:
            self._global_pose_stability.reset()
            return False, PoseStabilityStatus(False, 0.0, 0.0, 0.0)
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        status = self._global_pose_stability.observe(
            now,
            float(translation.x),
            float(translation.y),
            quaternion_to_yaw(
                float(rotation.x),
                float(rotation.y),
                float(rotation.z),
                float(rotation.w),
            ),
        )
        return True, status

    def _footprint_is_clear(self) -> bool:
        costmap = self._global_costmap
        if costmap is None or costmap.info.resolution <= 0.0:
            return False
        try:
            transform = self._tf_buffer.lookup_transform(
                costmap.header.frame_id or self._global_frame,
                self._base_frame,
                Time(),
            )
        except TransformException:
            return False
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        try:
            overlaps = rectangle_overlaps_lethal_cell(
                costmap.data,
                grid_width=int(costmap.info.width),
                grid_height=int(costmap.info.height),
                resolution=float(costmap.info.resolution),
                origin_x=float(costmap.info.origin.position.x),
                origin_y=float(costmap.info.origin.position.y),
                robot_x=float(translation.x),
                robot_y=float(translation.y),
                robot_yaw=quaternion_to_yaw(
                    float(rotation.x),
                    float(rotation.y),
                    float(rotation.z),
                    float(rotation.w),
                ),
                footprint_length=self._footprint_length,
                footprint_width=self._footprint_width,
                safety_margin=self._footprint_safety_margin,
                lethal_cost_threshold=self._lethal_cost_threshold,
            )
        except ValueError:
            return False
        return not overlaps

    def _confirm_footprint_clear(self, raw_clear: bool, now: float) -> bool:
        """Ignore a one-frame map artifact while Nav2 keeps motion safe."""
        if raw_clear:
            self._footprint_collision_started = None
            return True
        if self._footprint_collision_started is None:
            self._footprint_collision_started = now
        return (
            now - self._footprint_collision_started
            < self._footprint_collision_confirm_seconds
        )

    def _evaluate(self) -> None:
        now = time.monotonic()
        scan_ok = self._fresh(self._last_scan, self._scan_timeout, now)
        odom_ok = self._fresh(self._last_odom, self._odom_timeout, now)
        camera_ok = self._fresh(self._last_camera, self._camera_timeout, now)
        amcl_fresh = self._fresh(self._last_amcl, self._amcl_timeout, now)
        costmap_fresh = self._fresh(
            self._last_costmap, self._costmap_timeout, now
        )
        map_tf_ok, global_pose_status = self._global_pose_status(now)
        global_pose_stable = bool(
            not self._require_global_pose_stability
            or (map_tf_ok and global_pose_status.ready)
        )
        odom_tf_ok = self._has_transform(self._odom_frame, self._base_frame)
        inactive_nodes = [
            name for name, state in self._lifecycle_states.items()
            if state != State.PRIMARY_STATE_ACTIVE
        ]
        nav2_active = not inactive_nodes

        amcl_received = self._last_amcl is not None
        localization_ok = bool(
            not self._require_amcl
            or (
                amcl_received
                and amcl_fresh
                and self._position_variance <= self._max_position_variance
                and self._yaw_variance <= self._max_yaw_variance
            )
        )
        raw_footprint_clear = bool(
            costmap_fresh and self._footprint_is_clear()
        )
        footprint_clear = bool(
            costmap_fresh
            and self._confirm_footprint_clear(raw_footprint_clear, now)
        )
        if not costmap_fresh:
            self._footprint_collision_started = None
        base_driver_ok = bool(
            not self._require_base_driver or self._base_driver_ready
        )
        estop_ok = bool(not self._require_estop or self._estop_released)

        reasons = []
        if not nav2_active:
            reasons.append('Nav2 未激活: ' + ', '.join(inactive_nodes))
        if not scan_ok:
            reasons.append('/scan 无持续数据')
        if not odom_ok:
            reasons.append('/odom 无持续数据')
        if not map_tf_ok:
            reasons.append(f'{self._global_frame}→{self._odom_frame} TF 缺失')
        elif not global_pose_stable:
            reasons.append(
                f'{self._global_frame}→{self._odom_frame} 位姿尚未持续稳定 '
                f'{self._global_pose_stable_seconds:.1f} 秒'
                f'（已稳定 {global_pose_status.stable_for:.1f} 秒，'
                f'平移变化 {global_pose_status.translation_delta:.3f} m，'
                f'角度变化 '
                f'{math.degrees(global_pose_status.yaw_delta):.1f}°）'
            )
        if not odom_tf_ok:
            reasons.append(f'{self._odom_frame}→{self._base_frame} TF 缺失')
        if not localization_ok:
            reasons.append('AMCL 未收敛或定位数据过期')
        if not costmap_fresh:
            reasons.append('全局代价地图未就绪或数据过期')
        elif not footprint_clear:
            reasons.append('机器人实体轮廓持续与致命障碍重叠')
        if not base_driver_ok:
            reasons.append('底盘驱动或速度看门狗未就绪')
        if not estop_ok:
            reasons.append('硬件急停未释放')

        ready = not reasons
        if self._last_ready_state is None or ready != self._last_ready_state:
            if ready:
                self.get_logger().info(
                    '导航健康门已恢复：全部检查通过；'
                    f'{self._global_frame}→{self._odom_frame} 已稳定 '
                    f'{global_pose_status.stable_for:.1f} 秒'
                )
            else:
                self.get_logger().warning(
                    '导航健康门已关闭：' + '；'.join(reasons)
                )
            self._last_ready_state = ready
        status_message = String()
        status_message.data = json.dumps({
            'ready': ready,
            'reason': None if ready else '；'.join(reasons),
            'checks': {
                'nav2_active': nav2_active,
                'scan_ok': scan_ok,
                'odom_ok': odom_ok,
                'camera_ok': camera_ok,
                'map_to_odom_tf_ok': map_tf_ok,
                'global_pose_stable': global_pose_stable,
                'odom_to_base_tf_ok': odom_tf_ok,
                'localization_ok': localization_ok,
                'amcl_pose_received': amcl_received,
                'amcl_pose_fresh': amcl_fresh,
                'costmap_fresh': costmap_fresh,
                'footprint_raw_clear': raw_footprint_clear,
                'footprint_clear': footprint_clear,
                'base_driver_ok': base_driver_ok,
                'estop_released': estop_ok,
            },
            'inactive_nodes': inactive_nodes,
            'position_variance': (
                None if not math.isfinite(self._position_variance)
                else self._position_variance
            ),
            'yaw_variance': (
                None if not math.isfinite(self._yaw_variance)
                else self._yaw_variance
            ),
            'global_pose_stable_seconds': round(
                global_pose_status.stable_for, 2
            ),
            'global_pose_translation_delta': round(
                global_pose_status.translation_delta, 4
            ),
            'global_pose_yaw_delta_degrees': round(
                math.degrees(global_pose_status.yaw_delta), 3
            ),
        }, ensure_ascii=False)
        self._status_publisher.publish(status_message)

        # Publish details first so consumers can classify a false readiness
        # edge before deciding whether automatic recovery is permitted.
        ready_message = Bool()
        ready_message.data = ready
        self._ready_publisher.publish(ready_message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationHealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
