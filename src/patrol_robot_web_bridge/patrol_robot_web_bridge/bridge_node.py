import base64
import binascii
import json
import math
import queue
import re
import socket
import threading
import time
from collections import deque
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.msg import SpeedLimit
from nav2_msgs.srv import ClearEntireCostmap, LoadMap
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, JointState, LaserScan, PointCloud2
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


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
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_stream_fps', 12.0)
        self.declare_parameter('camera_jpeg_quality', 65)
        self.declare_parameter('camera_stream_width', 640)
        self.declare_parameter('camera_enabled_at_start', False)
        self.declare_parameter('camera_pan_limit_degrees', 90.0)
        self.declare_parameter('camera_tilt_up_limit_degrees', 25.0)
        self.declare_parameter('camera_tilt_down_limit_degrees', 35.0)
        self.declare_parameter('perception_initial_mode', 'fusion')
        self.declare_parameter('simulation_origin_x', -6.0)
        self.declare_parameter('simulation_origin_y', -4.0)
        self.declare_parameter('simulation_origin_yaw', 0.0)
        self.declare_parameter('odom_pose_is_world', True)
        self.declare_parameter('ground_truth_localization', False)

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
        self._initial_pose_repeats = 0
        self._map_storage_dir = Path(
            str(self.get_parameter('map_storage_dir').value)
        ).expanduser()
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
        initial_mode = str(self.get_parameter('perception_initial_mode').value)
        self._perception_mode = (
            initial_mode if initial_mode in ('lidar', 'camera', 'fusion') else 'fusion'
        )
        self._perception_generation = 0
        self._perception_pending = 0
        self._perception_fault_active = False
        self._perception_started_at = time.monotonic()
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
            'map': {
                'active_id': 'pipeline-demo',
                'active_name': '管廊综合测试区',
                'transitioning': False,
                'localization_ready': True,
                'error': None,
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

        self._cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
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
        self._initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10,
        )
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
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
        self.create_subscription(
            String,
            '/patrol_manager/status',
            self._on_patrol_status,
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
            for name in ('start', 'stop', 'reset')
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
        self.create_timer(0.05, self._process_commands)
        self.create_timer(0.1, self._manual_watchdog)
        self.create_timer(1.0, self._publish_speed_limit)
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
        transform.transform.rotation.w = 1.0
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
            last_frame = camera.pop('_last_frame', None)
            last_gimbal_state = camera.pop('_last_gimbal_state', None)
            last_camera_cloud = perception.pop('_last_camera_cloud', None)
            frame_times = tuple(self._camera_frame_times)
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
            and perception['last_camera_cloud_age'] < 1.5
        )
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
        status['map'] = map_status
        return status

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

    def _on_patrol_status(self, message):
        try:
            patrol = json.loads(message.data)
            with self._lock:
                self._status['patrol'] = patrol
        except json.JSONDecodeError:
            pass

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
            if path.startswith('/api/patrol/'):
                action = path.rsplit('/', 1)[-1]
                client = self._patrol_clients.get(action)
                if client and client.service_is_ready():
                    client.call_async(Trigger.Request())
            elif path == '/api/navigation/waypoints':
                message = String()
                message.data = json.dumps(payload, ensure_ascii=False)
                self._waypoint_publisher.publish(message)
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
                self._publish_manual(0.0, 0.0)
                client = self._patrol_clients['stop']
                if client.service_is_ready():
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
        else:
            resolution = max(
                requested_resolution,
                world_width / 800.0,
                world_height / 800.0,
            )
            width = max(2, int(math.ceil(world_width / resolution)))
            height = max(2, int(math.ceil(world_height / resolution)))
            occupied = [False] * (width * height)
            for row in range(height):
                for column in range(width):
                    if row in (0, height - 1) or column in (0, width - 1):
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

    def _write_scenario_map(self, payload):
        (
            origin_x, origin_y, resolution, width, height, occupied,
        ) = self._scenario_grid(payload)
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

    def _activate_map(self, payload):
        map_id = str(payload.get('id', '')).strip()
        map_name = str(payload.get('name', '')).strip()
        if not map_id or not map_name:
            with self._lock:
                self._status['map'].update({
                    'transitioning': False,
                    'error': '地图 ID 或名称不能为空',
                })
            return

        self._publish_manual(0.0, 0.0)
        stop_client = self._patrol_clients['stop']
        if stop_client.service_is_ready():
            stop_client.call_async(Trigger.Request())
        with self._lock:
            self._status['map'].update({
                'active_id': map_id,
                'active_name': map_name,
                'transitioning': True,
                'localization_ready': False,
                'error': None,
            })

        try:
            yaml_path = self._write_scenario_map(payload)
            scenario_message = String()
            scenario_message.data = json.dumps(payload, ensure_ascii=False)
            self._map_scenario_publisher.publish(scenario_message)
        except (ValueError, TypeError, OSError, binascii.Error) as error:
            with self._lock:
                self._status['map'].update({
                    'transitioning': False,
                    'error': f'地图数据无效：{error}',
                })
            return

        if not self._map_load_client.service_is_ready():
            with self._lock:
                self._status['map'].update({
                    'transitioning': False,
                    'error': '地图已保存，但 Nav2 地图服务尚未就绪',
                })
            return

        request = LoadMap.Request()
        request.map_url = str(yaml_path)
        future = self._map_load_client.call_async(request)
        future.add_done_callback(self._on_map_loaded)
        self.get_logger().warning(
            f'正在切换导航地图：{map_name} ({yaml_path})'
        )

    def _on_map_loaded(self, future):
        error = None
        try:
            response = future.result()
            if int(response.result) != 0:
                error = f'Nav2 返回错误码 {response.result}'
        except Exception as exception:
            error = str(exception)
        with self._lock:
            self._status['map'].update({
                'transitioning': not bool(error),
                'localization_ready': False,
                'error': error,
            })
        if error:
            self.get_logger().error(f'地图切换失败：{error}')
        else:
            # AMCL keeps the old map->odom transform when map_server loads a
            # different map. Re-seed it from Gazebo's ground-truth odometry so
            # the robot model, laser returns and new occupancy grid agree.
            self._initial_pose_repeats = 3
            self.get_logger().info('地图已加载，正在用仿真真值重新初始化 AMCL')

    def _publish_map_initial_pose(self):
        if self._initial_pose_repeats <= 0:
            return
        with self._lock:
            odom_pose = self._latest_odom_pose
        if odom_pose is None:
            return

        odom_x, odom_y, odom_yaw = odom_pose
        if self._odom_pose_is_world:
            # Gazebo OdometryPublisher reports the model's world pose even
            # though the message frame is named odom.
            map_x, map_y, map_yaw = odom_x, odom_y, odom_yaw
        else:
            # The lightweight simulator integrates odometry from zero at its
            # configured initial map pose.
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
        self._initial_pose_repeats -= 1

        if self._initial_pose_repeats == 0:
            self._clear_costmaps()
            with self._lock:
                self._status['map'].update({
                    'transitioning': False,
                    'localization_ready': True,
                    'error': None,
                })
            self.get_logger().info(
                f'地图定位已重置：x={map_x:.2f}, y={map_y:.2f}, '
                f'yaw={map_yaw:.2f}，代价地图已清除'
            )

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
        self._publish_manual(0.0, 0.0)
        stop_client = self._patrol_clients['stop']
        if stop_client.service_is_ready():
            stop_client.call_async(Trigger.Request())

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

        clients = (self._local_costmap_params, self._global_costmap_params)
        if not all(client.service_is_ready() for client in clients):
            with self._lock:
                self._status['perception'].update({
                    'transitioning': False,
                    'safety_ok': False,
                    'error': 'Nav2 本地或全局代价地图尚未就绪，车辆保持停车',
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
                    'camera_voxel_layer.enabled',
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
        if time.monotonic() - self._perception_started_at < 5.0:
            return
        snapshot = self.status_snapshot()
        perception = snapshot['perception']
        if perception.get('transitioning'):
            return
        safe = bool(perception.get('safety_ok'))
        with self._lock:
            self._status['perception']['safety_ok'] = safe
        if not safe and not self._perception_fault_active:
            self._perception_fault_active = True
            self._publish_manual(0.0, 0.0)
            stop_client = self._patrol_clients['stop']
            if stop_client.service_is_ready():
                stop_client.call_async(Trigger.Request())
            self.get_logger().error(
                f'感知模式 {perception.get("mode")} 无可用数据，已安全停车'
            )
        elif safe and self._perception_fault_active:
            self._perception_fault_active = False
            self.get_logger().info('感知数据已恢复，车辆保持停车等待人工恢复巡检')

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
        message.speed_limit = self._max_linear
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
            self._manual_override = True
            self.get_logger().warning('网页人工控制已接管，自动巡检正在暂停')
        elif not nonzero:
            self._manual_override = False
        self._cmd_publisher.publish(command)
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
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
