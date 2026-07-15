import json
import math
import queue
import threading
import time
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.msg import SpeedLimit
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


class RobotWebBridge(Node):
    """Small dependency-free HTTP gateway with ROS-side safety enforcement."""

    def __init__(self):
        super().__init__('patrol_robot_web_bridge')
        self.declare_parameter('http_host', '0.0.0.0')
        self.declare_parameter('http_port', 8765)
        self.declare_parameter('max_linear_speed', 0.6)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('manual_command_timeout', 0.5)
        self.declare_parameter('hardware_config_file', '~/.ros/patrol_robot/hardware.json')

        self._max_linear = float(self.get_parameter('max_linear_speed').value)
        self._max_angular = float(self.get_parameter('max_angular_speed').value)
        self._command_timeout = float(self.get_parameter('manual_command_timeout').value)
        self._commands = queue.Queue()
        self._lock = threading.Lock()
        self._last_manual_command = 0.0
        self._manual_active = False
        self._manual_override = False
        self._status = {
            'connected': True, 'patrol': {'state': 'UNKNOWN', 'running': False},
            'speed': 0.0, 'angular_speed': 0.0, 'x': 0.0, 'y': 0.0,
            'battery': None, 'lidar_ok': False, 'last_scan_age': None,
            'max_linear_speed': self._max_linear,
        }

        self._cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self._waypoint_publisher = self.create_publisher(String, '/patrol_manager/set_waypoints', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(String, '/patrol_manager/status', self._on_patrol_status, 10)
        self._patrol_clients = {
            name: self.create_client(Trigger, f'/patrol_manager/{name}')
            for name in ('start', 'stop', 'reset')
        }
        self._speed_limit_publisher = self.create_publisher(SpeedLimit, '/speed_limit', 10)
        self._local_costmap_params = self.create_client(SetParameters, '/local_costmap/local_costmap/set_parameters')
        self._global_costmap_params = self.create_client(SetParameters, '/global_costmap/global_costmap/set_parameters')
        self.create_timer(0.05, self._process_commands)
        self.create_timer(0.1, self._manual_watchdog)
        self.create_timer(1.0, self._publish_speed_limit)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_timer(0.2, self._update_map_pose)

        host = str(self.get_parameter('http_host').value)
        port = int(self.get_parameter('http_port').value)
        handler = self._make_handler()
        self._http = ThreadingHTTPServer((host, port), handler)
        self._http_thread = threading.Thread(target=self._http.serve_forever, daemon=True)
        self._http_thread.start()
        self.get_logger().info(f'车辆 Web 网关已监听 http://{host}:{port}')

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

            def do_OPTIONS(self):
                self._send(204, {})

            def do_GET(self):
                if urlparse(self.path).path == '/api/status':
                    self._send(200, bridge.status_snapshot())
                elif urlparse(self.path).path == '/api/health':
                    self._send(200, {'ok': True, 'service': 'patrol_robot_web_bridge'})
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
        status['last_scan_age'] = None if last_scan is None else round(time.monotonic() - last_scan, 2)
        status['lidar_ok'] = status['last_scan_age'] is not None and status['last_scan_age'] < 1.0
        return status

    def _on_odom(self, message):
        with self._lock:
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

    def _on_patrol_status(self, message):
        try:
            patrol = json.loads(message.data)
            with self._lock:
                self._status['patrol'] = patrol
        except json.JSONDecodeError:
            pass

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
            elif path == '/api/config/speed':
                self._set_speed_limit(float(payload.get('linear', self._max_linear)), float(payload.get('angular', self._max_angular)))
            elif path == '/api/control/manual':
                self._publish_manual(float(payload.get('linear', 0.0)), float(payload.get('angular', 0.0)))
            elif path == '/api/control/emergency-stop':
                self._publish_manual(0.0, 0.0)
                client = self._patrol_clients['stop']
                if client.service_is_ready():
                    client.call_async(Trigger.Request())
            elif path == '/api/config/hardware':
                self._set_hardware_config(payload)

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
                    value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=footprint),
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
        nonzero = not math.isclose(command.linear.x, 0.0) or not math.isclose(command.angular.z, 0.0)
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
        if self._manual_active and time.monotonic() - self._last_manual_command > self._command_timeout:
            self._publish_manual(0.0, 0.0)

    def destroy_node(self):
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
