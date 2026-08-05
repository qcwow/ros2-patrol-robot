"""Capture synchronized evidence around a motion-spin guard event."""

from __future__ import annotations

from collections import deque
from copy import deepcopy
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as NavigationPath
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Imu, LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from patrol_robot_patrol.rotation_diagnostics import (
    compose_planar_transforms,
    summarize_rotation_event,
)


def _stamp_ns(message) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _yaw(rotation) -> float:
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )


class RotationDiagnosticRecorder(Node):
    """Maintain a bounded pre-event buffer and write one file per trigger."""

    def __init__(self) -> None:
        super().__init__('rotation_diagnostic_recorder')
        self.declare_parameter('pre_event_seconds', 10.0)
        self.declare_parameter('post_event_seconds', 2.0)
        self.declare_parameter('sample_period_seconds', 0.1)
        self.declare_parameter(
            'output_directory', '~/.ros/patrol_robot/rotation_diagnostics'
        )
        self.declare_parameter('imu_topic', '/imu')
        # Subscribe to the small CameraInfo messages instead of deserializing
        # every full RGB/depth image in Python.  The camera driver stamps these
        # from the same streams, which preserves timestamp diagnostics without
        # consuming a large fraction of the Orin CPU.
        self.declare_parameter(
            'rgb_info_topic', '/depth_cam/rgb0/camera_info'
        )
        self.declare_parameter(
            'depth_info_topic', '/depth_cam/depth0/camera_info'
        )

        self._pre_seconds = max(
            1.0, float(self.get_parameter('pre_event_seconds').value)
        )
        self._post_seconds = max(
            0.0, float(self.get_parameter('post_event_seconds').value)
        )
        period = max(
            0.05, float(self.get_parameter('sample_period_seconds').value)
        )
        self._output_directory = Path(
            str(self.get_parameter('output_directory').value)
        ).expanduser()
        self._samples: deque[dict] = deque(
            maxlen=max(2, math.ceil(self._pre_seconds / period) + 2)
        )
        self._latest: dict[str, object] = {
            'commands': {},
            'sensors': {},
            'statuses': {},
            'map_to_odom': None,
        }
        self._capture: list[dict] | None = None
        self._capture_deadline: float | None = None
        self._trigger: dict | None = None
        self._trigger_latched: dict[str, bool] = {}

        for topic, key in (
            ('/cmd_vel_nav_raw', 'cmd_vel_nav_raw'),
            ('/cmd_vel_nav', 'cmd_vel_nav'),
            ('/cmd_vel_base_raw', 'cmd_vel_base_raw'),
            ('/cmd_vel_safety_checked', 'cmd_vel_safety_checked'),
            ('/controller/cmd_vel', 'controller_cmd_vel'),
        ):
            self.create_subscription(
                Twist,
                topic,
                lambda message, name=key: self._on_twist(name, message),
                10,
            )

        for topic, key in (
            ('/frontier_explorer/status', 'frontier_explorer'),
            ('/patrol_manager/status', 'patrol_manager'),
            ('/navigation_health/status', 'navigation_health'),
        ):
            self.create_subscription(
                String,
                topic,
                lambda message, name=key: self._on_status(name, message),
                10,
            )

        self.create_subscription(
            Imu,
            str(self.get_parameter('imu_topic').value),
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            '/odom_raw',
            lambda message: self._on_odom('odom_raw', message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            '/odom',
            lambda message: self._on_odom('odom_filtered', message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan, '/scan_raw', self._on_scan, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('rgb_info_topic').value),
            lambda message: self._on_camera_info('rgb', message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('depth_info_topic').value),
            lambda message: self._on_camera_info('depth', message),
            qos_profile_sensor_data,
        )
        self.create_subscription(NavigationPath, '/plan', self._on_plan, 10)
        self.create_subscription(TFMessage, '/tf', self._on_tf, 100)
        self.create_timer(period, self._sample)

    def _on_twist(self, name: str, message: Twist) -> None:
        commands = self._latest['commands']
        commands[f'{name}_linear_x'] = float(message.linear.x)
        commands[f'{name}_linear_y'] = float(message.linear.y)
        commands[f'{name}_angular_z'] = float(message.angular.z)
        commands[f'{name}_received_monotonic'] = time.monotonic()

    def _on_status(self, name: str, message: String) -> None:
        try:
            status = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            status = {'invalid_json': message.data[:500]}
        if not isinstance(status, dict):
            status = {'invalid_json_type': type(status).__name__}
        self._latest['statuses'][name] = status

        triggered = bool(status.get('motion_spin_guard_triggered'))
        if (
            triggered
            and not self._trigger_latched.get(name, False)
            and self._capture is None
        ):
            self._start_capture(name, status)
        self._trigger_latched[name] = triggered

    def _on_imu(self, message: Imu) -> None:
        self._latest['imu'] = {
            'stamp_ns': _stamp_ns(message),
            'received_system_ns': time.time_ns(),
            'angular_velocity_z': float(message.angular_velocity.z),
            'orientation_yaw': _yaw(message.orientation),
        }

    def _on_odom(self, name: str, message: Odometry) -> None:
        self._latest['sensors'][name] = {
            'stamp_ns': _stamp_ns(message),
            'received_system_ns': time.time_ns(),
            'x': float(message.pose.pose.position.x),
            'y': float(message.pose.pose.position.y),
            'yaw': _yaw(message.pose.pose.orientation),
            'linear_x': float(message.twist.twist.linear.x),
            'linear_y': float(message.twist.twist.linear.y),
            'angular_z': float(message.twist.twist.angular.z),
        }

    def _on_scan(self, message: LaserScan) -> None:
        self._latest['sensors']['scan'] = {
            'stamp_ns': _stamp_ns(message),
            'received_system_ns': time.time_ns(),
            'range_count': len(message.ranges),
        }

    def _on_camera_info(self, name: str, message: CameraInfo) -> None:
        self._latest['sensors'][name] = {
            'stamp_ns': _stamp_ns(message),
            'received_system_ns': time.time_ns(),
            'width': int(message.width),
            'height': int(message.height),
        }

    def _on_plan(self, message: NavigationPath) -> None:
        plan = {
            'stamp_ns': _stamp_ns(message),
            'pose_count': len(message.poses),
        }
        if message.poses:
            plan['start'] = {
                'x': float(message.poses[0].pose.position.x),
                'y': float(message.poses[0].pose.position.y),
            }
            plan['goal'] = {
                'x': float(message.poses[-1].pose.position.x),
                'y': float(message.poses[-1].pose.position.y),
            }
        self._latest['plan'] = plan

    def _on_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if (
                transform.header.frame_id.lstrip('/') == 'map'
                and transform.child_frame_id.lstrip('/') == 'odom'
            ):
                value = transform.transform
                self._latest['map_to_odom'] = {
                    'stamp_ns': _stamp_ns(transform),
                    'x': float(value.translation.x),
                    'y': float(value.translation.y),
                    'yaw': _yaw(value.rotation),
                }

    def _sample(self) -> None:
        now = time.monotonic()
        sample = deepcopy(self._latest)
        sample['monotonic_seconds'] = now
        sample['system_time_ns'] = time.time_ns()
        odom_to_base = sample['sensors'].get('odom_filtered')
        sample['odom_to_base'] = odom_to_base
        map_to_odom = sample.get('map_to_odom')
        if map_to_odom is not None and odom_to_base is not None:
            sample['map_to_base'] = compose_planar_transforms(
                map_to_odom, odom_to_base
            )
        else:
            sample['map_to_base'] = None
        self._samples.append(sample)

        if self._capture is not None:
            self._capture.append(sample)
            if now >= float(self._capture_deadline):
                self._finish_capture()

    def _start_capture(self, source: str, status: dict) -> None:
        now = time.monotonic()
        self._capture = list(self._samples)
        self._capture_deadline = now + self._post_seconds
        self._trigger = {
            'source': source,
            'monotonic_seconds': now,
            'system_time_ns': time.time_ns(),
            'status': deepcopy(status),
        }
        self.get_logger().error('转圈保护触发，正在保存多源诊断数据')
        if self._post_seconds == 0.0:
            self._finish_capture()

    def _finish_capture(self) -> None:
        if not self._capture or self._trigger is None:
            self._capture = None
            self._capture_deadline = None
            self._trigger = None
            return
        origin = float(self._capture[0]['monotonic_seconds'])
        for sample in self._capture:
            sample['elapsed_seconds'] = round(
                float(sample['monotonic_seconds']) - origin, 6
            )
        document = {
            'schema_version': 1,
            'created_at': datetime.now(timezone.utc).isoformat(),
            'pre_event_seconds': self._pre_seconds,
            'post_event_seconds': self._post_seconds,
            'trigger': self._trigger,
            'summary': summarize_rotation_event(self._capture),
            'samples': self._capture,
        }
        timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        self._output_directory.mkdir(parents=True, exist_ok=True)
        output = self._output_directory / f'rotation-event-{timestamp}.json'
        output.write_text(
            json.dumps(document, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8',
        )
        self.get_logger().error(
            f'转圈诊断已写入 {output}；判定：'
            f"{document['summary']['classification']}"
        )
        self._capture = None
        self._capture_deadline = None
        self._trigger = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RotationDiagnosticRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
