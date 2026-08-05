"""Fail-closed lidar filter shared by manual control and Nav2 output."""

import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from patrol_robot_patrol.manual_safety_geometry import evaluate_scan_safety


class ManualLidarSafety(Node):
    """Continuously publish a lidar-checked chassis command."""

    def __init__(self):
        super().__init__('manual_lidar_safety')
        defaults = {
            'input_cmd_vel_topic': '/cmd_vel_manual_raw',
            'output_cmd_vel_topic': '/cmd_vel_safety_checked',
            'scan_topic': '/scan_raw',
            'front_extent': 0.16,
            'rear_extent': 0.15,
            'half_width': 0.13,
            'lidar_offset_x': 0.0115,
            'corridor_margin': 0.02,
            'hard_clearance': 0.08,
            'slowdown_clearance': 0.35,
            'reaction_time': 0.35,
            'deceleration': 0.35,
            'rotation_clearance': 0.29,
            'command_timeout': 0.35,
            'scan_timeout': 0.50,
            'max_linear_speed': 0.12,
            'max_angular_speed': 0.35,
            'publish_rate': 20.0,
            'minimum_valid_range': 0.05,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self._settings = {
            name: float(self.get_parameter(name).value)
            for name in (
                'front_extent', 'rear_extent', 'half_width',
                'lidar_offset_x', 'corridor_margin', 'hard_clearance',
                'slowdown_clearance', 'reaction_time', 'deceleration',
                'rotation_clearance',
            )
        }
        self._minimum_valid_range = float(
            self.get_parameter('minimum_valid_range').value)
        self._command_timeout = float(
            self.get_parameter('command_timeout').value)
        self._scan_timeout = float(self.get_parameter('scan_timeout').value)
        self._max_linear = float(
            self.get_parameter('max_linear_speed').value)
        self._max_angular = float(
            self.get_parameter('max_angular_speed').value)

        input_topic = str(self.get_parameter('input_cmd_vel_topic').value)
        output_topic = str(self.get_parameter('output_cmd_vel_topic').value)
        scan_topic = str(self.get_parameter('scan_topic').value)
        self._command = None
        self._command_received_at = None
        self._scan = None
        self._scan_received_at = None
        self._last_state = None

        self._publisher = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, input_topic, self._on_command, 10)
        self.create_subscription(
            LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)
        publish_rate = max(
            float(self.get_parameter('publish_rate').value), 1.0)
        self.create_timer(1.0 / publish_rate, self._publish_safe_command)
        self.get_logger().info(
            f'雷达安全过滤已启用：{input_topic} -> {output_topic}，雷达={scan_topic}'
        )

    def _on_command(self, message):
        linear_x = float(message.linear.x)
        linear_y = float(message.linear.y)
        speed = math.hypot(linear_x, linear_y)
        if speed > self._max_linear > 0.0:
            scale = self._max_linear / speed
            linear_x *= scale
            linear_y *= scale
        angular_z = max(
            -self._max_angular,
            min(self._max_angular, float(message.angular.z)),
        )
        self._command = (linear_x, linear_y, angular_z)
        self._command_received_at = time.monotonic()

    def _on_scan(self, message):
        maximum_range = float(message.range_max)
        if not math.isfinite(maximum_range) or maximum_range <= 0.0:
            maximum_range = math.inf
        self._scan = (
            tuple(message.ranges),
            float(message.angle_min),
            float(message.angle_increment),
            max(
                float(message.range_min),
                self._minimum_valid_range,
            ),
            maximum_range,
        )
        self._scan_received_at = time.monotonic()

    @staticmethod
    def _moving(command):
        return command is not None and any(abs(value) > 1e-6 for value in command)

    def _set_state(self, state, detail=''):
        if state == self._last_state:
            return
        self._last_state = state
        message = state if not detail else f'{state}：{detail}'
        if state in ('clear', 'idle'):
            self.get_logger().info(message)
        else:
            self.get_logger().warning(message)

    def publish_stop(self, state='shutdown', detail=''):
        """Publish an explicit final stop before normal process shutdown."""
        self._publisher.publish(Twist())
        self._set_state(state, detail)

    def _publish_safe_command(self):
        now = time.monotonic()
        command = self._command
        if command is None:
            self.publish_stop('idle', '等待速度指令')
            return
        if (self._command_received_at is None
                or now - self._command_received_at > self._command_timeout):
            state = 'command_timeout' if self._moving(command) else 'idle'
            self.publish_stop(state, '速度源失联，已停车')
            return
        if not self._moving(command):
            self.publish_stop('clear')
            return
        if (self._scan is None or self._scan_received_at is None
                or now - self._scan_received_at > self._scan_timeout):
            self.publish_stop('scan_timeout', '雷达失联，已停车')
            return

        ranges, angle_min, angle_increment, range_min, range_max = self._scan
        linear_x, linear_y, angular_z = command
        decision = evaluate_scan_safety(
            ranges, angle_min, angle_increment,
            linear_x, linear_y, angular_z,
            maximum_valid_range=range_max,
            minimum_valid_range=range_min,
            **self._settings,
        )
        output = Twist()
        output.linear.x = linear_x * decision.linear_scale
        output.linear.y = linear_y * decision.linear_scale
        if decision.rotation_allowed:
            output.angular.z = angular_z
        self._publisher.publish(output)

        if decision.linear_scale <= 1e-6:
            self._set_state(
                'translation_blocked',
                f'余量={decision.nearest_translation_clearance:.3f}m，'
                f'需求={decision.required_stop_clearance:.3f}m',
            )
        elif not decision.rotation_allowed:
            self._set_state(
                'rotation_blocked',
                f'最近障碍={decision.nearest_rotation_range:.3f}m',
            )
        elif decision.linear_scale < 0.999:
            self._set_state(
                'slowing',
                f'速度比例={decision.linear_scale:.2f}',
            )
        else:
            self._set_state('clear')


def main(args=None):
    rclpy.init(args=args)
    node = ManualLidarSafety()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_stop('shutdown', '进程退出，发送零速')
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
