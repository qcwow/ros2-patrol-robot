"""Final command watchdog for factory drivers which retain the last command."""

import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class BaseCommandWatchdog(Node):
    """Republish only fresh, bounded commands and otherwise publish zero."""

    def __init__(self):
        super().__init__('base_command_watchdog')
        defaults = {
            'input_cmd_vel_topic': '/cmd_vel_safety_checked',
            'output_cmd_vel_topic': '/controller/cmd_vel',
            'command_timeout': 0.25,
            'publish_rate': 40.0,
            'max_linear_speed': 0.12,
            'max_angular_speed': 0.35,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        input_topic = str(self.get_parameter('input_cmd_vel_topic').value)
        output_topic = str(self.get_parameter('output_cmd_vel_topic').value)
        self._timeout = float(self.get_parameter('command_timeout').value)
        self._max_linear = float(
            self.get_parameter('max_linear_speed').value)
        self._max_angular = float(
            self.get_parameter('max_angular_speed').value)
        self._command = Twist()
        self._received_at = None
        self._timed_out = False

        self._publisher = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, input_topic, self._on_command, 10)
        publish_rate = max(
            float(self.get_parameter('publish_rate').value), 1.0)
        self.create_timer(1.0 / publish_rate, self._publish)
        self.get_logger().info(
            f'底盘速度看门狗已启用：{input_topic} -> {output_topic}'
        )

    def _on_command(self, message):
        command = Twist()
        linear_x = float(message.linear.x)
        linear_y = float(message.linear.y)
        speed = math.hypot(linear_x, linear_y)
        if speed > self._max_linear > 0.0:
            scale = self._max_linear / speed
            linear_x *= scale
            linear_y *= scale
        command.linear.x = linear_x
        command.linear.y = linear_y
        command.angular.z = max(
            -self._max_angular,
            min(self._max_angular, float(message.angular.z)),
        )
        self._command = command
        self._received_at = time.monotonic()
        if self._timed_out:
            self.get_logger().info('底盘速度源已恢复')
        self._timed_out = False

    def publish_stop(self):
        """Publish an explicit zero command."""
        self._publisher.publish(Twist())

    def _publish(self):
        stale = (
            self._received_at is None
            or time.monotonic() - self._received_at > self._timeout
        )
        if stale:
            self.publish_stop()
            if self._received_at is not None and not self._timed_out:
                self.get_logger().warning('底盘速度源超时，持续发送零速')
                self._timed_out = True
            return
        self._publisher.publish(self._command)


def main(args=None):
    rclpy.init(args=args)
    node = BaseCommandWatchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
