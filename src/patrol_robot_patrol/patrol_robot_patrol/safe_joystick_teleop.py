"""ROSOrin mecanum joystick adapter for the raw safety command channel."""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class SafeJoystickTeleop(Node):
    """Convert every Joy frame to Twist; downstream nodes enforce safety."""

    def __init__(self):
        super().__init__('safe_joystick_teleop')
        defaults = {
            'joy_topic': '/ros_robot_controller/joy',
            'output_cmd_vel_topic': '/cmd_vel_manual_raw',
            'max_linear_speed': 0.10,
            'max_angular_speed': 0.30,
            'deadzone': 0.10,
            'lateral_axis': 0,
            'forward_axis': 1,
            'yaw_axis': 2,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        joy_topic = str(self.get_parameter('joy_topic').value)
        output_topic = str(
            self.get_parameter('output_cmd_vel_topic').value)
        self._max_linear = float(
            self.get_parameter('max_linear_speed').value)
        self._max_angular = float(
            self.get_parameter('max_angular_speed').value)
        self._deadzone = float(self.get_parameter('deadzone').value)
        self._lateral_axis = int(self.get_parameter('lateral_axis').value)
        self._forward_axis = int(self.get_parameter('forward_axis').value)
        self._yaw_axis = int(self.get_parameter('yaw_axis').value)
        self._invalid_axes_logged = False

        self._publisher = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Joy, joy_topic, self._on_joy, 10)
        self.get_logger().info(
            f'安全手柄适配已启用：{joy_topic} -> {output_topic}'
        )

    def _axis(self, axes, index):
        if index < 0 or index >= len(axes):
            if not self._invalid_axes_logged:
                self.get_logger().error('Joy axes 数量不足，保持停车')
                self._invalid_axes_logged = True
            return 0.0
        value = float(axes[index])
        return 0.0 if abs(value) < self._deadzone else value

    def _on_joy(self, message):
        command = Twist()
        command.linear.x = (
            self._axis(message.axes, self._forward_axis) * self._max_linear)
        command.linear.y = (
            self._axis(message.axes, self._lateral_axis) * self._max_linear)
        command.angular.z = (
            self._axis(message.axes, self._yaw_axis) * self._max_angular)
        self._publisher.publish(command)

    def publish_stop(self):
        self._publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = SafeJoystickTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
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
