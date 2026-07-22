"""Record auditable metrics for a navigation regression scenario."""

from __future__ import annotations

import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from patrol_robot_patrol.navigation_metrics import NavigationRunMetrics


class NavigationRegressionRecorder(Node):
    def __init__(self) -> None:
        super().__init__('navigation_regression_recorder')
        self.declare_parameter('scenario', 'manual_run')
        self.declare_parameter(
            'output_directory',
            '~/.ros/patrol_robot/navigation_regression',
        )
        self._scenario = str(self.get_parameter('scenario').value)
        self._output_directory = Path(
            str(self.get_parameter('output_directory').value)
        ).expanduser()
        self._metrics: NavigationRunMetrics | None = None
        self._last_status: dict = {}
        self._actual_pose: tuple[float, float, float] | None = None
        self._ground_truth_pose: tuple[float, float, float] | None = None
        self._written = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(
            String,
            '/patrol_manager/status',
            self._on_status,
            10,
        )
        self.create_subscription(
            Odometry,
            '/odom',
            self._on_odom,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            '/ground_truth/odom',
            self._on_ground_truth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            '/scan',
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_timer(0.25, self._update_map_pose)

    def _ensure_metrics(self) -> NavigationRunMetrics:
        if self._metrics is None:
            self._metrics = NavigationRunMetrics(
                scenario=self._scenario,
                started_at=time.monotonic(),
            )
        return self._metrics

    def _on_odom(self, message: Odometry) -> None:
        if self._metrics is None:
            return
        self._metrics.update_pose(
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )

    def _on_scan(self, message: LaserScan) -> None:
        if self._metrics is not None:
            self._metrics.update_scan(message.ranges)

    def _on_ground_truth(self, message: Odometry) -> None:
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        self._ground_truth_pose = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            yaw,
        )

    def _on_status(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self._last_status = status
        state = str(status.get('state', 'UNKNOWN'))
        if bool(status.get('running')) or state in {'BLOCKED', 'ESTOP', 'COMPLETE'}:
            self._ensure_metrics().update_status(status)
        if state in {'BLOCKED', 'ESTOP', 'COMPLETE'} and not self._written:
            self._write_result()

    def _update_map_pose(self) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(
                'map', 'base_footprint', Time()
            )
        except TransformException:
            return
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        self._actual_pose = (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
            yaw,
        )

    def _write_result(self) -> None:
        if self._metrics is None:
            return
        result = self._metrics.summary(
            time.monotonic(), self._last_status, self._actual_pose
        )
        if self._actual_pose is not None and self._ground_truth_pose is not None:
            result['localization_position_error_meters'] = round(
                math.hypot(
                    self._actual_pose[0] - self._ground_truth_pose[0],
                    self._actual_pose[1] - self._ground_truth_pose[1],
                ),
                4,
            )
            yaw_delta = self._actual_pose[2] - self._ground_truth_pose[2]
            result['localization_yaw_error_radians'] = round(
                abs(math.atan2(math.sin(yaw_delta), math.cos(yaw_delta))),
                4,
            )
        timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        self._output_directory.mkdir(parents=True, exist_ok=True)
        output = self._output_directory / f'{self._scenario}-{timestamp}.json'
        output.write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8',
        )
        self._written = True
        self.get_logger().info(f'导航回归结果已写入 {output}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationRegressionRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        # A bridge may be torn down while its final sample is being converted
        # during launch shutdown. Surface conversion failures during normal
        # operation, but do not turn a clean, signal-driven stop into a failed
        # regression process.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
