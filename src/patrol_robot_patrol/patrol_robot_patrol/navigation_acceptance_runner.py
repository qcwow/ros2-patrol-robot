"""Safely arm, monitor, stop, and score one real-car patrol route."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
import yaml

from patrol_robot_patrol.navigation_acceptance import (
    NavigationAcceptanceLimits,
    completed_inspection_count,
    evaluate_navigation_acceptance,
)


TERMINAL_STATES = {'COMPLETE', 'BLOCKED', 'ESTOP'}


def _load_scenario(path: Path, scenario_id: str) -> dict:
    document = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    scenarios = document.get('scenarios', [])
    if not isinstance(scenarios, list):
        raise ValueError('scenarios 必须是列表')
    for scenario in scenarios:
        if isinstance(scenario, dict) and scenario.get('id') == scenario_id:
            return scenario
    raise ValueError(f'找不到验收场景 {scenario_id}')


class NavigationAcceptanceRunner(Node):
    """Start only after explicit arming and continuously enforce limits."""

    def __init__(self) -> None:
        super().__init__('navigation_acceptance_runner')
        self.declare_parameter('scenario_file', '')
        self.declare_parameter('scenario_id', 'normal_route_low_speed')
        self.declare_parameter('armed', False)
        self.declare_parameter(
            'output_directory',
            '~/.ros/patrol_robot/navigation_acceptance',
        )
        self.declare_parameter('final_stop_stable_seconds', 1.0)

        scenario_path = Path(
            str(self.get_parameter('scenario_file').value)
        ).expanduser()
        if not scenario_path.is_file():
            raise ValueError(f'验收场景文件不存在: {scenario_path}')
        self._scenario_id = str(self.get_parameter('scenario_id').value)
        self._scenario = _load_scenario(scenario_path, self._scenario_id)
        self._limits = NavigationAcceptanceLimits.from_mapping(
            self._scenario
        )
        self._expected_route_id = str(
            self._scenario.get('expected_route_id', '')
        )
        self._armed = bool(self.get_parameter('armed').value)
        self._output_directory = Path(
            str(self.get_parameter('output_directory').value)
        ).expanduser()
        self._stop_stable_seconds = max(
            0.5,
            float(self.get_parameter('final_stop_stable_seconds').value),
        )

        ready_qos = QoSProfile(depth=1)
        ready_qos.reliability = ReliabilityPolicy.RELIABLE
        ready_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool, '/navigation_health/ready', self._on_ready, ready_qos
        )
        self.create_subscription(
            String, '/navigation_health/status', self._on_health_status,
            ready_qos,
        )
        self.create_subscription(
            String, '/patrol_manager/status', self._on_patrol_status, 10
        )
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(
            LaserScan, '/scan_raw', self._on_scan, 10
        )
        self.create_subscription(
            Twist, '/controller/cmd_vel', self._on_final_command, 10
        )

        self._start_client = self.create_client(
            Trigger, '/patrol_manager/start'
        )
        self._stop_client = self.create_client(
            Trigger, '/patrol_manager/stop'
        )
        self._watchdog_parameters = self.create_client(
            GetParameters, '/base_command_watchdog/get_parameters'
        )

        now = time.monotonic()
        self._phase = 'WAITING'
        self._created_at = now
        self._ready = False
        self._ready_since: float | None = None
        self._patrol_status: dict = {}
        self._health_status: dict = {}
        self._state_history: list[str] = []
        self._last_pose: tuple[float, float] | None = None
        self._path_length = 0.0
        self._minimum_obstacle_distance = math.inf
        self._maximum_linear_speed = 0.0
        self._maximum_angular_speed = 0.0
        self._maximum_retry_count = 0
        self._health_false_events = 0
        self._maximum_map_correction_translation = 0.0
        self._maximum_map_correction_yaw_degrees = 0.0
        self._command_samples = 0
        self._last_nonzero_command = now
        self._run_started: float | None = None
        self._terminal_state: str | None = None
        self._stop_reason: str | None = None
        self._watchdog_verified = False
        self._watchdog_request_pending = False
        self._service_request_pending = False
        self._done = False
        self._passed = False
        self._report_path: Path | None = None
        self.create_timer(0.1, self._tick)

        if not self._armed:
            self.get_logger().error(
                '验收执行器未武装：必须显式设置 armed:=true'
            )

    @property
    def done(self) -> bool:
        return self._done

    @property
    def report_path(self) -> Path | None:
        return self._report_path

    @property
    def passed(self) -> bool:
        return self._passed

    def _on_ready(self, message: Bool) -> None:
        now = time.monotonic()
        ready = bool(message.data)
        if ready:
            if not self._ready:
                self._ready_since = now
        else:
            self._ready_since = None
            if self._phase == 'RUNNING' and self._ready:
                self._health_false_events += 1
                self._request_forced_stop('navigation_health_false')
        self._ready = ready

    def _on_health_status(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(status, dict):
            return
        self._health_status = status
        self._maximum_map_correction_translation = max(
            self._maximum_map_correction_translation,
            abs(float(status.get('global_pose_translation_delta') or 0.0)),
        )
        self._maximum_map_correction_yaw_degrees = max(
            self._maximum_map_correction_yaw_degrees,
            abs(float(status.get('global_pose_yaw_delta_degrees') or 0.0)),
        )

    def _on_patrol_status(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(status, dict):
            return
        self._patrol_status = status
        state = str(status.get('state', 'UNKNOWN')).upper()
        if not self._state_history or self._state_history[-1] != state:
            self._state_history.append(state)
        self._maximum_retry_count = max(
            self._maximum_retry_count,
            int(status.get('retry_count') or 0),
        )
        if self._phase == 'RUNNING' and state in TERMINAL_STATES:
            self._terminal_state = state
            self._phase = 'SETTLING'

    def _on_odom(self, message: Odometry) -> None:
        pose = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )
        if not all(math.isfinite(value) for value in pose):
            return
        if self._phase == 'RUNNING' and self._last_pose is not None:
            step = math.hypot(
                pose[0] - self._last_pose[0],
                pose[1] - self._last_pose[1],
            )
            if step <= 2.0:
                self._path_length += step
        self._last_pose = pose

    def _on_scan(self, message: LaserScan) -> None:
        if self._phase != 'RUNNING':
            return
        finite = [float(value) for value in message.ranges
                  if math.isfinite(value)]
        if finite:
            self._minimum_obstacle_distance = min(
                self._minimum_obstacle_distance, min(finite)
            )

    def _on_final_command(self, message: Twist) -> None:
        linear = math.hypot(float(message.linear.x), float(message.linear.y))
        angular = abs(float(message.angular.z))
        self._command_samples += 1
        self._maximum_linear_speed = max(self._maximum_linear_speed, linear)
        self._maximum_angular_speed = max(
            self._maximum_angular_speed, angular
        )
        if linear > 1e-3 or angular > 1e-3:
            self._last_nonzero_command = time.monotonic()
        if self._phase == 'RUNNING':
            if linear > self._limits.maximum_linear_speed + 0.005:
                self._request_forced_stop('linear_speed_limit_exceeded')
            elif angular > self._limits.maximum_angular_speed + 0.005:
                self._request_forced_stop('angular_speed_limit_exceeded')

    def _request_watchdog_parameters(self) -> None:
        if self._watchdog_request_pending:
            return
        if not self._watchdog_parameters.service_is_ready():
            return
        request = GetParameters.Request()
        request.names = ['max_linear_speed', 'max_angular_speed']
        self._watchdog_request_pending = True
        future = self._watchdog_parameters.call_async(request)
        future.add_done_callback(self._on_watchdog_parameters)

    def _on_watchdog_parameters(self, future) -> None:
        self._watchdog_request_pending = False
        try:
            values = future.result().values
            linear = float(values[0].double_value)
            angular = float(values[1].double_value)
        except Exception as error:
            self.get_logger().warning(f'读取最终看门狗限速失败: {error}')
            return
        if (
            linear > self._limits.maximum_linear_speed + 1e-6
            or angular > self._limits.maximum_angular_speed + 1e-6
        ):
            self._finish_without_motion(
                'watchdog_limits_exceed_scenario',
                details={
                    'watchdog_max_linear_speed': linear,
                    'watchdog_max_angular_speed': angular,
                },
            )
            return
        self._watchdog_verified = True

    def _tick(self) -> None:
        if self._done:
            return
        now = time.monotonic()
        if not self._armed:
            self._finish_without_motion('not_armed')
            return
        if self._phase == 'WAITING':
            self._request_watchdog_parameters()
            route_id = str(self._patrol_status.get('route_id', ''))
            route_matches = (
                not self._expected_route_id
                or route_id == self._expected_route_id
            )
            state = str(self._patrol_status.get('state', '')).upper()
            ready_stable = (
                self._ready
                and self._ready_since is not None
                and now - self._ready_since
                >= self._limits.readiness_stable_seconds
            )
            if (
                self._watchdog_verified
                and route_matches
                and state == 'PAUSED'
                and ready_stable
                and not self._service_request_pending
            ):
                self._service_request_pending = True
                future = self._start_client.call_async(Trigger.Request())
                future.add_done_callback(self._on_start_response)
            elif now - self._created_at > 60.0:
                self._finish_without_motion(
                    'preconditions_timeout',
                    details={
                        'route_matches': route_matches,
                        'patrol_state': state,
                        'navigation_health_ready': self._ready,
                        'watchdog_verified': self._watchdog_verified,
                    },
                )
            return
        if self._phase == 'RUNNING':
            if (
                self._run_started is not None
                and now - self._run_started > self._limits.timeout_seconds
            ):
                self._request_forced_stop('scenario_timeout')
            return
        if self._phase == 'SETTLING':
            stopped = (
                self._command_samples > 0
                and now - self._last_nonzero_command
                >= self._stop_stable_seconds
            )
            if stopped:
                self._write_report(final_stop_verified=True)

    def _on_start_response(self, future) -> None:
        self._service_request_pending = False
        try:
            response = future.result()
        except Exception as error:
            self._finish_without_motion(
                'start_service_error', details={'error': str(error)}
            )
            return
        if not response.success:
            self._finish_without_motion(
                'start_rejected', details={'message': response.message}
            )
            return
        self._phase = 'RUNNING'
        self._run_started = time.monotonic()
        self._last_nonzero_command = self._run_started
        self.get_logger().warning(
            f'真车导航验收已启动: {self._scenario_id}'
        )

    def _request_forced_stop(self, reason: str) -> None:
        if self._phase not in {'RUNNING', 'STOPPING'}:
            return
        if self._phase == 'STOPPING':
            return
        self._phase = 'STOPPING'
        self._stop_reason = reason
        self._terminal_state = 'ABORTED'
        future = self._stop_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_stop_response)
        self.get_logger().error(f'验收保护停车: {reason}')

    def _on_stop_response(self, future) -> None:
        try:
            response = future.result()
            if not response.success:
                self._stop_reason = (
                    f'{self._stop_reason}; stop_rejected={response.message}'
                )
        except Exception as error:
            self._stop_reason = f'{self._stop_reason}; stop_error={error}'
        self._phase = 'SETTLING'

    def _finish_without_motion(
        self, reason: str, details: dict | None = None
    ) -> None:
        self._stop_reason = reason
        self._terminal_state = 'PRECONDITION_FAILED'
        self._write_report(
            final_stop_verified=self._maximum_linear_speed <= 1e-3
            and self._maximum_angular_speed <= 1e-3,
            details=details,
        )

    def _write_report(
        self,
        final_stop_verified: bool,
        details: dict | None = None,
    ) -> None:
        if self._done:
            return
        finished = time.monotonic()
        started = self._run_started or finished
        report = {
            'schema_version': 1,
            'scenario_id': self._scenario_id,
            'created_at': datetime.now(timezone.utc).isoformat(),
            'terminal_state': self._terminal_state or 'UNKNOWN',
            'stop_reason': self._stop_reason,
            'elapsed_seconds': round(max(0.0, finished - started), 3),
            'path_length_meters': round(self._path_length, 4),
            'minimum_obstacle_distance_meters': (
                None if not math.isfinite(self._minimum_obstacle_distance)
                else round(self._minimum_obstacle_distance, 4)
            ),
            'maximum_linear_speed': round(self._maximum_linear_speed, 4),
            'maximum_angular_speed': round(self._maximum_angular_speed, 4),
            'maximum_retry_count': self._maximum_retry_count,
            'health_false_events': self._health_false_events,
            'maximum_map_correction_translation': round(
                self._maximum_map_correction_translation, 4
            ),
            'maximum_map_correction_yaw_degrees': round(
                self._maximum_map_correction_yaw_degrees, 3
            ),
            'completed_inspections': completed_inspection_count(
                self._patrol_status
            ),
            'completed_loops': int(
                self._patrol_status.get('completed_loops') or 0
            ),
            'final_stop_verified': bool(final_stop_verified),
            'command_samples': self._command_samples,
            'state_history': self._state_history,
            'final_patrol_status': self._patrol_status,
            'final_navigation_health_status': self._health_status,
            'details': details or {},
        }
        report['evaluation'] = evaluate_navigation_acceptance(
            report, self._limits
        )
        self._passed = bool(report['evaluation']['passed'])
        timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        self._output_directory.mkdir(parents=True, exist_ok=True)
        output = self._output_directory / (
            f'{self._scenario_id}-{timestamp}.json'
        )
        output.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8',
        )
        self._report_path = output
        self._done = True
        if report['evaluation']['passed']:
            self.get_logger().info(f'导航验收通过: {output}')
        else:
            self.get_logger().error(f'导航验收未通过: {output}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationAcceptanceRunner()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        if not node.done:
            node._request_forced_stop('runner_interrupted')
    finally:
        passed = node.passed
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not passed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
