import math
import json
import struct
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.msg import SpeedLimit
from nav2_msgs.srv import ClearEntireCostmap
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger
from std_msgs.msg import Bool, Float32, Header, String

from patrol_robot_patrol.failed_path_memory import FailedPathMemory
from patrol_robot_patrol.health_recovery_progress import HealthRecoveryProgress
from patrol_robot_patrol.navigation_failure import (
    navigation_error_label,
    should_blacklist_route,
)
from patrol_robot_patrol.navigation_motion_guard import (
    MotionGuardStatus,
    NavigationMotionGuard,
)
from patrol_robot_patrol.route_model import PatrolRoute, Waypoint, load_route, parse_route
from patrol_robot_patrol.task_ledger import PatrolTaskLedger


class PatrolManager(Node):
    """Execute every waypoint as an individually acknowledged patrol task."""

    def __init__(self) -> None:
        super().__init__('patrol_manager')

        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('autostart', True)
        self.declare_parameter('loop', True)
        self.declare_parameter('loop_count', 1)
        self.declare_parameter('default_dwell_seconds', 2.0)
        self.declare_parameter('start_delay_seconds', 8.0)
        self.declare_parameter('goal_timeout_seconds', 120.0)
        self.declare_parameter('max_retries', 2)
        self.declare_parameter('retry_delay_seconds', 3.0)
        self.declare_parameter('lap_restart_delay_seconds', 1.5)
        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('publish_nav2_speed_limit_direct', False)
        self.declare_parameter('behavior_tree', '')
        self.declare_parameter('behavior_tree_no_spin', '')
        self.declare_parameter('behavior_tree_restricted', '')
        self.declare_parameter('require_navigation_health', False)
        self.declare_parameter('enforce_required_sensors', False)
        self.declare_parameter('health_recovery_timeout_seconds', 8.0)
        self.declare_parameter('health_recovery_stable_seconds', 1.5)
        self.declare_parameter('max_health_recoveries', 3)
        self.declare_parameter('health_recovery_reset_stable_seconds', 5.0)
        self.declare_parameter('health_recovery_reset_progress_meters', 0.5)
        self.declare_parameter('failed_path_start_clearance', 0.80)
        self.declare_parameter('failed_path_goal_clearance', 0.80)
        self.declare_parameter('failed_path_band_radius', 0.18)
        self.declare_parameter('failed_path_similarity_distance', 0.75)
        self.declare_parameter('failed_path_similarity_ratio', 0.70)
        self.declare_parameter('max_similar_path_replans', 6)
        self.declare_parameter('similar_path_replan_delay_seconds', 1.25)
        self.declare_parameter('max_route_failures', 6)
        self.declare_parameter('enable_motion_spin_guard', False)
        self.declare_parameter('motion_spin_window_seconds', 2.0)
        self.declare_parameter('motion_spin_max_translation', 0.08)
        self.declare_parameter('motion_spin_min_yaw_degrees', 20.0)
        self.declare_parameter('motion_spin_min_goal_progress', 0.02)

        self._loop = self.get_parameter('loop').value
        self._loop_count = max(1, int(self.get_parameter('loop_count').value))
        self._autostart = self.get_parameter('autostart').value
        self._default_dwell = float(
            self.get_parameter('default_dwell_seconds').value
        )
        self._start_delay = max(
            0.0, float(self.get_parameter('start_delay_seconds').value)
        )
        self._goal_timeout = max(
            1.0, float(self.get_parameter('goal_timeout_seconds').value)
        )
        self._max_retries = max(0, int(self.get_parameter('max_retries').value))
        self._retry_delay = max(
            0.0, float(self.get_parameter('retry_delay_seconds').value)
        )
        self._lap_restart_delay = max(
            0.5,
            float(self.get_parameter('lap_restart_delay_seconds').value),
        )
        self._behavior_tree = str(self.get_parameter('behavior_tree').value)
        self._publish_nav2_speed_limit_direct = bool(
            self.get_parameter('publish_nav2_speed_limit_direct').value
        )
        self._behavior_trees = {
            'standard': self._behavior_tree,
            'no_spin': str(self.get_parameter('behavior_tree_no_spin').value),
            'restricted': str(
                self.get_parameter('behavior_tree_restricted').value
            ),
        }
        self._require_navigation_health = bool(
            self.get_parameter('require_navigation_health').value
        )
        self._enforce_required_sensors = bool(
            self.get_parameter('enforce_required_sensors').value
        )
        self._health_recovery_timeout = max(
            2.0,
            float(self.get_parameter('health_recovery_timeout_seconds').value),
        )
        self._health_recovery_stable = max(
            0.5,
            float(self.get_parameter('health_recovery_stable_seconds').value),
        )
        self._max_health_recoveries = max(
            0,
            int(self.get_parameter('max_health_recoveries').value),
        )
        self._health_recovery_reset_stable = max(
            0.5,
            float(
                self.get_parameter(
                    'health_recovery_reset_stable_seconds'
                ).value
            ),
        )
        self._health_recovery_reset_progress = max(
            0.05,
            float(
                self.get_parameter(
                    'health_recovery_reset_progress_meters'
                ).value
            ),
        )
        self._failed_paths = FailedPathMemory(
            start_clearance=float(
                self.get_parameter('failed_path_start_clearance').value
            ),
            goal_clearance=float(
                self.get_parameter('failed_path_goal_clearance').value
            ),
            band_radius=float(
                self.get_parameter('failed_path_band_radius').value
            ),
            similarity_distance=float(
                self.get_parameter('failed_path_similarity_distance').value
            ),
            similarity_ratio=float(
                self.get_parameter('failed_path_similarity_ratio').value
            ),
        )
        self._max_similar_path_replans = max(
            1, int(self.get_parameter('max_similar_path_replans').value)
        )
        self._similar_path_replan_delay = max(
            1.0,
            float(
                self.get_parameter('similar_path_replan_delay_seconds').value
            ),
        )
        self._max_route_failures = max(
            1, int(self.get_parameter('max_route_failures').value)
        )
        self._enable_motion_spin_guard = bool(
            self.get_parameter('enable_motion_spin_guard').value
        )
        self._motion_guard = NavigationMotionGuard(
            float(self.get_parameter('motion_spin_window_seconds').value),
            float(self.get_parameter('motion_spin_max_translation').value),
            math.radians(float(
                self.get_parameter('motion_spin_min_yaw_degrees').value
            )),
            float(
                self.get_parameter('motion_spin_min_goal_progress').value
            ),
        )

        waypoint_file = str(self.get_parameter('waypoint_file').value)
        if not waypoint_file:
            raise ValueError('参数 waypoint_file 不能为空')
        self._set_route(load_route(waypoint_file, self._default_dwell))

        action_name = str(self.get_parameter('action_name').value)
        self._navigation = ActionClient(self, NavigateToPose, action_name)
        self._path_preflight = ActionClient(
            self,
            ComputePathToPose,
            'compute_path_to_pose',
        )
        self._controller_parameters = self.create_client(
            SetParameters,
            '/controller_server/set_parameters',
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
        self.create_service(Trigger, '~/start', self._on_start)
        self.create_service(Trigger, '~/stop', self._on_stop)
        self.create_service(Trigger, '~/reset', self._on_reset)
        self.create_service(Trigger, '~/estop', self._on_estop)
        self.create_service(Trigger, '~/clear_estop', self._on_clear_estop)
        self.create_subscription(String, '~/set_waypoints', self._on_set_waypoints, 10)
        self.create_subscription(
            Bool,
            '/navigation_health/ready',
            self._on_navigation_health,
            10,
        )
        self.create_subscription(
            String,
            '/navigation_health/status',
            self._on_navigation_health_status,
            10,
        )
        self.create_subscription(Path, '/plan', self._on_global_plan, 10)
        self._status_publisher = self.create_publisher(String, '~/status', 10)
        self._failed_path_publisher = self.create_publisher(
            PointCloud2,
            '~/failed_path_points',
            10,
        )
        selector_qos = QoSProfile(depth=1)
        selector_qos.reliability = ReliabilityPolicy.RELIABLE
        selector_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._goal_checker_publisher = self.create_publisher(
            String,
            '/goal_checker_selector',
            selector_qos,
        )
        self._speed_limit_publisher = self.create_publisher(
            Float32,
            '~/speed_limit',
            selector_qos,
        )
        self._nav2_speed_limit_publisher = (
            self.create_publisher(SpeedLimit, '/speed_limit', selector_qos)
            if self._publish_nav2_speed_limit_direct else None
        )

        self._route_cursor = 0
        self._index = self._route.home_index
        self._retry_count = 0
        self._state = 'START_DELAY' if self._autostart else 'PAUSED'
        self._state_deadline = self._now() + self._start_delay
        self._goal_started_at = 0.0
        self._goal_handle = None
        self._goal_token = 0
        self._policy_generation = 0
        self._preflight_generation = 0
        self._active_token: Optional[int] = None
        self._cancel_reason: Optional[str] = None
        self._last_feedback_log = 0.0
        self._completed_loops = 0
        self._returning_home = False
        self._task_ledger = PatrolTaskLedger(
            len(self._waypoints), self._loop_count, self._route.task_indices
        )
        self._dwell_deadline = 0.0
        self._blocked_reason: Optional[str] = None
        self._last_failure_status: Optional[int] = None
        self._last_failure_error_code: Optional[int] = None
        self._last_failure_error_message: Optional[str] = None
        self._navigation_health_ready = not self._require_navigation_health
        self._navigation_health_reason: Optional[str] = None
        self._navigation_health_checks: dict[str, bool] = {}
        self._health_recovery_count = 0
        self._health_recovery_deadline = 0.0
        self._health_ready_since: Optional[float] = None
        self._health_recovery_progress = HealthRecoveryProgress(
            self._health_recovery_reset_stable,
            self._health_recovery_reset_progress,
        )
        self._active_plan: list[tuple[float, float]] = []
        self._last_robot_position: Optional[tuple[float, float]] = None
        self._goal_received_feedback = False
        self._similar_path_replan_count = 0
        self._last_candidate_similarity = 0.0
        self._route_failure_count = 0
        self._route_replan_pending = False
        self._motion_guard_status: Optional[MotionGuardStatus] = None
        self._motion_guard_triggered = False

        self.create_timer(0.25, self._tick)
        self.create_timer(0.5, self._publish_status)
        self.create_timer(0.5, self._publish_failed_path_points)
        self.get_logger().info(
            f'已加载路线 {self._route.route_id}，共 {len(self._waypoints)} 个点，'
            f'巡检任务 {len(self._route.task_indices)} 个，坐标系={self._frame_id}，'
            f'自动启动={self._autostart}，循环={self._loop}'
        )

    def _set_route(self, route: PatrolRoute) -> None:
        self._route = route
        self._frame_id = route.frame_id
        self._waypoints = list(route.waypoints)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _ready_state(self) -> str:
        if self._require_navigation_health and not self._navigation_health_ready:
            return 'WAITING_HEALTH'
        if not self._required_sensor_ready(self._waypoints[self._index]):
            return 'WAITING_SENSOR'
        return 'WAITING_SERVER'

    def _required_sensor_ready(self, waypoint: Waypoint) -> bool:
        if not self._enforce_required_sensors:
            return True
        sensor = waypoint.required_sensor
        if sensor == 'none':
            return True
        lidar_ok = bool(self._navigation_health_checks.get('scan_ok'))
        camera_ok = bool(self._navigation_health_checks.get('camera_ok'))
        if sensor == 'lidar':
            return lidar_ok
        if sensor == 'rgbd':
            return camera_ok
        return lidar_ok and camera_ok

    def _on_navigation_health_status(self, message: String) -> None:
        try:
            status = json.loads(message.data)
            reason = status.get('reason')
            self._navigation_health_reason = str(reason) if reason else None
            checks = status.get('checks')
            self._navigation_health_checks = (
                checks if isinstance(checks, dict) else {}
            )
            if (
                not self._navigation_health_ready
                and self._state in ('BLOCKED', 'HEALTH_RECOVERY')
                and self._navigation_health_reason
            ):
                self._blocked_reason = self._navigation_health_reason
            if (
                self._state == 'HEALTH_RECOVERY'
                and self._health_fault_requires_manual()
            ):
                self._block_health_recovery(
                    self._navigation_health_reason
                    or '底盘驱动或硬件急停未处于安全状态'
                )
            if (
                self._state == 'WAITING_SENSOR'
                and self._required_sensor_ready(self._waypoints[self._index])
            ):
                self._state = 'WAITING_SERVER'
                self.get_logger().info('路线点要求的传感器已就绪')
        except (TypeError, json.JSONDecodeError):
            self._navigation_health_reason = '导航健康状态格式无效'

    def _on_navigation_health(self, message: Bool) -> None:
        ready = bool(message.data)
        self._navigation_health_ready = ready
        if ready:
            if self._state == 'HEALTH_RECOVERY':
                if self._health_ready_since is None:
                    self._health_ready_since = self._now()
                    self.get_logger().info(
                        '导航健康已恢复，正在确认状态持续稳定'
                    )
                return
            if self._state == 'WAITING_HEALTH':
                self._state = self._ready_state()
                self.get_logger().info('导航健康检查已通过，允许发送巡检目标')
            return
        if not self._require_navigation_health:
            return
        if self._state == 'HEALTH_RECOVERY':
            self._health_ready_since = None
            return
        unsafe_states = {
            'WAITING_SERVER',
            'SENDING_GOAL',
            'CONFIGURING_GOAL',
            'CHECKING_PATH',
            'NAVIGATING',
            'DWELL',
            'RETRY_WAIT',
        }
        if self._state in unsafe_states:
            self._blocked_reason = (
                self._navigation_health_reason or '导航健康检查未通过'
            )
            self.get_logger().error(
                f'导航健康状态失效：{self._blocked_reason}；正在安全停车'
            )
            self._request_cancel('health')

    def _health_fault_requires_manual(self) -> bool:
        return bool(
            self._navigation_health_checks.get('base_driver_ok') is False
            or self._navigation_health_checks.get('estop_released') is False
        )

    def _block_health_recovery(self, reason: str) -> None:
        self._health_ready_since = None
        self._health_recovery_progress.clear()
        self._blocked_reason = reason
        self._state = 'BLOCKED'
        self.get_logger().error(
            f'导航健康自动恢复终止：{reason}；等待人工确认'
        )

    def _begin_health_recovery(self) -> None:
        reason = self._navigation_health_reason or '导航健康检查未通过'
        self._blocked_reason = reason
        if self._health_fault_requires_manual():
            self._block_health_recovery(reason)
            return
        if self._health_recovery_count >= self._max_health_recoveries:
            self._block_health_recovery(
                f'{reason}；自动恢复已达到 {self._max_health_recoveries} 次上限'
            )
            return
        self._health_recovery_count += 1
        # Every new health event must prove a fresh stretch of safe motion.
        # Repeated faults at the same bottleneck therefore remain consecutive,
        # while faults separated by meaningful healthy travel start a new streak.
        self._health_recovery_progress.arm()
        now = self._now()
        self._health_recovery_deadline = now + self._health_recovery_timeout
        self._health_ready_since = now if self._navigation_health_ready else None
        self._state = 'HEALTH_RECOVERY'
        self._clear_costmaps('导航健康瞬态故障')
        self.get_logger().warning(
            f'车辆已安全停车；等待健康状态稳定后自动续行 '
            f'({self._health_recovery_count}/{self._max_health_recoveries})'
        )

    def _tick_health_recovery(self, now: float) -> None:
        if self._health_fault_requires_manual():
            self._block_health_recovery(
                self._navigation_health_reason
                or '底盘驱动或硬件急停未处于安全状态'
            )
            return
        if now >= self._health_recovery_deadline:
            self._block_health_recovery(
                f'{self._blocked_reason or "导航健康检查未通过"}；'
                f'{self._health_recovery_timeout:.1f} 秒内未稳定恢复'
            )
            return
        if not self._navigation_health_ready:
            self._health_ready_since = None
            return
        if self._health_ready_since is None:
            self._health_ready_since = now
            return
        if now - self._health_ready_since < self._health_recovery_stable:
            return
        self._blocked_reason = None
        self._last_failure_status = None
        self._last_failure_error_code = None
        self._last_failure_error_message = None
        self._health_ready_since = None
        self._state = 'RETRY_WAIT'
        self._state_deadline = now + min(self._retry_delay, 1.0)
        self.get_logger().warning(
            '导航健康已持续稳定，正在自动低速重试当前路线点'
        )

    def _on_set_waypoints(self, message: String) -> None:
        """Replace the active route from a JSON message without restarting."""
        try:
            document = json.loads(message.data)
            route = parse_route(document, self._default_dwell)
            self._request_cancel('reset')
            self._set_route(route)
            self._loop_count = max(1, min(int(document.get('loop_count', 1)), 1000))
            self._route_cursor = 0
            self._index = self._route.home_index
            self._retry_count = 0
            self._reset_health_recovery_streak()
            self._completed_loops = 0
            self._returning_home = False
            self._task_ledger = PatrolTaskLedger(
                len(self._waypoints), self._loop_count, self._route.task_indices
            )
            self._dwell_deadline = 0.0
            self._blocked_reason = None
            self._last_failure_status = None
            self._last_failure_error_code = None
            self._last_failure_error_message = None
            self._state = 'PAUSED'
            self.get_logger().info(
                f'网页已更新语义路线 {route.route_id}，'
                f'共 {len(route.waypoints)} 个点，'
                f'{len(route.task_indices)} 个巡检任务，'
                f'计划 {self._loop_count} 圈'
            )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().error(f'网页巡检点配置无效: {error}')

    def _publish_status(self) -> None:
        message = String()
        current = self._waypoints[self._index] if self._waypoints else None
        running = self._state not in ('PAUSED', 'BLOCKED', 'ESTOP', 'COMPLETE')
        tasks = []
        route_positions = {
            index: position
            for position, index in enumerate(self._route.ordered_indices)
        }
        for index, waypoint in enumerate(self._waypoints):
            is_current = index == self._index
            if is_current and self._state == 'BLOCKED':
                task_state = 'blocked'
            elif waypoint.waypoint_type == 'HOME':
                task_state = 'returning' if self._returning_home and running else 'home'
            elif is_current and self._state == 'DWELL':
                task_state = 'dwell'
            elif is_current and running:
                task_state = 'active'
            elif waypoint.count_as_task and self._task_ledger.remaining[index] == 0:
                task_state = 'complete'
            elif (
                not waypoint.count_as_task
                and route_positions[index] < self._route_cursor
            ):
                task_state = 'passed'
            else:
                task_state = 'pending'
            tasks.append({
                'index': index,
                'id': waypoint.waypoint_id,
                'name': waypoint.name,
                'role': waypoint.role,
                'type': waypoint.waypoint_type,
                'count_as_task': waypoint.count_as_task,
                'remaining_visits': (
                    self._task_ledger.remaining[index]
                    if waypoint.count_as_task else None
                ),
                'completed_visits': (
                    self._task_ledger.completed_visits(index)
                    if waypoint.count_as_task else None
                ),
                'position_tolerance': waypoint.position_tolerance,
                'yaw_tolerance': waypoint.yaw_tolerance,
                'speed_limit': waypoint.speed_limit,
                'recovery_policy': waypoint.recovery_policy,
                'required_sensor': waypoint.required_sensor,
                'state': task_state,
            })
        message.data = json.dumps({
            'route_id': self._route.route_id,
            'state': self._state,
            'running': running,
            'current_index': self._index,
            'current_waypoint_id': current.waypoint_id if current else None,
            'current_waypoint': current.name if current else None,
            'current_waypoint_type': current.waypoint_type if current else None,
            'current_goal': ({
                'x': current.x,
                'y': current.y,
                'yaw': current.yaw,
            } if current else None),
            'speed_limit': current.speed_limit if current else None,
            'position_tolerance': current.position_tolerance if current else None,
            'yaw_tolerance': current.yaw_tolerance if current else None,
            'recovery_policy': current.recovery_policy if current else None,
            'waypoint_count': len(self._waypoints),
            'inspection_task_count': len(self._route.task_indices),
            'loop_count': self._loop_count,
            'completed_loops': self._completed_loops,
            'current_loop': min(
                self._loop_count,
                self._completed_loops + 1,
            ),
            'remaining_loops': max(
                0, self._loop_count - self._completed_loops
            ),
            'returning_home': self._returning_home,
            'retry_count': self._retry_count,
            'max_retries': self._max_retries,
            'automatic_retry': (
                (
                    self._state == 'RETRY_WAIT'
                    and (
                        self._retry_count > 0
                        or self._health_recovery_count > 0
                    )
                )
                or self._state == 'HEALTH_RECOVERY'
            ),
            'health_recovery_count': self._health_recovery_count,
            'max_health_recoveries': self._max_health_recoveries,
            'health_recovery_reset_pending': (
                self._health_recovery_count > 0
                and self._health_recovery_progress.armed
            ),
            'health_recovery_safe_progress_meters': round(
                self._health_recovery_progress.progress_meters,
                2,
            ),
            'health_recovery_safe_seconds': round(
                self._health_recovery_progress.elapsed_seconds(self._now()),
                1,
            ),
            'health_recovery_reset_progress_meters': (
                self._health_recovery_reset_progress
            ),
            'health_recovery_reset_stable_seconds': (
                self._health_recovery_reset_stable
            ),
            'blocked_reason': self._blocked_reason,
            'last_failure_status': self._last_failure_status,
            'last_failure_error_code': self._last_failure_error_code,
            'last_failure_error_message': self._last_failure_error_message,
            'failed_route_count': self._failed_paths.route_count,
            'failed_route_exclusion_points': len(self._failed_paths.points),
            'route_failure_count': self._route_failure_count,
            'max_route_failures': self._max_route_failures,
            'route_replan_pending': self._route_replan_pending,
            'similar_path_replan_count': self._similar_path_replan_count,
            'last_candidate_similarity': round(
                self._last_candidate_similarity, 3
            ),
            'navigation_health_ready': self._navigation_health_ready,
            'navigation_health_reason': self._navigation_health_reason,
            'motion_spin_guard_enabled': self._enable_motion_spin_guard,
            'motion_spin_guard_triggered': self._motion_guard_triggered,
            'motion_spin_guard': ({
                'elapsed_seconds': round(
                    self._motion_guard_status.elapsed, 2
                ),
                'translation_meters': round(
                    self._motion_guard_status.translation, 3
                ),
                'yaw_change_degrees': round(math.degrees(
                    self._motion_guard_status.yaw_change
                ), 2),
                'goal_progress_meters': round(
                    self._motion_guard_status.distance_progress, 3
                ),
            } if self._motion_guard_status is not None else None),
            'required_sensor_ready': (
                self._required_sensor_ready(current) if current else False
            ),
            'waypoint_tasks': tasks,
        }, ensure_ascii=False)
        self._status_publisher.publish(message)

    def _tick(self) -> None:
        now = self._now()

        if self._state == 'HEALTH_RECOVERY':
            self._tick_health_recovery(now)
            return

        if self._state == 'DWELL':
            if now >= self._dwell_deadline:
                self._advance_after_inspection()
            return

        if self._state in ('START_DELAY', 'RETRY_WAIT'):
            if now >= self._state_deadline:
                self._state = self._ready_state()

        if self._state == 'WAITING_HEALTH':
            return

        if self._state == 'WAITING_SENSOR':
            return

        if self._state == 'WAITING_SERVER':
            if (
                self._navigation.wait_for_server(timeout_sec=0.0)
                and self._path_preflight.wait_for_server(timeout_sec=0.0)
                and self._controller_parameters.service_is_ready()
            ):
                self._check_current_path()
            return

        if self._state == 'NAVIGATING':
            if now - self._goal_started_at >= self._goal_timeout:
                waypoint = self._waypoints[self._index]
                self.get_logger().warning(
                    f'巡航点“{waypoint.name}”导航超时，正在取消目标'
                )
                self._request_cancel('timeout')

    def _pose_for_waypoint(self, waypoint: Waypoint) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        pose.pose.orientation.z = math.sin(waypoint.yaw / 2.0)
        pose.pose.orientation.w = math.cos(waypoint.yaw / 2.0)
        return pose

    def _check_current_path(self) -> None:
        self._active_plan = []
        self._last_robot_position = None
        self._goal_received_feedback = False
        goal = ComputePathToPose.Goal()
        goal.goal = self._pose_for_waypoint(self._waypoints[self._index])
        goal.planner_id = 'GridBased'
        goal.use_start = False
        self._preflight_generation += 1
        generation = self._preflight_generation
        self._state = 'CHECKING_PATH'
        future = self._path_preflight.send_goal_async(goal)
        future.add_done_callback(
            lambda result, token=generation: self._on_preflight_response(
                result, token
            )
        )

    def _on_preflight_response(self, future, generation: int) -> None:
        if (
            generation != self._preflight_generation
            or self._state != 'CHECKING_PATH'
        ):
            return
        try:
            goal_handle = future.result()
        except Exception as error:
            self._block_preflight(f'路径预检请求失败: {error}')
            return
        if not goal_handle.accepted:
            self._block_preflight('规划器拒绝路径预检请求')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, token=generation: self._on_preflight_result(
                result, token
            )
        )

    def _on_preflight_result(self, future, generation: int) -> None:
        if (
            generation != self._preflight_generation
            or self._state != 'CHECKING_PATH'
        ):
            return
        try:
            wrapped = future.result()
            path = wrapped.result.path
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not path.poses:
                error_code = getattr(wrapped.result, 'error_code', None)
                self._block_preflight(
                    f'当前路线点不存在可行路径，错误码={error_code}'
                )
                return
        except Exception as error:
            self._block_preflight(f'读取路径预检结果失败: {error}')
            return
        candidate = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in path.poses
        ]
        # Keep the preflight action's exact raw candidate so an immediate
        # FollowPath rejection can still blacklist the complete route.
        self._active_plan = candidate
        # An immediate controller rejection may arrive before the first
        # NavigateToPose feedback. Never crop the new path with a robot pose
        # left over from the previous waypoint.
        self._last_robot_position = candidate[0]
        similar, similarity = self._failed_paths.is_similar(candidate)
        self._last_candidate_similarity = similarity
        if similar:
            self._reject_similar_candidate(candidate, similarity)
            return
        if self._similar_path_replan_count:
            self.get_logger().info(
                f'已找到不同候选路线，与失败路线最大重合度 '
                f'{similarity:.0%}；允许发送给局部控制器'
            )
        self._similar_path_replan_count = 0
        self._route_replan_pending = False
        self._configure_current_goal()

    def _reject_similar_candidate(
        self,
        candidate: list[tuple[float, float]],
        similarity: float,
    ) -> None:
        if self._similar_path_replan_count >= self._max_similar_path_replans:
            self._block_preflight(
                '连续规划结果均与已失败路线高度重合，'
                f'最后重合度={similarity:.0%}'
            )
            return

        added = self._failed_paths.remember(candidate, candidate[0])
        self._similar_path_replan_count += 1
        self._route_replan_pending = True
        self._publish_failed_path_points()
        self._clear_global_costmap('拒绝高度相似的候选路线')
        self._state = 'RETRY_WAIT'
        self._state_deadline = self._now() + self._similar_path_replan_delay
        self.get_logger().warning(
            f'候选路线与失败路线重合 {similarity:.0%}，'
            '已拒绝并加入黑名单；'
            f'{self._similar_path_replan_delay:.2f} 秒后重新选择'
            '未使用的最短路线 '
            f'({self._similar_path_replan_count}/'
            f'{self._max_similar_path_replans})；新增禁行栅格 {added}'
        )

    def _on_global_plan(self, message: Path) -> None:
        if not message.poses or not self._waypoints:
            return
        waypoint = self._waypoints[self._index]
        endpoint = message.poses[-1].pose.position
        if math.hypot(endpoint.x - waypoint.x, endpoint.y - waypoint.y) > max(
            0.50, waypoint.position_tolerance * 2.0
        ):
            return
        self._active_plan = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in message.poses
        ]

    def _block_preflight(self, reason: str) -> None:
        waypoint = self._waypoints[self._index]
        self.get_logger().warning(
            f'路线点“{waypoint.name}”路径预检失败：{reason}'
        )
        if self._route_replan_pending or self._failed_paths.route_count > 0:
            self._goal_handle = None
            self._active_token = None
            self._last_failure_status = GoalStatus.STATUS_ABORTED
            self._route_replan_pending = False
            self._blocked_reason = (
                f'{waypoint.name}：已禁用 {self._failed_paths.route_count} 条'
                f'失败路线后，没有找到未使用的可行替代路线（{reason}）'
            )
            self._state = 'BLOCKED'
            self.get_logger().error(
                f'路线点“{waypoint.name}”没有未使用的可行替代路线，'
                '已进入 BLOCKED；不会再低速重复同一路线'
            )
            return
        self._handle_failure(
            status=GoalStatus.STATUS_ABORTED,
            failure_detail=f'路径预检失败：{reason}',
        )

    def _configure_current_goal(self) -> None:
        waypoint = self._waypoints[self._index]
        goal_checker = (
            'transit_goal_checker'
            if waypoint.waypoint_type == 'TRANSIT'
            else 'precise_goal_checker'
        )
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                f'{goal_checker}.xy_goal_tolerance',
                value=waypoint.position_tolerance,
            ).to_parameter_msg(),
            Parameter(
                f'{goal_checker}.yaw_goal_tolerance',
                value=waypoint.yaw_tolerance,
            ).to_parameter_msg(),
        ]
        self._policy_generation += 1
        generation = self._policy_generation
        self._state = 'CONFIGURING_GOAL'
        future = self._controller_parameters.call_async(request)
        future.add_done_callback(
            lambda result, token=generation: self._on_goal_policy_configured(
                result, token
            )
        )

    def _on_goal_policy_configured(self, future, generation: int) -> None:
        if (
            generation != self._policy_generation
            or self._state != 'CONFIGURING_GOAL'
        ):
            return
        try:
            response = future.result()
            failures = [
                result.reason or '控制器拒绝参数'
                for result in response.results
                if not result.successful
            ]
        except Exception as error:
            failures = [str(error)]
        if failures:
            self.get_logger().error(
                '应用逐点导航容差失败: ' + '；'.join(failures)
            )
            self._handle_failure()
            return
        self._send_current_goal()

    def _send_current_goal(self) -> None:
        # Submit one pose per action. A closed route whose first and last pose
        # are identical can otherwise be reported as zero distance before its
        # intermediate inspection tasks have actually completed.
        waypoint = self._waypoints[self._index]
        pose = self._pose_for_waypoint(waypoint)

        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.behavior_tree = (
            self._behavior_trees.get(waypoint.recovery_policy)
            or self._behavior_tree
        )

        goal_checker = String()
        goal_checker.data = (
            'transit_goal_checker'
            if waypoint.waypoint_type == 'TRANSIT'
            else 'precise_goal_checker'
        )
        self._goal_checker_publisher.publish(goal_checker)
        effective_speed = self._publish_current_speed_limit()

        self._goal_token += 1
        token = self._goal_token
        self._active_token = token
        self._goal_handle = None
        self._cancel_reason = None
        self._goal_started_at = self._now()
        self._goal_received_feedback = False
        self._motion_guard.reset()
        self._motion_guard_status = None
        self._motion_guard_triggered = False
        self._state = 'SENDING_GOAL'

        if self._returning_home:
            description = f'返回基地“{waypoint.name}”'
        elif waypoint.waypoint_type == 'HOME':
            description = f'确认基地“{waypoint.name}”'
        elif waypoint.waypoint_type == 'TRANSIT':
            description = f'通过路线点“{waypoint.name}”'
        else:
            description = (
                f'执行巡检任务“{waypoint.name}”，剩余 '
                f'{self._task_ledger.remaining[self._index]} 次'
            )
        self.get_logger().info(
            f'开始单点导航：{description}；限速 {effective_speed:.2f} m/s，'
            f'容差 {waypoint.position_tolerance:.2f} m / '
            f'{math.degrees(waypoint.yaw_tolerance):.1f}°，'
            f'恢复策略 {waypoint.recovery_policy}'
        )
        future = self._navigation.send_goal_async(
            goal,
            feedback_callback=lambda feedback, goal_token=token:
                self._on_feedback(feedback, goal_token),
        )
        future.add_done_callback(
            lambda response, goal_token=token:
                self._on_goal_response(response, goal_token)
        )

    def _on_goal_response(self, future, token: int) -> None:
        if token != self._active_token:
            return
        try:
            goal_handle = future.result()
        except Exception as error:  # rclpy future transports the action exception.
            self.get_logger().error(f'发送导航目标失败: {error}')
            if self._cancel_reason is not None:
                self._finish_local_cancel(self._cancel_reason)
            else:
                self._handle_failure()
            return

        if not goal_handle.accepted:
            self.get_logger().warning('Nav2 拒绝了导航目标')
            if self._cancel_reason is not None:
                self._finish_local_cancel(self._cancel_reason)
            else:
                self._handle_failure()
            return

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, goal_token=token:
                self._on_result(result, goal_token)
        )

        if self._cancel_reason is not None:
            goal_handle.cancel_goal_async()
            self._state = 'CANCELLING'
        else:
            self._state = 'NAVIGATING'

    def _on_feedback(self, feedback_message, token: int) -> None:
        if token != self._active_token or self._state != 'NAVIGATING':
            return
        feedback = feedback_message.feedback
        self._goal_received_feedback = True
        self._last_robot_position = (
            feedback.current_pose.pose.position.x,
            feedback.current_pose.pose.position.y,
        )
        now = self._now()
        if self._enable_motion_spin_guard:
            orientation = feedback.current_pose.pose.orientation
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
            self._motion_guard_status = self._motion_guard.observe(
                now,
                float(feedback.current_pose.pose.position.x),
                float(feedback.current_pose.pose.position.y),
                yaw,
                float(feedback.distance_remaining),
            )
            if self._motion_guard_status.tripped:
                self._motion_guard_triggered = True
                self._blocked_reason = (
                    '转圈保护触发：'
                    f'{self._motion_guard_status.elapsed:.1f} 秒内仅移动 '
                    f'{self._motion_guard_status.translation:.2f} m，'
                    f'航向变化 {math.degrees(self._motion_guard_status.yaw_change):.1f}°，'
                    f'目标进度 {self._motion_guard_status.distance_progress:.2f} m'
                )
                self.get_logger().error(
                    f'{self._blocked_reason}；正在取消导航并进入 BLOCKED'
                )
                self._request_cancel('spin_guard')
                return
        self._observe_health_recovery_progress(
            now,
            float(feedback.distance_remaining),
        )
        if now - self._last_feedback_log < 5.0:
            return
        self._last_feedback_log = now
        waypoint = self._waypoints[self._index]
        self.get_logger().info(
            f'当前任务“{waypoint.name}”剩余约 '
            f'{feedback.distance_remaining:.2f} m'
        )

    def _effective_speed_limit(self, waypoint: Waypoint) -> float:
        # Bounded retries become progressively slower so the controller has
        # more time to regulate curvature and obstacle cost near walls.
        return max(
            0.06,
            waypoint.speed_limit * (
                0.80 ** (self._retry_count + self._health_recovery_count)
            ),
        )

    def _publish_current_speed_limit(self) -> float:
        effective_speed = self._effective_speed_limit(
            self._waypoints[self._index]
        )
        message = Float32()
        message.data = effective_speed
        self._speed_limit_publisher.publish(message)
        if self._nav2_speed_limit_publisher is not None:
            nav2_message = SpeedLimit()
            nav2_message.percentage = False
            nav2_message.speed_limit = effective_speed
            self._nav2_speed_limit_publisher.publish(nav2_message)
        return effective_speed

    def _reset_health_recovery_streak(self) -> None:
        self._health_recovery_count = 0
        self._health_recovery_progress.clear()

    def _observe_health_recovery_progress(
        self,
        now: float,
        distance_remaining: float,
    ) -> None:
        if self._health_recovery_count <= 0:
            return
        qualified = self._health_recovery_progress.observe(
            now,
            distance_remaining,
            self._navigation_health_ready,
        )
        if not qualified:
            return

        safe_distance = self._health_recovery_progress.progress_meters
        safe_seconds = self._health_recovery_progress.elapsed_seconds(now)
        previous_count = self._health_recovery_count
        self._reset_health_recovery_streak()
        restored_speed = self._publish_current_speed_limit()
        self.get_logger().info(
            f'自动恢复后已连续健康行驶 {safe_distance:.2f} m / '
            f'{safe_seconds:.1f} 秒；连续健康恢复记录 '
            f'{previous_count}/{self._max_health_recoveries} 已清零，'
            f'当前路线点限速恢复为 {restored_speed:.2f} m/s'
        )

    def _on_result(self, future, token: int) -> None:
        if token != self._active_token:
            return
        error_code: Optional[int] = None
        error_message: Optional[str] = None
        try:
            wrapped_result = future.result()
            status = wrapped_result.status
            raw_error_code = getattr(wrapped_result.result, 'error_code', None)
            if raw_error_code is not None:
                error_code = int(raw_error_code)
            raw_error_message = getattr(wrapped_result.result, 'error_msg', '')
            if raw_error_message:
                error_message = str(raw_error_message)
        except Exception as error:
            self.get_logger().error(f'读取导航结果失败: {error}')
            status = GoalStatus.STATUS_ABORTED

        cancel_reason = self._cancel_reason
        self._goal_handle = None
        self._active_token = None
        self._cancel_reason = None

        if cancel_reason == 'stop':
            if self._health_recovery_count > 0:
                self._health_recovery_progress.arm()
            self._state = 'PAUSED'
            self.get_logger().info('巡航已停止')
            return
        if cancel_reason == 'reset':
            self._clear_failed_paths('巡航复位')
            self._route_cursor = 0
            self._index = self._route.home_index
            self._retry_count = 0
            self._reset_health_recovery_streak()
            self._completed_loops = 0
            self._returning_home = False
            self._task_ledger.reset()
            self._dwell_deadline = 0.0
            self._blocked_reason = None
            self._last_failure_status = None
            self._last_failure_error_code = None
            self._last_failure_error_message = None
            self._state = 'PAUSED'
            self.get_logger().info('巡航已复位到基地')
            return
        if cancel_reason == 'health':
            self._begin_health_recovery()
            return
        if cancel_reason == 'spin_guard':
            self._retry_count = 0
            self._last_failure_status = GoalStatus.STATUS_CANCELED
            self._last_failure_error_code = None
            self._last_failure_error_message = '转圈保护主动取消导航'
            self._state = 'BLOCKED'
            return
        if cancel_reason == 'estop':
            self._state = 'ESTOP'
            self._retry_count = 0
            self._reset_health_recovery_streak()
            self.get_logger().error('急停已锁定，普通启动命令不能解除')
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._clear_failed_paths('已到达当前路线点')
            self._retry_count = 0
            self._reset_health_recovery_streak()
            self._blocked_reason = None
            self._last_failure_status = None
            self._last_failure_error_code = None
            self._last_failure_error_message = None
            self._complete_current_task()
            return

        waypoint = self._waypoints[self._index]
        error_label = navigation_error_label(error_code)
        error_detail = error_label
        if error_message:
            error_detail = f'{error_label}：{error_message}'
        if cancel_reason == 'timeout':
            self.get_logger().warning(f'巡航点“{waypoint.name}”因超时未到达')
        else:
            self.get_logger().warning(
                f'巡航点“{waypoint.name}”导航失败，状态码={status}，'
                f'Nav2={error_code}（{error_detail}）'
            )
        self._last_failure_error_code = error_code
        self._last_failure_error_message = error_message or error_label
        if cancel_reason != 'timeout' and should_blacklist_route(
            error_code,
            action_aborted=status == GoalStatus.STATUS_ABORTED,
            has_candidate_path=bool(self._active_plan),
        ):
            self._handle_route_failure(
                status=status,
                error_code=error_code,
                error_detail=error_detail,
            )
            return
        self._handle_failure(
            status=status,
            timed_out=cancel_reason == 'timeout',
            failure_detail=(
                None if cancel_reason == 'timeout' else error_detail
            ),
        )

    def _remember_failed_path(self) -> int:
        if not self._active_plan:
            self.get_logger().warning(
                '导航失败但没有捕获到全局路径，无法建立路线黑名单'
            )
            return 0
        robot_position = self._last_robot_position or self._active_plan[0]
        added = self._failed_paths.remember(
            self._active_plan,
            robot_position,
        )
        if not added:
            self.get_logger().warning(
                '导航失败路线过短或已经完全在黑名单中，未新增禁行栅格'
            )
            return 0
        self._publish_failed_path_points()
        self.get_logger().warning(
            f'已记忆第 {self._failed_paths.route_count} 条受阻路线，'
            f'新增 {added} 个临时禁行栅格；'
            '下次将从其余路线中选择最短路'
        )
        return added

    def _publish_failed_path_points(self, publish_empty: bool = False) -> None:
        points = self._failed_paths.points
        if not points and not publish_empty:
            return
        message = PointCloud2()
        message.header = Header()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.height = 1
        message.width = len(points)
        message.fields = [
            PointField(
                name=name,
                offset=offset,
                datatype=PointField.FLOAT32,
                count=1,
            )
            for name, offset in (('x', 0), ('y', 4), ('z', 8))
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = message.point_step * message.width
        message.data = b''.join(
            struct.pack('<fff', x, y, 0.05) for x, y in points
        )
        message.is_dense = True
        self._failed_path_publisher.publish(message)

    def _clear_failed_paths(self, reason: str) -> None:
        had_failed_paths = self._failed_paths.clear()
        self._similar_path_replan_count = 0
        self._last_candidate_similarity = 0.0
        self._route_failure_count = 0
        self._route_replan_pending = False
        if not had_failed_paths:
            return
        # Replace the layer's buffered cloud before resetting its cells, so
        # the previous goal's exclusion cannot be marked again after clearing.
        self._publish_failed_path_points(publish_empty=True)
        global_client = self._costmap_clear_clients[1]
        if global_client.service_is_ready():
            global_client.call_async(ClearEntireCostmap.Request())
        self.get_logger().info(f'{reason}：已清除失败路线临时禁行记录')

    def _clear_global_costmap(self, reason: str) -> None:
        client = self._costmap_clear_clients[1]
        if not client.service_is_ready():
            return
        client.call_async(ClearEntireCostmap.Request())
        self.get_logger().info(f'{reason}：已请求清理全局代价地图')

    def _request_cancel(self, reason: str) -> None:
        if self._state == 'CANCELLING':
            return
        self._cancel_reason = reason
        if self._goal_handle is not None:
            self._state = 'CANCELLING'
            self._goal_handle.cancel_goal_async()
        elif self._state == 'SENDING_GOAL':
            self._state = 'CANCELLING'
        else:
            self._finish_local_cancel(reason)

    def _finish_local_cancel(self, reason: str) -> None:
        was_dwelling = self._state == 'DWELL'
        self._policy_generation += 1
        self._preflight_generation += 1
        self._goal_handle = None
        self._active_token = None
        self._cancel_reason = None
        if reason == 'reset':
            self._clear_failed_paths('巡航复位')
            self._route_cursor = 0
            self._index = self._route.home_index
            self._retry_count = 0
            self._reset_health_recovery_streak()
            self._completed_loops = 0
            self._returning_home = False
            self._task_ledger.reset()
            self._dwell_deadline = 0.0
            self._blocked_reason = None
            self._last_failure_status = None
            self._last_failure_error_code = None
            self._last_failure_error_message = None
        elif reason == 'stop' and was_dwelling:
            # Arrival was already acknowledged and its counter was already
            # decremented, so resume from the following task after a stop.
            self._advance_after_inspection()
        if reason == 'health':
            self._begin_health_recovery()
        elif reason == 'spin_guard':
            self._retry_count = 0
            self._last_failure_status = GoalStatus.STATUS_CANCELED
            self._last_failure_error_code = None
            self._last_failure_error_message = '转圈保护主动取消导航'
            self._state = 'BLOCKED'
        elif reason == 'estop':
            self._retry_count = 0
            self._reset_health_recovery_streak()
            self._state = 'ESTOP'
        else:
            self._state = 'PAUSED'

    def _handle_failure(
        self,
        status: Optional[int] = None,
        timed_out: bool = False,
        failure_detail: Optional[str] = None,
    ) -> None:
        self._goal_handle = None
        self._active_token = None
        self._last_failure_status = status
        self._route_replan_pending = False
        waypoint = self._waypoints[self._index]
        # Clearing maps never moves the base, so this remains safe even for
        # restricted HOME/equipment points where reverse and spin are forbidden.
        self._clear_costmaps('导航失败')
        if self._health_recovery_count > 0:
            self._health_recovery_progress.arm()
        if self._retry_count < self._max_retries:
            self._retry_count += 1
            self.get_logger().warning(
                f'{self._retry_delay:.1f} 秒后进行第 {self._retry_count} 次重试'
            )
            self._state = 'RETRY_WAIT'
            self._state_deadline = self._now() + self._retry_delay
            return

        failure = failure_detail or (
            '导航超时' if timed_out else f'导航失败（状态码 {status}）'
        )
        self._blocked_reason = f'{waypoint.name}：{failure}'
        self._state = 'BLOCKED'
        self.get_logger().error(
            f'路线点“{waypoint.name}”连续失败 {self._max_retries + 1} 次，'
            '已进入 BLOCKED 并保持停车；请检查现场后人工确认继续'
        )

    def _handle_route_failure(
        self,
        status: int,
        error_code: int,
        error_detail: str,
    ) -> None:
        """Exclude an infeasible candidate and request a genuinely new plan."""

        self._goal_handle = None
        self._active_token = None
        self._last_failure_status = status
        self._route_failure_count += 1
        added = self._remember_failed_path()
        waypoint = self._waypoints[self._index]

        if added <= 0 and self._failed_paths.route_count == 0:
            self._route_replan_pending = False
            self._blocked_reason = (
                f'{waypoint.name}：{error_detail}；未能捕获足够长的失败路线，'
                '无法安全建立路线禁用区'
            )
            self._state = 'BLOCKED'
            self.get_logger().error(
                '局部控制器拒绝了路线，但没有可用于禁用的完整路径；'
                '为避免重复同一路线，已停车等待检查'
            )
            return

        if self._route_failure_count >= self._max_route_failures:
            self._route_replan_pending = False
            self._blocked_reason = (
                f'{waypoint.name}：已经连续尝试并禁用 '
                f'{self._route_failure_count} 条候选路线；最后失败为 '
                f'{error_code}（{error_detail}）'
            )
            self._state = 'BLOCKED'
            self.get_logger().error(
                f'路线点“{waypoint.name}”已达到 '
                f'{self._max_route_failures} 次不同路线执行上限，'
                '已进入 BLOCKED；不会退回重复路线'
            )
            return

        # Do not increment _retry_count: low-speed retry is reserved for
        # transient transport/timing failures. The next action starts at the
        # waypoint's configured speed and must use the failed-path layer.
        self._retry_count = 0
        self._route_replan_pending = True
        self._clear_costmaps('局部控制器拒绝当前路线')
        self._state = 'RETRY_WAIT'
        self._state_deadline = self._now() + self._similar_path_replan_delay
        self.get_logger().warning(
            f'Nav2 错误 {error_code}（{error_detail}）：当前完整路线已禁用；'
            f'{self._similar_path_replan_delay:.2f} 秒后规划未使用的最短路线 '
            f'({self._route_failure_count}/{self._max_route_failures})，'
            '不对原路线执行低速重试'
        )

    def _complete_current_task(self) -> None:
        waypoint = self._waypoints[self._index]
        if self._returning_home:
            self._finish_loop()
            return

        if waypoint.waypoint_type == 'HOME':
            self.get_logger().info(f'车辆已位于基地“{waypoint.name}”')
            if len(self._route.ordered_indices) == 1:
                self._finish_loop()
            else:
                self._route_cursor = 1
                self._index = self._route.ordered_indices[self._route_cursor]
                self._state = self._ready_state()
            return

        if waypoint.count_as_task:
            remaining = self._task_ledger.complete_inspection(self._index)
            self.get_logger().info(
                f'巡检任务“{waypoint.name}”已完成，本项目剩余 {remaining} 次'
            )
        else:
            self.get_logger().info(f'已通过路线过渡点“{waypoint.name}”')
        if waypoint.dwell > 0.0:
            self._state = 'DWELL'
            self._dwell_deadline = self._now() + waypoint.dwell
            self.get_logger().info(
                f'在“{waypoint.name}”停留 {waypoint.dwell:.1f} 秒'
            )
        else:
            self._advance_after_inspection()

    def _advance_after_inspection(self) -> None:
        self._dwell_deadline = 0.0
        if self._route_cursor + 1 < len(self._route.ordered_indices):
            self._route_cursor += 1
            self._index = self._route.ordered_indices[self._route_cursor]
            self._state = self._ready_state()
            return
        self._route_cursor = 0
        self._index = self._route.home_index
        self._returning_home = True
        self._state = self._ready_state()
        self.get_logger().info('本圈全部巡检任务完成，正在返回基地')

    def _finish_loop(self) -> None:
        if not self._task_ledger.round_ready(self._completed_loops):
            self._state = 'PAUSED'
            self._returning_home = False
            self.get_logger().error(
                '已到达基地，但本圈仍有未完成巡检任务；拒绝计入完成圈数'
            )
            return

        self._completed_loops += 1
        # A fresh lap must not inherit transient lidar marks from the previous
        # approach to the home point. Static map obstacles are restored by the
        # static layer immediately after this clear request.
        self._clear_costmaps('开始下一圈前')
        self._route_cursor = 0
        self._index = self._route.home_index
        self._returning_home = False
        if self._completed_loops >= self._loop_count:
            self._state = 'COMPLETE'
            self.get_logger().info(
                f'已完成 {self._completed_loops} 圈巡检并返回基地'
            )
            return
        if len(self._route.ordered_indices) > 1:
            self._route_cursor = 1
            self._index = self._route.ordered_indices[self._route_cursor]
        # Give the static layer and a fresh laser scan time to repopulate after
        # clearing; otherwise the next lap may plan against a half-reset map.
        self._state = 'RETRY_WAIT'
        self._state_deadline = self._now() + self._lap_restart_delay
        self.get_logger().info(
            f'第 {self._completed_loops} 圈完成，车辆已在基地；'
            f'{self._lap_restart_delay:.1f} 秒后开始第 '
            f'{self._completed_loops + 1} 圈'
        )

    def _clear_costmaps(self, reason: str) -> None:
        requested = 0
        for client in self._costmap_clear_clients:
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())
                requested += 1
        if requested:
            self.get_logger().info(
                f'{reason}：已请求清理 {requested} 个导航代价地图'
            )

    def _on_start(self, _request, response):
        if self._state not in ('PAUSED', 'BLOCKED', 'COMPLETE'):
            response.success = False
            response.message = f'当前状态为 {self._state}，无需重复启动'
            return response
        was_blocked = self._state == 'BLOCKED'
        if self._state == 'COMPLETE':
            self._route_cursor = 0
            self._index = self._route.home_index
            self._retry_count = 0
            self._reset_health_recovery_streak()
            self._completed_loops = 0
            self._returning_home = False
            self._task_ledger.reset()
        if was_blocked:
            self.get_logger().warning(
                '收到人工确认，重新尝试路线点'
                f'“{self._waypoints[self._index].name}”'
            )
            # Manual confirmation means the physical obstruction has been
            # inspected or removed, so a new recovery cycle may reconsider
            # routes rejected before that confirmation.
            self._clear_failed_paths('人工确认现场后')
            self._clear_costmaps('人工恢复')
        self._retry_count = 0
        self._reset_health_recovery_streak()
        self._blocked_reason = None
        self._last_failure_status = None
        self._last_failure_error_code = None
        self._last_failure_error_message = None
        self._state = self._ready_state()
        response.success = True
        if was_blocked and self._state == 'WAITING_HEALTH':
            response.message = '已重新检查；导航健康恢复后自动重试当前路线点'
        elif was_blocked:
            response.message = '已确认安全，正在清理地图并重试当前路线点'
        else:
            response.message = '巡航已启动'
        return response

    def _on_stop(self, _request, response):
        if self._state in ('PAUSED', 'BLOCKED', 'ESTOP'):
            response.success = True
            response.message = (
                '巡航已经停止'
                if self._state == 'PAUSED'
                else '车辆已停车且保持安全锁定'
            )
            return response
        self._request_cancel('stop')
        response.success = True
        response.message = '正在停止巡航'
        return response

    def _on_reset(self, _request, response):
        if self._state == 'ESTOP':
            response.success = False
            response.message = '急停锁定中，不能通过任务复位解除'
            return response
        self._request_cancel('reset')
        response.success = True
        response.message = '正在复位巡航任务'
        return response

    def _on_estop(self, _request, response):
        self._blocked_reason = '急停已触发'
        if self._state != 'ESTOP':
            self._request_cancel('estop')
        response.success = True
        response.message = '急停已触发并锁定'
        return response

    def _on_clear_estop(self, _request, response):
        if self._state != 'ESTOP':
            response.success = False
            response.message = '当前未处于急停状态'
            return response
        if self._require_navigation_health and not self._navigation_health_ready:
            response.success = False
            response.message = '导航健康检查未通过，拒绝解除急停'
            return response
        self._blocked_reason = None
        self._last_failure_status = None
        self._last_failure_error_code = None
        self._last_failure_error_message = None
        self._state = 'PAUSED'
        response.success = True
        response.message = '急停已解除，巡检仍保持暂停'
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PatrolManager()
        rclpy.spin(node)
    except (FileNotFoundError, ValueError) as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f'patrol_manager: {error}')
        raise
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
