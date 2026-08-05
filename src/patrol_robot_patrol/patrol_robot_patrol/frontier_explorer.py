"""Autonomous frontier exploration for live SLAM maps."""

import json
import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .frontier_geometry import (
    cell_to_world,
    extract_frontier_clusters,
    select_frontier_goal,
)
from .navigation_motion_guard import NavigationMotionGuard


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.declare_parameter('autostart', True)
        self.declare_parameter('start_delay_seconds', 12.0)
        self.declare_parameter('planning_period_seconds', 2.0)
        self.declare_parameter('goal_timeout_seconds', 90.0)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('stop_command_topic', '/cmd_vel')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('behavior_tree', '')
        self.declare_parameter('free_threshold', 20)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('min_frontier_size', 8)
        self.declare_parameter('goal_offset', 0.45)
        self.declare_parameter('goal_search_radius', 0.80)
        self.declare_parameter('robot_clearance', 0.34)
        self.declare_parameter('min_goal_distance', 0.65)
        self.declare_parameter('max_goal_distance', 10.0)
        self.declare_parameter('information_gain_weight', 1.0)
        self.declare_parameter('distance_weight', 0.35)
        self.declare_parameter('blacklist_radius', 0.80)
        self.declare_parameter('blacklist_timeout_seconds', 120.0)
        self.declare_parameter('no_frontier_cycles', 5)
        self.declare_parameter('max_unreachable_cycles', 8)
        self.declare_parameter('require_navigation_health', False)
        self.declare_parameter('enable_motion_spin_guard', False)
        self.declare_parameter('motion_spin_window_seconds', 2.0)
        self.declare_parameter('motion_spin_max_translation', 0.08)
        self.declare_parameter('motion_spin_min_yaw_degrees', 20.0)
        self.declare_parameter('motion_spin_min_goal_progress', 0.02)

        self._global_frame = str(self.get_parameter('global_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._behavior_tree = str(self.get_parameter('behavior_tree').value)
        self._require_navigation_health = bool(
            self.get_parameter('require_navigation_health').value
        )
        self._navigation_health_ready = not self._require_navigation_health
        self._navigation_health_reason = None
        self._navigation_health_checks = {}
        self._global_pose_stable_seconds = 0.0
        self._global_pose_translation_delta = 0.0
        self._global_pose_yaw_delta_degrees = 0.0
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
        self._motion_guard_triggered = False
        self._enabled = bool(self.get_parameter('autostart').value)
        self._startup_deadline = (
            time.monotonic()
            + float(self.get_parameter('start_delay_seconds').value)
        )
        self._last_plan_time = 0.0
        self._goal_started_at = None
        self._map = None
        self._goal_handle = None
        self._pending_goal = False
        self._goal_request_id = 0
        self._current_goal = None
        self._blacklist = []
        self._frontier_count = 0
        self._empty_frontier_cycles = 0
        self._invalid_frontier_cycles = 0
        self._goals_reached = 0
        self._goals_failed = 0
        self._distance_remaining = None
        self._state = 'STARTING' if self._enabled else 'IDLE'
        self._detail = '等待 SLAM、TF 和 Nav2 就绪'

        self._tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._navigator = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter('action_name').value),
        )

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('map_topic').value),
            self._on_map,
            map_qos,
        )
        self.create_subscription(
            Bool,
            '/navigation_health/ready',
            self._on_navigation_health,
            map_qos,
        )
        self.create_subscription(
            String,
            '/navigation_health/status',
            self._on_navigation_health_status,
            map_qos,
        )
        self._frontiers_publisher = self.create_publisher(
            PoseArray, '/frontier_explorer/frontiers', 10
        )
        self._goal_publisher = self.create_publisher(
            PoseStamped, '/frontier_explorer/goal', 10
        )
        self._status_publisher = self.create_publisher(
            String, '/frontier_explorer/status', 10
        )
        self._stop_publisher = self.create_publisher(
            Twist,
            str(self.get_parameter('stop_command_topic').value),
            10,
        )
        self.create_service(
            Trigger, '/frontier_explorer/start', self._on_start
        )
        self.create_service(
            Trigger, '/frontier_explorer/stop', self._on_stop
        )
        self.create_service(
            Trigger, '/frontier_explorer/reset', self._on_reset
        )
        self.create_timer(0.5, self._tick)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            'Frontier Explorer 已启动：根据 /map 选择未知边界，并通过 Nav2 自主探索'
        )

    def _on_map(self, message):
        self._map = message

    def _on_navigation_health(self, message):
        ready = bool(message.data)
        self._navigation_health_ready = ready
        if (
            self._require_navigation_health
            and not ready
            and (self._goal_handle is not None or self._pending_goal)
        ):
            self._cancel_goal(
                (
                    self._navigation_health_reason
                    or '导航健康失效，已取消前沿目标并停车'
                ),
                blacklist=False,
            )
            self._state = 'WAITING_FOR_HEALTH'

    def _on_navigation_health_status(self, message):
        try:
            status = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        reason = status.get('reason')
        self._navigation_health_reason = (
            str(reason) if reason else None
        )
        checks = status.get('checks')
        self._navigation_health_checks = (
            checks if isinstance(checks, dict) else {}
        )
        self._global_pose_stable_seconds = float(
            status.get('global_pose_stable_seconds') or 0.0
        )
        self._global_pose_translation_delta = float(
            status.get('global_pose_translation_delta') or 0.0
        )
        self._global_pose_yaw_delta_degrees = float(
            status.get('global_pose_yaw_delta_degrees') or 0.0
        )

    def _on_start(self, _request, response):
        self._enabled = True
        self._startup_deadline = time.monotonic()
        self._empty_frontier_cycles = 0
        self._invalid_frontier_cycles = 0
        self._state = 'WAITING'
        self._detail = '等待下一轮前沿选择'
        response.success = True
        response.message = '自主探索已启动'
        return response

    def _on_stop(self, _request, response):
        self._enabled = False
        self._cancel_goal('用户停止自主探索', blacklist=False)
        self._state = 'STOPPED'
        self._detail = '已停车并停止选择新前沿'
        response.success = True
        response.message = '自主探索已停止'
        return response

    def _on_reset(self, _request, response):
        self._cancel_goal('重置自主探索', blacklist=False)
        self._blacklist.clear()
        self._empty_frontier_cycles = 0
        self._invalid_frontier_cycles = 0
        self._goals_reached = 0
        self._goals_failed = 0
        self._enabled = True
        self._startup_deadline = time.monotonic()
        self._state = 'WAITING'
        self._detail = '历史失败点已清除'
        response.success = True
        response.message = '自主探索已重置'
        return response

    def _robot_pose(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
            return (
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
            )
        except TransformException:
            return None

    def _tick(self):
        now = time.monotonic()
        self._expire_blacklist(now)
        if not self._enabled:
            return
        if now < self._startup_deadline:
            return
        if (
            self._require_navigation_health
            and not self._navigation_health_ready
        ):
            self._state = 'WAITING_FOR_HEALTH'
            self._detail = (
                self._navigation_health_reason
                or '等待 RTAB-Map 位姿和导航状态持续稳定'
            )
            return
        if self._goal_handle is not None or self._pending_goal:
            if (
                self._goal_started_at is not None
                and now - self._goal_started_at
                > float(self.get_parameter('goal_timeout_seconds').value)
            ):
                self._goals_failed += 1
                self._cancel_goal('前沿目标执行超时', blacklist=True)
            return
        if now - self._last_plan_time < float(
            self.get_parameter('planning_period_seconds').value
        ):
            return
        self._last_plan_time = now
        self._plan_next_goal()

    def _plan_next_goal(self):
        if self._map is None:
            self._state = 'WAITING_FOR_MAP'
            self._detail = '尚未收到 SLAM 栅格地图'
            return
        if not self._navigator.server_is_ready():
            self._state = 'WAITING_FOR_NAV2'
            self._detail = 'NavigateToPose 尚未激活'
            return
        robot_pose = self._robot_pose()
        if robot_pose is None:
            self._state = 'WAITING_FOR_TF'
            self._detail = 'map 到 base_footprint 变换尚未就绪'
            return

        info = self._map.info
        data = tuple(self._map.data)
        clusters = extract_frontier_clusters(
            info.width,
            info.height,
            data,
            free_threshold=int(self.get_parameter('free_threshold').value),
            min_cluster_size=int(
                self.get_parameter('min_frontier_size').value
            ),
        )
        self._frontier_count = len(clusters)
        self._publish_frontiers(clusters)
        if not clusters:
            self._empty_frontier_cycles += 1
            self._invalid_frontier_cycles = 0
            required = int(self.get_parameter('no_frontier_cycles').value)
            if self._empty_frontier_cycles >= required:
                self._enabled = False
                self._state = 'COMPLETED'
                self._detail = '连续多轮未发现未知边界，探索完成'
                self._publish_zero()
                self.get_logger().info('自主探索完成：地图中没有剩余有效前沿')
            else:
                self._state = 'VERIFYING_COMPLETE'
                self._detail = (
                    f'暂未发现前沿，正在复核 '
                    f'{self._empty_frontier_cycles}/{required}'
                )
            return

        self._empty_frontier_cycles = 0
        origin = info.origin.position
        goal = select_frontier_goal(
            info.width,
            info.height,
            info.resolution,
            origin.x,
            origin.y,
            data,
            robot_pose[0],
            robot_pose[1],
            clusters,
            free_threshold=int(self.get_parameter('free_threshold').value),
            occupied_threshold=int(
                self.get_parameter('occupied_threshold').value
            ),
            goal_offset=float(self.get_parameter('goal_offset').value),
            goal_search_radius=float(
                self.get_parameter('goal_search_radius').value
            ),
            robot_clearance=float(
                self.get_parameter('robot_clearance').value
            ),
            min_goal_distance=float(
                self.get_parameter('min_goal_distance').value
            ),
            max_goal_distance=float(
                self.get_parameter('max_goal_distance').value
            ),
            information_gain_weight=float(
                self.get_parameter('information_gain_weight').value
            ),
            distance_weight=float(
                self.get_parameter('distance_weight').value
            ),
            blacklisted_points=[
                (entry['x'], entry['y']) for entry in self._blacklist
            ],
            blacklist_radius=float(
                self.get_parameter('blacklist_radius').value
            ),
        )
        if goal is None:
            self._invalid_frontier_cycles += 1
            required = int(
                self.get_parameter('max_unreachable_cycles').value
            )
            if self._invalid_frontier_cycles >= required:
                self._enabled = False
                self._state = 'COMPLETED_WITH_UNREACHABLE'
                self._detail = (
                    f'探索已安全结束，保留 {len(clusters)} 个不可达前沿'
                )
                self._publish_zero()
                self.get_logger().warning(
                    '自主探索安全结束：剩余前沿均不满足可达性或安全间距'
                )
                return
            self._state = 'NO_REACHABLE_FRONTIER'
            self._detail = (
                f'发现 {len(clusters)} 个前沿，但暂无安全可达目标；'
                f'复核 {self._invalid_frontier_cycles}/{required}'
            )
            return

        self._invalid_frontier_cycles = 0
        self._send_goal(goal)

    def _publish_frontiers(self, clusters):
        message = PoseArray()
        message.header.frame_id = self._global_frame
        message.header.stamp = self.get_clock().now().to_msg()
        info = self._map.info
        for cluster in clusters:
            points = [
                cell_to_world(
                    index,
                    info.width,
                    info.resolution,
                    info.origin.position.x,
                    info.origin.position.y,
                )
                for index in cluster
            ]
            pose = PoseStamped().pose
            pose.position.x = sum(point[0] for point in points) / len(points)
            pose.position.y = sum(point[1] for point in points) / len(points)
            pose.orientation.w = 1.0
            message.poses.append(pose)
        self._frontiers_publisher.publish(message)

    def _send_goal(self, goal):
        pose = PoseStamped()
        pose.header.frame_id = self._global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = goal.x
        pose.pose.position.y = goal.y
        pose.pose.orientation.z = math.sin(goal.yaw * 0.5)
        pose.pose.orientation.w = math.cos(goal.yaw * 0.5)
        self._goal_publisher.publish(pose)

        action_goal = NavigateToPose.Goal()
        action_goal.pose = pose
        if self._behavior_tree:
            action_goal.behavior_tree = self._behavior_tree
        self._current_goal = goal
        self._pending_goal = True
        self._goal_request_id += 1
        request_id = self._goal_request_id
        self._goal_started_at = time.monotonic()
        self._motion_guard.reset()
        self._motion_guard_triggered = False
        self._state = 'SENDING_GOAL'
        self._detail = (
            f'前沿 {goal.cluster_size} 格，距离 {goal.distance:.2f} m'
        )
        future = self._navigator.send_goal_async(
            action_goal,
            feedback_callback=self._on_feedback,
        )
        future.add_done_callback(
            lambda completed, goal_id=request_id:
                self._on_goal_response(completed, goal_id)
        )

    def _on_goal_response(self, future, request_id):
        try:
            goal_handle = future.result()
        except Exception as exception:
            if request_id != self._goal_request_id:
                return
            self._pending_goal = False
            self._goals_failed += 1
            self._blacklist_current_goal()
            self._clear_active_goal()
            self._state = 'GOAL_ERROR'
            self._detail = str(exception)
            return
        if request_id != self._goal_request_id or not self._enabled:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        self._pending_goal = False
        if not goal_handle.accepted:
            self._goals_failed += 1
            self._blacklist_current_goal()
            self._clear_active_goal()
            self._state = 'GOAL_REJECTED'
            self._detail = 'Nav2 拒绝了前沿目标'
            return
        self._goal_handle = goal_handle
        self._state = 'NAVIGATING'
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed, goal_id=request_id:
                self._on_goal_result(completed, goal_id)
        )

    def _on_feedback(self, feedback_message):
        feedback = feedback_message.feedback
        self._distance_remaining = float(feedback.distance_remaining)
        if not self._enable_motion_spin_guard:
            return
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
        status = self._motion_guard.observe(
            time.monotonic(),
            float(feedback.current_pose.pose.position.x),
            float(feedback.current_pose.pose.position.y),
            yaw,
            self._distance_remaining,
        )
        if not status.tripped:
            return
        self._motion_guard_triggered = True
        self._goals_failed += 1
        detail = (
            f'转圈保护触发：{status.elapsed:.1f} 秒内仅移动 '
            f'{status.translation:.2f} m，航向变化 '
            f'{math.degrees(status.yaw_change):.1f}°，'
            f'目标进度 {status.distance_progress:.2f} m'
        )
        self._cancel_goal(detail, blacklist=False)
        self._enabled = False
        self._state = 'BLOCKED'
        self.get_logger().error(f'{detail}；自主探索已锁定并停车')

    def _on_goal_result(self, future, request_id):
        if request_id != self._goal_request_id:
            return
        try:
            wrapped_result = future.result()
            status = wrapped_result.status
        except Exception as exception:
            status = GoalStatus.STATUS_ABORTED
            self._detail = str(exception)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._goals_reached += 1
            self._state = 'GOAL_REACHED'
            self._detail = '已到达前沿，等待新地图后继续探索'
        elif not self._enabled and status == GoalStatus.STATUS_CANCELED:
            self._state = 'STOPPED'
        else:
            self._goals_failed += 1
            self._blacklist_current_goal()
            self._state = 'GOAL_FAILED'
            self._detail = f'前沿目标失败，Nav2 状态码 {status}'
        self._clear_active_goal()
        self._last_plan_time = 0.0

    def _cancel_goal(self, reason, *, blacklist):
        if blacklist:
            self._blacklist_current_goal()
        self._goal_request_id += 1
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._clear_active_goal()
        self._publish_zero()
        self._detail = reason

    def _blacklist_current_goal(self):
        if self._current_goal is None:
            return
        self._blacklist.append({
            'x': self._current_goal.x,
            'y': self._current_goal.y,
            'expires_at': (
                time.monotonic()
                + float(
                    self.get_parameter(
                        'blacklist_timeout_seconds'
                    ).value
                )
            ),
        })

    def _expire_blacklist(self, now):
        self._blacklist = [
            entry for entry in self._blacklist
            if entry['expires_at'] > now
        ]

    def _clear_active_goal(self):
        self._goal_handle = None
        self._pending_goal = False
        self._goal_started_at = None
        self._current_goal = None
        self._distance_remaining = None

    def _publish_zero(self):
        self._stop_publisher.publish(Twist())

    def _publish_status(self):
        goal = self._current_goal
        message = String()
        message.data = json.dumps({
            'enabled': self._enabled,
            'state': self._state,
            'detail': self._detail,
            'frontier_clusters': self._frontier_count,
            'goals_reached': self._goals_reached,
            'goals_failed': self._goals_failed,
            'blacklisted_goals': len(self._blacklist),
            'distance_remaining': (
                round(self._distance_remaining, 3)
                if self._distance_remaining is not None
                else None
            ),
            'navigation_health_ready': self._navigation_health_ready,
            'navigation_health_reason': self._navigation_health_reason,
            'navigation_health_checks': self._navigation_health_checks,
            'global_pose_stable_seconds': round(
                self._global_pose_stable_seconds, 2
            ),
            'global_pose_translation_delta': round(
                self._global_pose_translation_delta, 4
            ),
            'global_pose_yaw_delta_degrees': round(
                self._global_pose_yaw_delta_degrees, 3
            ),
            'motion_spin_guard_enabled': self._enable_motion_spin_guard,
            'motion_spin_guard_triggered': self._motion_guard_triggered,
            'goal': (
                {
                    'x': round(goal.x, 3),
                    'y': round(goal.y, 3),
                    'score': round(goal.score, 3),
                    'cluster_size': goal.cluster_size,
                }
                if goal is not None
                else None
            ),
        }, ensure_ascii=False)
        self._status_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
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
