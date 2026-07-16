import math
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String


@dataclass(frozen=True)
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float
    dwell: float


def load_waypoints(path: str, default_dwell: float) -> tuple[str, list[Waypoint]]:
    waypoint_path = Path(path).expanduser()
    if not waypoint_path.is_file():
        raise FileNotFoundError(f'巡航点文件不存在: {waypoint_path}')

    with waypoint_path.open('r', encoding='utf-8') as stream:
        document = yaml.safe_load(stream) or {}

    frame_id = str(document.get('frame_id', 'map'))
    raw_waypoints = document.get('waypoints')
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError('巡航点文件必须包含非空的 waypoints 列表')

    waypoints = []
    for index, item in enumerate(raw_waypoints):
        if not isinstance(item, dict):
            raise ValueError(f'第 {index + 1} 个巡航点不是对象')
        try:
            waypoint = Waypoint(
                name=str(item.get('name', f'waypoint_{index + 1}')),
                x=float(item['x']),
                y=float(item['y']),
                yaw=float(item.get('yaw', 0.0)),
                dwell=max(0.0, float(item.get('dwell', default_dwell))),
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f'第 {index + 1} 个巡航点格式错误: {error}') from error
        waypoints.append(waypoint)

    return frame_id, waypoints


class PatrolManager(Node):
    """Navigate a complete patrol route with retry and timeout control."""

    def __init__(self) -> None:
        super().__init__('patrol_manager')

        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('autostart', True)
        self.declare_parameter('loop', True)
        self.declare_parameter('loop_count', 1)
        self.declare_parameter('default_dwell_seconds', 2.0)
        self.declare_parameter('start_delay_seconds', 8.0)
        self.declare_parameter('goal_timeout_seconds', 120.0)
        self.declare_parameter('max_retries', 1)
        self.declare_parameter('retry_delay_seconds', 3.0)
        self.declare_parameter('stop_on_failure', False)
        self.declare_parameter('action_name', 'navigate_through_poses')

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
        self._stop_on_failure = self.get_parameter('stop_on_failure').value

        waypoint_file = str(self.get_parameter('waypoint_file').value)
        if not waypoint_file:
            raise ValueError('参数 waypoint_file 不能为空')
        self._frame_id, self._waypoints = load_waypoints(
            waypoint_file, self._default_dwell
        )

        action_name = str(self.get_parameter('action_name').value)
        self._navigation = ActionClient(self, NavigateThroughPoses, action_name)
        self.create_service(Trigger, '~/start', self._on_start)
        self.create_service(Trigger, '~/stop', self._on_stop)
        self.create_service(Trigger, '~/reset', self._on_reset)
        self.create_subscription(String, '~/set_waypoints', self._on_set_waypoints, 10)
        self._status_publisher = self.create_publisher(String, '~/status', 10)

        self._index = 0
        self._retry_count = 0
        self._state = 'START_DELAY' if self._autostart else 'PAUSED'
        self._state_deadline = self._now() + self._start_delay
        self._goal_started_at = 0.0
        self._goal_handle = None
        self._goal_token = 0
        self._active_token: Optional[int] = None
        self._cancel_reason: Optional[str] = None
        self._last_feedback_log = 0.0
        self._completed_loops = 0
        self._returning_home = False
        self._active_route: list[tuple[int, Waypoint, bool]] = []
        self._active_route_position = -1

        self.create_timer(0.25, self._tick)
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info(
            f'已加载 {len(self._waypoints)} 个巡航点，坐标系={self._frame_id}，'
            f'自动启动={self._autostart}，循环={self._loop}'
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _on_set_waypoints(self, message: String) -> None:
        """Replace the active route from a JSON message without restarting."""
        try:
            document = json.loads(message.data)
            raw_waypoints = document.get('waypoints', [])
            if not isinstance(raw_waypoints, list) or not raw_waypoints:
                raise ValueError('waypoints 必须为非空数组')
            waypoints = []
            for index, item in enumerate(raw_waypoints):
                waypoints.append(Waypoint(
                    name=str(item.get('name', f'waypoint_{index + 1}')),
                    x=float(item['x']),
                    y=float(item['y']),
                    yaw=float(item.get('yaw', 0.0)),
                    dwell=max(0.0, float(item.get('dwell', self._default_dwell))),
                ))
            self._request_cancel('reset')
            self._frame_id = str(document.get('frame_id', 'map'))
            self._waypoints = waypoints
            self._loop_count = max(1, min(int(document.get('loop_count', 1)), 1000))
            self._index = 0
            self._retry_count = 0
            self._completed_loops = 0
            self._returning_home = False
            self._state = 'PAUSED'
            self.get_logger().info(
                f'网页已更新巡检路线，共 {len(waypoints)} 个点，'
                f'计划 {self._loop_count} 圈'
            )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().error(f'网页巡检点配置无效: {error}')

    def _publish_status(self) -> None:
        message = String()
        current = self._waypoints[self._index] if self._waypoints else None
        message.data = json.dumps({
            'state': self._state,
            'running': self._state not in ('PAUSED', 'COMPLETE'),
            'current_index': self._index,
            'current_waypoint': current.name if current else None,
            'waypoint_count': len(self._waypoints),
            'loop_count': self._loop_count,
            'completed_loops': self._completed_loops,
            'returning_home': self._returning_home,
        }, ensure_ascii=False)
        self._status_publisher.publish(message)

    def _tick(self) -> None:
        now = self._now()

        if self._state in ('START_DELAY', 'RETRY_WAIT'):
            if now >= self._state_deadline:
                self._state = 'WAITING_SERVER'

        if self._state == 'WAITING_SERVER':
            if self._navigation.wait_for_server(timeout_sec=0.0):
                self._send_current_goal()
            return

        if self._state == 'NAVIGATING':
            route_timeout = self._goal_timeout * max(1, len(self._active_route))
            if now - self._goal_started_at >= route_timeout:
                waypoint = self._waypoints[self._index]
                self.get_logger().warning(
                    f'巡航点“{waypoint.name}”导航超时，正在取消目标'
                )
                self._request_cancel('timeout')

    def _build_route(self) -> list[tuple[int, Waypoint, bool]]:
        if len(self._waypoints) == 1:
            return [(0, self._waypoints[0], False)]
        if self._returning_home:
            return [(0, self._waypoints[0], True)]
        start = min(max(self._index, 0), len(self._waypoints) - 1)
        route = [
            (index, self._waypoints[index], False)
            for index in range(start, len(self._waypoints))
        ]
        # Every route finishes at point 1. On the initial run point 1 is also
        # the first target, ensuring a map switch always starts from home.
        route.append((0, self._waypoints[0], True))
        return route

    def _send_current_goal(self) -> None:
        self._active_route = self._build_route()
        self._active_route_position = -1
        poses = []
        stamp = self.get_clock().now().to_msg()
        for _index, waypoint, _returning_home in self._active_route:
            pose = PoseStamped()
            pose.header.frame_id = self._frame_id
            pose.header.stamp = stamp
            pose.pose.position.x = waypoint.x
            pose.pose.position.y = waypoint.y
            pose.pose.orientation.z = math.sin(waypoint.yaw / 2.0)
            pose.pose.orientation.w = math.cos(waypoint.yaw / 2.0)
            poses.append(pose)

        goal = NavigateThroughPoses.Goal()
        goal.poses = poses

        self._goal_token += 1
        token = self._goal_token
        self._active_token = token
        self._goal_handle = None
        self._cancel_reason = None
        self._goal_started_at = self._now()
        self._state = 'SENDING_GOAL'

        route_names = ' → '.join(target.name for _, target, _ in self._active_route)
        self.get_logger().info(f'开始连续路线规划：{route_names}')
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
        remaining = max(
            0,
            min(int(feedback.number_of_poses_remaining), len(self._active_route)),
        )
        route_position = min(
            max(len(self._active_route) - remaining, 0),
            len(self._active_route) - 1,
        )
        if self._active_route and route_position != self._active_route_position:
            self._active_route_position = route_position
            self._index, target, self._returning_home = self._active_route[route_position]
            if self._returning_home:
                self.get_logger().info('本圈路线已通过所有巡检点，正在返回出发点')
            else:
                self.get_logger().info(
                    f'连续路线当前目标：{self._index + 1}/{len(self._waypoints)} '
                    f'“{target.name}”'
                )
        now = self._now()
        if now - self._last_feedback_log < 5.0:
            return
        self._last_feedback_log = now
        self.get_logger().info(
            f'整段路线剩余约 {feedback.distance_remaining:.2f} m，'
            f'剩余 {remaining} 个目标'
        )

    def _on_result(self, future, token: int) -> None:
        if token != self._active_token:
            return
        try:
            wrapped_result = future.result()
            status = wrapped_result.status
        except Exception as error:
            self.get_logger().error(f'读取导航结果失败: {error}')
            status = GoalStatus.STATUS_ABORTED

        cancel_reason = self._cancel_reason
        self._goal_handle = None
        self._active_token = None
        self._cancel_reason = None
        self._active_route = []

        if cancel_reason == 'stop':
            self._state = 'PAUSED'
            self.get_logger().info('巡航已停止')
            return
        if cancel_reason == 'reset':
            self._index = 0
            self._retry_count = 0
            self._completed_loops = 0
            self._returning_home = False
            self._state = 'PAUSED'
            self.get_logger().info('巡航已复位到第一个巡航点')
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._retry_count = 0
            self._complete_route()
            return

        waypoint = self._waypoints[self._index]
        if cancel_reason == 'timeout':
            self.get_logger().warning(f'巡航点“{waypoint.name}”因超时未到达')
        else:
            self.get_logger().warning(
                f'巡航点“{waypoint.name}”导航失败，状态码={status}'
            )
        self._handle_failure()

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
        self._goal_handle = None
        self._active_token = None
        self._active_route = []
        self._cancel_reason = None
        if reason == 'reset':
            self._index = 0
            self._retry_count = 0
            self._completed_loops = 0
            self._returning_home = False
        self._state = 'PAUSED'

    def _handle_failure(self) -> None:
        self._goal_handle = None
        self._active_token = None
        self._active_route = []
        if self._retry_count < self._max_retries:
            self._retry_count += 1
            self.get_logger().warning(
                f'{self._retry_delay:.1f} 秒后进行第 {self._retry_count} 次重试'
            )
            self._state = 'RETRY_WAIT'
            self._state_deadline = self._now() + self._retry_delay
            return

        self._retry_count = 0
        if self._stop_on_failure:
            self._state = 'PAUSED'
            self.get_logger().error('连续导航失败，已按配置停止巡航')
        else:
            self.get_logger().warning('跳过当前巡航点')
            self._advance_waypoint(arrived=False)

    def _complete_route(self) -> None:
        self._completed_loops += 1
        self._index = 0
        self._returning_home = False
        if self._completed_loops >= self._loop_count:
            self._state = 'COMPLETE'
            self.get_logger().info(
                f'已完成 {self._completed_loops} 圈连续巡检并返回出发点'
            )
            return
        self._index = 1 if len(self._waypoints) > 1 else 0
        self._state = 'WAITING_SERVER'
        self.get_logger().info(
            f'第 {self._completed_loops} 圈完成，开始第 '
            f'{self._completed_loops + 1} 圈连续巡检'
        )

    def _advance_waypoint(self, arrived: bool) -> None:
        if self._returning_home:
            if not arrived:
                self._state = 'PAUSED'
                self.get_logger().error('返回出发点失败，巡检已暂停，未计入完成圈数')
                return
            self._completed_loops += 1
            self._returning_home = False
            if self._completed_loops >= self._loop_count:
                self._state = 'COMPLETE'
                self.get_logger().info(
                    f'已完成 {self._completed_loops} 圈巡检并返回出发点'
                )
                return
            self._index = 1 if len(self._waypoints) > 1 else 0
            self._state = 'WAITING_SERVER'
            self.get_logger().info(
                f'第 {self._completed_loops} 圈完成，开始第 '
                f'{self._completed_loops + 1} 圈'
            )
            return

        if len(self._waypoints) == 1:
            self._completed_loops += 1
            if self._completed_loops >= self._loop_count:
                self._state = 'COMPLETE'
                self.get_logger().info(
                    f'已完成 {self._completed_loops} 圈巡检并停在出发点'
                )
            else:
                self._state = 'WAITING_SERVER'
            return

        if self._index + 1 < len(self._waypoints):
            self._index += 1
            self._state = 'WAITING_SERVER'
            return

        # A patrol lap is only complete after physically returning to point 1.
        self._index = 0
        self._returning_home = True
        self._state = 'WAITING_SERVER'
        self.get_logger().info('末巡检点已完成，正在返回第一个巡检点')

    def _on_start(self, _request, response):
        if self._state not in ('PAUSED', 'COMPLETE'):
            response.success = False
            response.message = f'当前状态为 {self._state}，无需重复启动'
            return response
        if self._state == 'COMPLETE':
            self._index = 0
            self._retry_count = 0
            self._completed_loops = 0
            self._returning_home = False
        self._state = 'WAITING_SERVER'
        response.success = True
        response.message = '巡航已启动'
        return response

    def _on_stop(self, _request, response):
        if self._state == 'PAUSED':
            response.success = True
            response.message = '巡航已经停止'
            return response
        self._request_cancel('stop')
        response.success = True
        response.message = '正在停止巡航'
        return response

    def _on_reset(self, _request, response):
        self._request_cancel('reset')
        response.success = True
        response.message = '正在复位巡航任务'
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
