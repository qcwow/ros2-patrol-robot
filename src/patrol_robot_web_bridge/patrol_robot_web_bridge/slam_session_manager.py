"""Supervise Humble's non-lifecycle asynchronous SLAM Toolbox process."""

import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_srvs.srv import Trigger


class SlamSessionManager(Node):
    """Give Humble SLAM Toolbox deterministic lifecycle and reset semantics.

    The Humble asynchronous SLAM node is an ordinary rclcpp node. Pausing its
    scan input does not stop its map->odom broadcaster and Humble provides no
    service that clears the in-memory pose graph. Supervising the process makes
    deactivation stop TF completely and makes reset start a genuinely fresh
    mapping session.
    """

    def __init__(self) -> None:
        super().__init__('slam_session_manager')
        self.declare_parameter('slam_params_file', '')
        self.declare_parameter('map_topic', '/slam_map')
        # rclpy declares use_sim_time itself when launch supplies an override.
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.declare_parameter('startup_active', True)

        params_file = str(self.get_parameter('slam_params_file').value)
        if not params_file:
            raise RuntimeError('slam_params_file 不能为空')
        self._params_file = Path(params_file).expanduser().resolve()
        if not self._params_file.is_file():
            raise RuntimeError(f'SLAM 参数文件不存在：{self._params_file}')

        self._map_topic = str(self.get_parameter('map_topic').value)
        self._use_sim_time = bool(self.get_parameter('use_sim_time').value)
        prefix = Path(get_package_prefix('slam_toolbox'))
        self._executable = (
            prefix / 'lib' / 'slam_toolbox' / 'async_slam_toolbox_node'
        )
        if not self._executable.is_file():
            raise RuntimeError(f'找不到异步 SLAM 可执行文件：{self._executable}')

        self._lock = threading.RLock()
        self._process: subprocess.Popen | None = None
        self._state_id = State.PRIMARY_STATE_INACTIVE
        self._state_label = 'inactive'

        self.create_service(
            ChangeState,
            '/slam_toolbox/change_state',
            self._on_change_state,
        )
        self.create_service(
            GetState,
            '/slam_toolbox/get_state',
            self._on_get_state,
        )
        self.create_service(
            Trigger,
            '/slam_toolbox/reset_session',
            self._on_reset_session,
        )
        wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self._wall_clock = wall_clock
        self.create_timer(0.5, self._check_process, clock=wall_clock)

        if bool(self.get_parameter('startup_active').value):
            with self._lock:
                self._start_locked()

    def _command(self) -> list[str]:
        return [
            str(self._executable),
            '--ros-args',
            '--params-file',
            str(self._params_file),
            '-p',
            f'use_sim_time:={str(self._use_sim_time).lower()}',
            '-r',
            f'/map:={self._map_topic}',
            '-r',
            f'map:={self._map_topic}',
        ]

    def _start_locked(self) -> None:
        if self._process is not None and self._process.poll() is None:
            self._state_id = State.PRIMARY_STATE_ACTIVE
            self._state_label = 'active'
            return
        self._process = subprocess.Popen(
            self._command(),
            start_new_session=True,
        )
        # Detect parameter and executable errors before reporting activation.
        time.sleep(0.2)
        return_code = self._process.poll()
        if return_code is not None:
            self._process = None
            self._state_id = State.PRIMARY_STATE_INACTIVE
            self._state_label = 'inactive'
            raise RuntimeError(f'SLAM Toolbox 启动后立即退出，状态码={return_code}')
        self._state_id = State.PRIMARY_STATE_ACTIVE
        self._state_label = 'active'
        self.get_logger().info(
            f'已启动新的异步 SLAM 会话，地图输出：{self._map_topic}'
        )

    @staticmethod
    def _signal_process_group(process: subprocess.Popen, signum: int) -> None:
        if process.pid <= 1:
            raise RuntimeError(f'拒绝向异常 SLAM PID 发送信号：{process.pid}')
        os.killpg(os.getpgid(process.pid), signum)

    def _stop_locked(self) -> None:
        process = self._process
        self._process = None
        if process is not None and process.poll() is None:
            self._signal_process_group(process, signal.SIGINT)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self._signal_process_group(process, signal.SIGTERM)
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    self._signal_process_group(process, signal.SIGKILL)
                    process.wait(timeout=2.0)
        self._state_id = State.PRIMARY_STATE_INACTIVE
        self._state_label = 'inactive'
        if self.context.ok():
            self.get_logger().info('异步 SLAM 会话已停止，map→odom 已释放')

    def _restart_locked(self) -> None:
        self._stop_locked()
        self._start_locked()

    def _on_get_state(self, _request, response):
        with self._lock:
            response.current_state.id = self._state_id
            response.current_state.label = self._state_label
        return response

    def _on_change_state(self, request, response):
        transition = int(request.transition.id)
        try:
            with self._lock:
                if transition == Transition.TRANSITION_ACTIVATE:
                    self._start_locked()
                elif transition == Transition.TRANSITION_DEACTIVATE:
                    self._stop_locked()
                else:
                    response.success = False
                    return response
            response.success = True
        except Exception as error:
            self.get_logger().error(f'SLAM 生命周期切换失败：{error}')
            response.success = False
        return response

    def _on_reset_session(self, _request, response):
        try:
            with self._lock:
                if self._state_id == State.PRIMARY_STATE_ACTIVE:
                    self._restart_locked()
            response.success = True
            response.message = '已创建全新的 SLAM 会话'
        except Exception as error:
            response.success = False
            response.message = str(error)
            self.get_logger().error(f'重建 SLAM 会话失败：{error}')
        return response

    def _check_process(self) -> None:
        with self._lock:
            process = self._process
            if process is None:
                return
            return_code = process.poll()
            if return_code is None:
                return
            self._process = None
            self._state_id = State.PRIMARY_STATE_INACTIVE
            self._state_label = 'inactive'
        self.get_logger().error(
            f'异步 SLAM 进程意外退出，状态码={return_code}'
        )

    def close(self) -> None:
        with self._lock:
            self._stop_locked()


def main(args=None):
    rclpy.init(args=args)
    node = SlamSessionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
