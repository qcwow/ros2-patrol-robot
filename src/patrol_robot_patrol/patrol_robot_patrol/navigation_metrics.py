"""ROS-independent aggregation for navigation regression results."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any


@dataclass
class NavigationRunMetrics:
    scenario: str
    started_at: float
    path_length: float = 0.0
    minimum_obstacle_distance: float = math.inf
    maximum_retry_count: int = 0
    state_history: list[str] = field(default_factory=list)
    _last_pose: tuple[float, float] | None = None

    def update_pose(self, x: float, y: float) -> None:
        if not math.isfinite(x) or not math.isfinite(y):
            return
        pose = (x, y)
        if self._last_pose is not None:
            step = math.hypot(x - self._last_pose[0], y - self._last_pose[1])
            if step <= 2.0:
                self.path_length += step
        self._last_pose = pose

    def update_scan(self, ranges: list[float] | tuple[float, ...]) -> None:
        finite = [distance for distance in ranges if math.isfinite(distance)]
        if finite:
            self.minimum_obstacle_distance = min(
                self.minimum_obstacle_distance,
                min(finite),
            )

    def update_status(self, status: dict[str, Any]) -> None:
        state = str(status.get('state', 'UNKNOWN'))
        if not self.state_history or self.state_history[-1] != state:
            self.state_history.append(state)
        self.maximum_retry_count = max(
            self.maximum_retry_count,
            int(status.get('retry_count', 0)),
        )

    def summary(
        self,
        finished_at: float,
        status: dict[str, Any],
        actual_pose: tuple[float, float, float] | None,
    ) -> dict[str, Any]:
        goal = status.get('current_goal') or {}
        position_error = None
        yaw_error = None
        if actual_pose is not None and goal:
            position_error = math.hypot(
                actual_pose[0] - float(goal['x']),
                actual_pose[1] - float(goal['y']),
            )
            yaw_delta = actual_pose[2] - float(goal['yaw'])
            yaw_error = abs(math.atan2(math.sin(yaw_delta), math.cos(yaw_delta)))
        return {
            'scenario': self.scenario,
            'result': str(status.get('state', 'UNKNOWN')),
            'success': status.get('state') == 'COMPLETE',
            'elapsed_seconds': max(0.0, finished_at - self.started_at),
            'path_length_meters': round(self.path_length, 3),
            'minimum_obstacle_distance_meters': (
                None if not math.isfinite(self.minimum_obstacle_distance)
                else round(self.minimum_obstacle_distance, 3)
            ),
            'final_position_error_meters': (
                None if position_error is None else round(position_error, 4)
            ),
            'final_yaw_error_radians': (
                None if yaw_error is None else round(yaw_error, 4)
            ),
            'maximum_retry_count': self.maximum_retry_count,
            'blocked_reason': status.get('blocked_reason'),
            'last_failure_status': status.get('last_failure_status'),
            'completed_loops': status.get('completed_loops', 0),
            'state_history': self.state_history,
        }
