"""ROS-independent progress gate for resetting consecutive health recoveries."""

from __future__ import annotations

import math


class HealthRecoveryProgress:
    """Require healthy forward progress before clearing a recovery streak."""

    def __init__(
        self,
        required_stable_seconds: float,
        required_progress_meters: float,
    ) -> None:
        self.required_stable_seconds = max(0.0, float(required_stable_seconds))
        self.required_progress_meters = max(0.0, float(required_progress_meters))
        self.armed = False
        self._started_at: float | None = None
        self._initial_distance: float | None = None
        self._best_distance: float | None = None

    def arm(self) -> None:
        """Start a fresh probation period after an automatic recovery."""
        self.armed = True
        self._reset_observation()

    def clear(self) -> None:
        self.armed = False
        self._reset_observation()

    def interrupt(self) -> None:
        """Discard partial proof while retaining the pending probation."""
        if self.armed:
            self._reset_observation()

    def _reset_observation(self) -> None:
        self._started_at = None
        self._initial_distance = None
        self._best_distance = None

    def observe(
        self,
        now: float,
        distance_remaining: float,
        health_ready: bool,
    ) -> bool:
        """Return true only after both healthy time and goal progress qualify."""
        if not self.armed:
            return False
        if not health_ready:
            self.interrupt()
            return False
        if not math.isfinite(now) or not math.isfinite(distance_remaining):
            return False

        distance = max(0.0, float(distance_remaining))
        if self._started_at is None:
            self._started_at = float(now)
            self._initial_distance = distance
            self._best_distance = distance
            return False

        self._best_distance = min(
            distance,
            self._best_distance if self._best_distance is not None else distance,
        )
        return bool(
            self.elapsed_seconds(now) >= self.required_stable_seconds
            and self.progress_meters >= self.required_progress_meters
        )

    def elapsed_seconds(self, now: float) -> float:
        if self._started_at is None or not math.isfinite(now):
            return 0.0
        return max(0.0, float(now) - self._started_at)

    @property
    def progress_meters(self) -> float:
        if self._initial_distance is None or self._best_distance is None:
            return 0.0
        return max(0.0, self._initial_distance - self._best_distance)
