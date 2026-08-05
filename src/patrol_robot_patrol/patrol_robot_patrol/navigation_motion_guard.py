"""Pure navigation pose guards shared by ROS-facing safety nodes."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math


def shortest_angular_distance(start: float, end: float) -> float:
    """Return the signed shortest rotation from ``start`` to ``end``."""
    return math.atan2(math.sin(end - start), math.cos(end - start))


@dataclass(frozen=True)
class PoseStabilityStatus:
    ready: bool
    stable_for: float
    translation_delta: float
    yaw_delta: float


class PoseStabilityGate:
    """Require a transform to remain near an anchor for a fixed duration."""

    def __init__(
        self,
        stable_seconds: float,
        max_translation_delta: float,
        max_yaw_delta: float,
    ) -> None:
        self.stable_seconds = max(0.0, float(stable_seconds))
        self.max_translation_delta = max(
            0.0, float(max_translation_delta)
        )
        self.max_yaw_delta = max(0.0, float(max_yaw_delta))
        self.reset()

    def reset(self) -> None:
        self._anchor: tuple[float, float, float, float] | None = None

    def observe(
        self, now: float, x: float, y: float, yaw: float
    ) -> PoseStabilityStatus:
        sample = (float(now), float(x), float(y), float(yaw))
        if self._anchor is None:
            self._anchor = sample
            return PoseStabilityStatus(False, 0.0, 0.0, 0.0)

        anchor_time, anchor_x, anchor_y, anchor_yaw = self._anchor
        translation = math.hypot(x - anchor_x, y - anchor_y)
        yaw_delta = abs(shortest_angular_distance(anchor_yaw, yaw))
        if (
            translation > self.max_translation_delta
            or yaw_delta > self.max_yaw_delta
            or now < anchor_time
        ):
            self._anchor = sample
            return PoseStabilityStatus(False, 0.0, translation, yaw_delta)

        stable_for = max(0.0, now - anchor_time)
        return PoseStabilityStatus(
            stable_for >= self.stable_seconds,
            stable_for,
            translation,
            yaw_delta,
        )


@dataclass(frozen=True)
class MotionGuardStatus:
    tripped: bool
    elapsed: float
    translation: float
    yaw_change: float
    distance_progress: float


class NavigationMotionGuard:
    """Detect turning without useful translation or goal progress."""

    def __init__(
        self,
        window_seconds: float,
        max_translation: float,
        min_yaw_change: float,
        min_distance_progress: float,
    ) -> None:
        self.window_seconds = max(0.1, float(window_seconds))
        self.max_translation = max(0.0, float(max_translation))
        self.min_yaw_change = max(0.0, float(min_yaw_change))
        self.min_distance_progress = max(
            0.0, float(min_distance_progress)
        )
        self.reset()

    def reset(self) -> None:
        self._samples: deque[
            tuple[float, float, float, float, float]
        ] = deque()
        self._unwrapped_yaw: float | None = None
        self._last_yaw: float | None = None

    def observe(
        self,
        now: float,
        x: float,
        y: float,
        yaw: float,
        distance_remaining: float,
    ) -> MotionGuardStatus:
        now = float(now)
        yaw = float(yaw)
        if self._last_yaw is None:
            self._unwrapped_yaw = yaw
        else:
            self._unwrapped_yaw += shortest_angular_distance(
                self._last_yaw, yaw
            )
        self._last_yaw = yaw
        self._samples.append((
            now,
            float(x),
            float(y),
            float(self._unwrapped_yaw),
            float(distance_remaining),
        ))

        cutoff = now - self.window_seconds
        # Retain the last sample at or before the window boundary so the
        # measured interval does not shrink below the configured duration.
        while (
            len(self._samples) >= 2
            and self._samples[1][0] <= cutoff
        ):
            self._samples.popleft()

        first = self._samples[0]
        elapsed = max(0.0, now - first[0])
        translation = math.hypot(x - first[1], y - first[2])
        yaw_change = abs(float(self._unwrapped_yaw) - first[3])
        distance_progress = first[4] - float(distance_remaining)
        tripped = bool(
            elapsed >= self.window_seconds
            and translation <= self.max_translation
            and yaw_change >= self.min_yaw_change
            and distance_progress < self.min_distance_progress
        )
        return MotionGuardStatus(
            tripped,
            elapsed,
            translation,
            yaw_change,
            distance_progress,
        )
