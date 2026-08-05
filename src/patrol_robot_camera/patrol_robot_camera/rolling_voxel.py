"""Bounded rolling voxel storage for a robot-centred local map."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class _Voxel:
    position: np.ndarray
    color: np.ndarray
    observations: int
    last_seen: float


class RollingVoxelMap:
    """Keep a fixed-size, observation-filtered voxel map near the robot."""

    def __init__(
        self,
        *,
        voxel_size: float,
        half_extent: float,
        max_voxels: int,
        min_observations: int,
        stale_seconds: float,
    ) -> None:
        if voxel_size <= 0.0:
            raise ValueError('voxel_size must be greater than zero')
        if half_extent <= voxel_size:
            raise ValueError('half_extent must be greater than voxel_size')
        if max_voxels <= 0:
            raise ValueError('max_voxels must be greater than zero')
        if min_observations <= 0:
            raise ValueError('min_observations must be greater than zero')
        if stale_seconds <= 0.0:
            raise ValueError('stale_seconds must be greater than zero')

        self.voxel_size = float(voxel_size)
        self.half_extent = float(half_extent)
        self.max_voxels = int(max_voxels)
        self.min_observations = int(min_observations)
        self.stale_seconds = float(stale_seconds)
        self._voxels: dict[tuple[int, int, int], _Voxel] = {}
        self._origin = np.zeros(3, dtype=np.float32)

    @property
    def candidate_voxel_count(self) -> int:
        return len(self._voxels)

    def clear(self) -> None:
        self._voxels.clear()

    def insert(
        self,
        points: np.ndarray,
        colors: np.ndarray,
        origin: np.ndarray,
        stamp: float,
    ) -> None:
        points = np.asarray(points, dtype=np.float32)
        colors = np.asarray(colors, dtype=np.uint8)
        origin = np.asarray(origin, dtype=np.float32).reshape(3)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError('points must be an Nx3 array')
        if colors.shape != points.shape:
            raise ValueError('colors must have the same Nx3 shape as points')

        self._origin = origin
        if points.size == 0:
            self._prune(stamp)
            return

        finite = np.isfinite(points).all(axis=1)
        inside = (np.abs(points - origin) <= self.half_extent).all(axis=1)
        valid = finite & inside
        points = points[valid]
        colors = colors[valid]
        if points.size == 0:
            self._prune(stamp)
            return

        keys = np.floor(points / self.voxel_size).astype(np.int32)
        # One update per input voxel prevents dense images from inflating the
        # confidence merely because many neighbouring pixels hit one cell.
        _, indices = np.unique(keys, axis=0, return_index=True)
        for index in indices:
            key = tuple(int(value) for value in keys[index])
            point = points[index]
            color = colors[index]
            voxel = self._voxels.get(key)
            if voxel is None:
                self._voxels[key] = _Voxel(
                    position=point.copy(),
                    color=color.copy(),
                    observations=1,
                    last_seen=float(stamp),
                )
                continue

            observations = min(voxel.observations + 1, 255)
            # A short exponential average suppresses depth shimmer without
            # leaving a long trail behind moving objects.
            voxel.position = 0.5 * voxel.position + 0.5 * point
            voxel.color = np.clip(
                0.5 * voxel.color.astype(np.float32)
                + 0.5 * color.astype(np.float32),
                0,
                255,
            ).astype(np.uint8)
            voxel.observations = observations
            voxel.last_seen = float(stamp)

        self._prune(stamp)

    def _prune(self, stamp: float) -> None:
        stale_before = float(stamp) - self.stale_seconds
        remove = [
            key
            for key, voxel in self._voxels.items()
            if voxel.last_seen < stale_before
            or np.any(np.abs(voxel.position - self._origin) > self.half_extent)
        ]
        for key in remove:
            self._voxels.pop(key, None)

        excess = len(self._voxels) - self.max_voxels
        if excess <= 0:
            return
        oldest_and_farthest = sorted(
            self._voxels.items(),
            key=lambda item: (
                item[1].last_seen,
                -float(np.sum((item[1].position - self._origin) ** 2)),
            ),
        )
        for key, _ in oldest_and_farthest[:excess]:
            self._voxels.pop(key, None)

    def snapshot(self) -> tuple[np.ndarray, np.ndarray]:
        visible = [
            voxel
            for voxel in self._voxels.values()
            if voxel.observations >= self.min_observations
        ]
        if not visible:
            return (
                np.empty((0, 3), dtype=np.float32),
                np.empty((0, 3), dtype=np.uint8),
            )
        points = np.ascontiguousarray(
            np.stack([voxel.position for voxel in visible]),
            dtype=np.float32,
        )
        colors = np.ascontiguousarray(
            np.stack([voxel.color for voxel in visible]),
            dtype=np.uint8,
        )
        return points, colors
