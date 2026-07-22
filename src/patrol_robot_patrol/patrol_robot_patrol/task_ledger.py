"""Per-waypoint completion accounting for finite patrol rounds."""

from __future__ import annotations


class PatrolTaskLedger:
    """Track visits only for semantic inspection tasks."""

    def __init__(
        self,
        waypoint_count: int,
        loop_count: int,
        task_indices: tuple[int, ...] | list[int] | None = None,
    ) -> None:
        if waypoint_count < 1:
            raise ValueError('waypoint_count 必须大于等于 1')
        if loop_count < 1:
            raise ValueError('loop_count 必须大于等于 1')
        self.waypoint_count = waypoint_count
        self.loop_count = loop_count
        # Keep the old constructor behavior for third-party callers while the
        # patrol manager supplies explicit semantic inspection indexes.
        indexes = range(1, waypoint_count) if task_indices is None else task_indices
        self.task_indices = tuple(dict.fromkeys(int(index) for index in indexes))
        if any(index < 0 or index >= waypoint_count for index in self.task_indices):
            raise ValueError('task_indices 包含越界路线点')
        self.remaining: list[int] = []
        self.reset()

    def reset(self) -> None:
        self.remaining = [0 for _ in range(self.waypoint_count)]
        for index in self.task_indices:
            self.remaining[index] = self.loop_count

    def complete_inspection(self, index: int) -> int:
        if index not in self.task_indices:
            raise IndexError('只有 INSPECTION 任务可以扣减次数')
        if self.remaining[index] <= 0:
            raise ValueError('巡检任务已全部完成，不能重复扣减')
        self.remaining[index] -= 1
        return self.remaining[index]

    def round_ready(self, completed_loops: int) -> bool:
        """Return true only when every task was completed in the current lap."""
        expected = self.loop_count - (completed_loops + 1)
        return expected >= 0 and all(
            self.remaining[index] == expected for index in self.task_indices
        )

    def completed_visits(self, index: int) -> int:
        if index not in self.task_indices:
            return 0
        return self.loop_count - self.remaining[index]
