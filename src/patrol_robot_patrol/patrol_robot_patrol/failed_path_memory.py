import math
from collections.abc import Sequence


Point2D = tuple[float, float]


class FailedPathMemory:
    """Remember complete failed routes and build persistent keep-out bands."""

    def __init__(
        self,
        start_clearance: float = 0.80,
        goal_clearance: float = 0.80,
        band_radius: float = 0.18,
        resolution: float = 0.05,
        similarity_distance: float = 0.75,
        similarity_ratio: float = 0.70,
    ) -> None:
        self._start_clearance = max(0.0, start_clearance)
        self._goal_clearance = max(0.0, goal_clearance)
        self._band_radius = max(0.0, band_radius)
        self._resolution = max(0.01, resolution)
        self._similarity_distance = max(resolution, similarity_distance)
        self._similarity_ratio = min(1.0, max(0.0, similarity_ratio))
        self._points: dict[tuple[int, int], Point2D] = {}
        self._routes: list[tuple[Point2D, ...]] = []

    @property
    def points(self) -> tuple[Point2D, ...]:
        return tuple(self._points.values())

    @property
    def route_count(self) -> int:
        return len(self._routes)

    @property
    def similarity_ratio(self) -> float:
        return self._similarity_ratio

    def clear(self) -> bool:
        had_exclusions = bool(self._points or self._routes)
        self._points.clear()
        self._routes.clear()
        return had_exclusions

    def remember(self, path: Sequence[Point2D], robot: Point2D) -> int:
        remaining = _path_from_nearest(path, robot)
        if len(remaining) < 2:
            return 0
        route = tuple(_resample(remaining, 0.10))
        if len(route) < 2:
            return 0

        total_length = _path_length(route)
        exclusion_end = total_length - self._goal_clearance
        if exclusion_end <= self._start_clearance:
            return 0

        before = len(self._points)
        centers = _resample_between(
            route,
            self._start_clearance,
            exclusion_end,
            self._resolution,
        )
        radius_cells = math.ceil(self._band_radius / self._resolution)
        for center_x, center_y in centers:
            center_cell_x = round(center_x / self._resolution)
            center_cell_y = round(center_y / self._resolution)
            for offset_x in range(-radius_cells, radius_cells + 1):
                for offset_y in range(-radius_cells, radius_cells + 1):
                    if math.hypot(offset_x, offset_y) * self._resolution > (
                        self._band_radius + self._resolution * 0.5
                    ):
                        continue
                    key = (center_cell_x + offset_x, center_cell_y + offset_y)
                    self._points[key] = (
                        key[0] * self._resolution,
                        key[1] * self._resolution,
                    )
        added = len(self._points) - before
        if added:
            self._routes.append(route)
        return added

    def similarity(self, candidate: Sequence[Point2D]) -> float:
        if not self._routes or len(candidate) < 2:
            return 0.0
        sampled = _resample(candidate, 0.10)
        interior = _resample_between(
            sampled,
            self._start_clearance,
            max(
                self._start_clearance,
                _path_length(sampled) - self._goal_clearance,
            ),
            0.15,
        )
        if not interior:
            return 0.0

        best = 0.0
        threshold_squared = self._similarity_distance ** 2
        for failed in self._routes:
            overlapping = sum(
                1
                for point in interior
                if any(
                    _distance_squared(point, failed_point) <= threshold_squared
                    for failed_point in failed
                )
            )
            best = max(best, overlapping / len(interior))
        return best

    def is_similar(self, candidate: Sequence[Point2D]) -> tuple[bool, float]:
        similarity = self.similarity(candidate)
        return similarity >= self._similarity_ratio, similarity


def _path_from_nearest(
    path: Sequence[Point2D], robot: Point2D
) -> list[Point2D]:
    if not path:
        return []
    nearest = min(
        range(len(path)),
        key=lambda index: _distance_squared(path[index], robot),
    )
    return list(path[nearest:])


def _distance_squared(first: Point2D, second: Point2D) -> float:
    delta_x = first[0] - second[0]
    delta_y = first[1] - second[1]
    return delta_x * delta_x + delta_y * delta_y


def _path_length(path: Sequence[Point2D]) -> float:
    return sum(math.dist(path[index - 1], path[index]) for index in range(1, len(path)))


def _resample(path: Sequence[Point2D], spacing: float) -> list[Point2D]:
    if not path:
        return []
    return _resample_between(path, 0.0, _path_length(path), spacing)


def _resample_between(
    path: Sequence[Point2D],
    start_distance: float,
    end_distance: float,
    spacing: float,
) -> list[Point2D]:
    if len(path) < 2 or end_distance < start_distance:
        return []
    cumulative = [0.0]
    for index in range(1, len(path)):
        cumulative.append(cumulative[-1] + math.dist(path[index - 1], path[index]))
    end_distance = min(end_distance, cumulative[-1])
    if end_distance < start_distance:
        return []

    result: list[Point2D] = []
    distance = start_distance
    while distance <= end_distance + 1.0e-9:
        result.append(_point_at_distance(path, cumulative, distance))
        distance += max(0.01, spacing)
    if not result or math.dist(
        result[-1], _point_at_distance(path, cumulative, end_distance)
    ) > spacing * 0.5:
        result.append(_point_at_distance(path, cumulative, end_distance))
    return result


def _point_at_distance(
    path: Sequence[Point2D],
    cumulative: Sequence[float],
    target: float,
) -> Point2D:
    for index in range(1, len(cumulative)):
        if cumulative[index] < target:
            continue
        segment_length = cumulative[index] - cumulative[index - 1]
        if segment_length <= 1.0e-9:
            return path[index]
        ratio = (target - cumulative[index - 1]) / segment_length
        start = path[index - 1]
        end = path[index]
        return (
            start[0] + (end[0] - start[0]) * ratio,
            start[1] + (end[1] - start[1]) * ratio,
        )
    return path[-1]
