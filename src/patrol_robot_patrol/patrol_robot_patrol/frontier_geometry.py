"""Pure occupancy-grid geometry used by autonomous frontier exploration."""

from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FrontierGoal:
    x: float
    y: float
    yaw: float
    cluster_size: int
    distance: float
    score: float


def _neighbors(index, width, height, diagonal=True):
    row, column = divmod(index, width)
    offsets = (
        (-1, -1), (-1, 0), (-1, 1),
        (0, -1), (0, 1),
        (1, -1), (1, 0), (1, 1),
    ) if diagonal else ((-1, 0), (0, -1), (0, 1), (1, 0))
    for row_offset, column_offset in offsets:
        neighbor_row = row + row_offset
        neighbor_column = column + column_offset
        if 0 <= neighbor_row < height and 0 <= neighbor_column < width:
            yield neighbor_row * width + neighbor_column


def extract_frontier_clusters(
    width,
    height,
    data,
    *,
    free_threshold=20,
    min_cluster_size=8,
):
    """Return connected free-cell boundaries adjacent to unknown space."""
    if width <= 0 or height <= 0 or len(data) != width * height:
        return []

    frontiers = {
        index
        for index, value in enumerate(data)
        if 0 <= value <= free_threshold
        and any(data[neighbor] < 0 for neighbor in _neighbors(
            index, width, height
        ))
    }
    clusters = []
    while frontiers:
        seed = frontiers.pop()
        cluster = [seed]
        pending = [seed]
        while pending:
            current = pending.pop()
            for neighbor in _neighbors(current, width, height):
                if neighbor in frontiers:
                    frontiers.remove(neighbor)
                    pending.append(neighbor)
                    cluster.append(neighbor)
        if len(cluster) >= min_cluster_size:
            clusters.append(cluster)
    return clusters


def reachable_free_cells(
    width,
    height,
    data,
    robot_index,
    *,
    free_threshold=20,
):
    """Find the known-free component connected to the robot."""
    if (
        robot_index < 0
        or robot_index >= len(data)
        or not 0 <= data[robot_index] <= free_threshold
    ):
        return set()
    visited = {robot_index}
    pending = deque([robot_index])
    while pending:
        current = pending.popleft()
        for neighbor in _neighbors(
            current, width, height, diagonal=False
        ):
            if (
                neighbor not in visited
                and 0 <= data[neighbor] <= free_threshold
            ):
                visited.add(neighbor)
                pending.append(neighbor)
    return visited


def world_to_cell(x, y, width, height, resolution, origin_x, origin_y):
    if resolution <= 0.0:
        return None
    column = math.floor((x - origin_x) / resolution)
    row = math.floor((y - origin_y) / resolution)
    if not 0 <= row < height or not 0 <= column < width:
        return None
    return row * width + column


def cell_to_world(index, width, resolution, origin_x, origin_y):
    row, column = divmod(index, width)
    return (
        origin_x + (column + 0.5) * resolution,
        origin_y + (row + 0.5) * resolution,
    )


def _has_occupied_clearance(
    index,
    width,
    height,
    data,
    clearance_cells,
    occupied_threshold,
):
    row, column = divmod(index, width)
    radius_squared = clearance_cells * clearance_cells
    for row_offset in range(-clearance_cells, clearance_cells + 1):
        for column_offset in range(-clearance_cells, clearance_cells + 1):
            if row_offset * row_offset + column_offset * column_offset > radius_squared:
                continue
            check_row = row + row_offset
            check_column = column + column_offset
            if not 0 <= check_row < height or not 0 <= check_column < width:
                continue
            if data[check_row * width + check_column] >= occupied_threshold:
                return False
    return True


def _nearest_safe_cell(
    target_index,
    width,
    height,
    data,
    reachable,
    *,
    search_cells,
    clearance_cells,
    free_threshold,
    occupied_threshold,
):
    target_row, target_column = divmod(target_index, width)
    candidates = []
    for row_offset in range(-search_cells, search_cells + 1):
        for column_offset in range(-search_cells, search_cells + 1):
            row = target_row + row_offset
            column = target_column + column_offset
            if not 0 <= row < height or not 0 <= column < width:
                continue
            distance_squared = (
                row_offset * row_offset + column_offset * column_offset
            )
            if distance_squared <= search_cells * search_cells:
                candidates.append((distance_squared, row * width + column))
    candidates.sort()
    for _, index in candidates:
        if index not in reachable or not 0 <= data[index] <= free_threshold:
            continue
        if _has_occupied_clearance(
            index,
            width,
            height,
            data,
            clearance_cells,
            occupied_threshold,
        ):
            return index
    return None


def select_frontier_goal(
    width,
    height,
    resolution,
    origin_x,
    origin_y,
    data,
    robot_x,
    robot_y,
    clusters,
    *,
    free_threshold=20,
    occupied_threshold=65,
    goal_offset=0.45,
    goal_search_radius=0.80,
    robot_clearance=0.34,
    min_goal_distance=0.65,
    max_goal_distance=10.0,
    information_gain_weight=1.0,
    distance_weight=0.35,
    blacklisted_points=(),
    blacklist_radius=0.80,
):
    """Select a reachable, collision-cleared goal behind the best frontier."""
    robot_index = world_to_cell(
        robot_x,
        robot_y,
        width,
        height,
        resolution,
        origin_x,
        origin_y,
    )
    if robot_index is None:
        return None
    reachable = reachable_free_cells(
        width,
        height,
        data,
        robot_index,
        free_threshold=free_threshold,
    )
    if not reachable:
        return None

    search_cells = max(1, math.ceil(goal_search_radius / resolution))
    clearance_cells = max(1, math.ceil(robot_clearance / resolution))
    best = None
    for cluster in clusters:
        reachable_frontier = [index for index in cluster if index in reachable]
        if not reachable_frontier:
            continue
        frontier_points = [
            cell_to_world(index, width, resolution, origin_x, origin_y)
            for index in reachable_frontier
        ]
        centroid_x = sum(point[0] for point in frontier_points) / len(frontier_points)
        centroid_y = sum(point[1] for point in frontier_points) / len(frontier_points)
        # A newly observed area often creates one ring-shaped frontier whose
        # centroid lies on the robot. Obstacles can also pull part of that ring
        # very close to the base. Select a real frontier cell that is far
        # enough for the offset goal to satisfy min_goal_distance; otherwise
        # the centroid-nearest cell creates a goal at the robot and the entire
        # (otherwise reachable) cluster is incorrectly rejected.
        eligible_frontier_points = [
            point
            for point in frontier_points
            if math.hypot(point[0] - robot_x, point[1] - robot_y)
            >= goal_offset + min_goal_distance
        ]
        if not eligible_frontier_points:
            continue
        frontier_x, frontier_y = min(
            eligible_frontier_points,
            key=lambda point: (
                (point[0] - centroid_x) ** 2 + (point[1] - centroid_y) ** 2
            ),
        )
        delta_x = frontier_x - robot_x
        delta_y = frontier_y - robot_y
        frontier_distance = math.hypot(delta_x, delta_y)
        if frontier_distance <= 1e-6:
            continue

        target_x = frontier_x - goal_offset * delta_x / frontier_distance
        target_y = frontier_y - goal_offset * delta_y / frontier_distance
        target_index = world_to_cell(
            target_x,
            target_y,
            width,
            height,
            resolution,
            origin_x,
            origin_y,
        )
        if target_index is None:
            continue
        safe_index = _nearest_safe_cell(
            target_index,
            width,
            height,
            data,
            reachable,
            search_cells=search_cells,
            clearance_cells=clearance_cells,
            free_threshold=free_threshold,
            occupied_threshold=occupied_threshold,
        )
        if safe_index is None:
            continue
        goal_x, goal_y = cell_to_world(
            safe_index, width, resolution, origin_x, origin_y
        )
        goal_distance = math.hypot(goal_x - robot_x, goal_y - robot_y)
        if not min_goal_distance <= goal_distance <= max_goal_distance:
            continue
        if any(
            math.hypot(goal_x - point[0], goal_y - point[1]) < blacklist_radius
            for point in blacklisted_points
        ):
            continue

        score = (
            information_gain_weight * len(reachable_frontier) * resolution
            - distance_weight * goal_distance
        )
        candidate = FrontierGoal(
            x=goal_x,
            y=goal_y,
            yaw=math.atan2(frontier_y - goal_y, frontier_x - goal_x),
            cluster_size=len(reachable_frontier),
            distance=goal_distance,
            score=score,
        )
        if best is None or candidate.score > best.score:
            best = candidate
    return best
