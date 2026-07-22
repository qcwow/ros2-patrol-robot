"""Pure geometry helpers used by the navigation health gate."""

from __future__ import annotations

import math
from collections.abc import Sequence


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw for a normalized or near-normalized quaternion."""
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def rectangle_overlaps_lethal_cell(
    data: Sequence[int],
    *,
    grid_width: int,
    grid_height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    footprint_length: float,
    footprint_width: float,
    safety_margin: float = 0.0,
    lethal_cost_threshold: int = 100,
) -> bool:
    """Check an oriented rectangular body against lethal OccupancyGrid cells.

    Nav2 publishes inscribed/inflated cells as value 99 and lethal obstacles as
    100. Only the latter represents a physical overlap for this safety gate.
    Unknown cells (-1) remain a planning concern but are not a collision.
    """
    if (
        grid_width <= 0
        or grid_height <= 0
        or resolution <= 0.0
        or len(data) < grid_width * grid_height
    ):
        raise ValueError('invalid occupancy grid')

    half_length = footprint_length * 0.5 + max(0.0, safety_margin)
    half_width = footprint_width * 0.5 + max(0.0, safety_margin)
    cosine = math.cos(robot_yaw)
    sine = math.sin(robot_yaw)

    # Include cells whose square touches the body, not only cells whose centre
    # is inside it. This is conservative by at most half a map cell.
    cell_padding = resolution * math.sqrt(0.5)
    search_radius = math.hypot(half_length, half_width) + cell_padding
    min_column = math.floor((robot_x - search_radius - origin_x) / resolution)
    max_column = math.floor((robot_x + search_radius - origin_x) / resolution)
    min_row = math.floor((robot_y - search_radius - origin_y) / resolution)
    max_row = math.floor((robot_y + search_radius - origin_y) / resolution)

    # Leaving the known costmap is unsafe. Check the oriented corners rather
    # than the circular search bounds, which would over-report near map edges.
    map_max_x = origin_x + grid_width * resolution
    map_max_y = origin_y + grid_height * resolution
    for local_x in (-half_length, half_length):
        for local_y in (-half_width, half_width):
            corner_x = robot_x + cosine * local_x - sine * local_y
            corner_y = robot_y + sine * local_x + cosine * local_y
            if (
                corner_x < origin_x
                or corner_y < origin_y
                or corner_x >= map_max_x
                or corner_y >= map_max_y
            ):
                return True

    min_column = max(0, min_column)
    min_row = max(0, min_row)
    max_column = min(grid_width - 1, max_column)
    max_row = min(grid_height - 1, max_row)

    expanded_length = half_length + cell_padding
    expanded_width = half_width + cell_padding
    for row in range(min_row, max_row + 1):
        for column in range(min_column, max_column + 1):
            if int(data[row * grid_width + column]) < lethal_cost_threshold:
                continue
            delta_x = origin_x + (column + 0.5) * resolution - robot_x
            delta_y = origin_y + (row + 0.5) * resolution - robot_y
            local_x = cosine * delta_x + sine * delta_y
            local_y = -sine * delta_x + cosine * delta_y
            if (
                abs(local_x) <= expanded_length
                and abs(local_y) <= expanded_width
            ):
                return True
    return False
