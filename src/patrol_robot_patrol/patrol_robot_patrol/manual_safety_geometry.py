"""Pure geometry used by the physical-car lidar safety filter."""

from dataclasses import dataclass
import math
from typing import Iterable, Optional


@dataclass(frozen=True)
class SafetyDecision:
    """Result of evaluating one laser scan against a velocity command."""

    linear_scale: float
    rotation_allowed: bool
    nearest_translation_clearance: Optional[float]
    nearest_rotation_range: Optional[float]
    required_stop_clearance: float


def _rectangle_support(direction_x, direction_y, front_extent,
                       rear_extent, half_width):
    longitudinal = front_extent if direction_x >= 0.0 else rear_extent
    return longitudinal * abs(direction_x) + half_width * abs(direction_y)


def evaluate_scan_safety(
        ranges: Iterable[float], angle_min: float, angle_increment: float,
        linear_x: float, linear_y: float, angular_z: float, *,
        front_extent: float = 0.16, rear_extent: float = 0.15,
        half_width: float = 0.13, lidar_offset_x: float = 0.0115,
        corridor_margin: float = 0.02, hard_clearance: float = 0.08,
        slowdown_clearance: float = 0.35, reaction_time: float = 0.35,
        deceleration: float = 0.35, rotation_clearance: float = 0.29,
        minimum_valid_range: float = 0.05,
        maximum_valid_range: float = math.inf) -> SafetyDecision:
    """Calculate a safe scale using a swept rectangular chassis footprint."""
    speed = math.hypot(linear_x, linear_y)
    translation_requested = speed > 1e-6
    rotation_requested = abs(angular_z) > 1e-6

    if translation_requested:
        direction_x = linear_x / speed
        direction_y = linear_y / speed
        normal_x = -direction_y
        normal_y = direction_x
        direction_support = _rectangle_support(
            direction_x, direction_y, front_extent, rear_extent, half_width)
        normal_support = _rectangle_support(
            normal_x, normal_y, front_extent, rear_extent, half_width)
    else:
        direction_x = direction_y = 0.0
        normal_x = normal_y = 0.0
        direction_support = normal_support = 0.0

    nearest_translation = None
    nearest_rotation = None
    angle = angle_min
    for measured_range in ranges:
        if (math.isfinite(measured_range)
                and minimum_valid_range <= measured_range <= maximum_valid_range):
            point_x = lidar_offset_x + measured_range * math.cos(angle)
            point_y = measured_range * math.sin(angle)

            if translation_requested:
                along = point_x * direction_x + point_y * direction_y
                across = abs(point_x * normal_x + point_y * normal_y)
                if along > 0.0 and across <= normal_support + corridor_margin:
                    clearance = along - direction_support
                    if (nearest_translation is None
                            or clearance < nearest_translation):
                        nearest_translation = clearance

            if rotation_requested:
                distance_from_base = math.hypot(point_x, point_y)
                if (nearest_rotation is None
                        or distance_from_base < nearest_rotation):
                    nearest_rotation = distance_from_base
        angle += angle_increment

    safe_deceleration = max(deceleration, 1e-3)
    required_stop_clearance = (
        hard_clearance
        + speed * max(reaction_time, 0.0)
        + speed * speed / (2.0 * safe_deceleration)
    )
    if nearest_translation is None:
        linear_scale = 1.0
    elif nearest_translation <= required_stop_clearance:
        linear_scale = 0.0
    else:
        full_speed_clearance = max(
            slowdown_clearance, required_stop_clearance + 1e-3)
        linear_scale = min(
            1.0,
            (nearest_translation - required_stop_clearance)
            / (full_speed_clearance - required_stop_clearance),
        )

    rotation_allowed = (
        not rotation_requested
        or nearest_rotation is None
        or nearest_rotation > rotation_clearance
    )
    return SafetyDecision(
        linear_scale=linear_scale,
        rotation_allowed=rotation_allowed,
        nearest_translation_clearance=nearest_translation,
        nearest_rotation_range=nearest_rotation,
        required_stop_clearance=required_stop_clearance,
    )
