"""Pure helpers for classifying navigation rotation events."""

from __future__ import annotations

import math
from typing import Iterable, Mapping

from patrol_robot_patrol.navigation_motion_guard import (
    shortest_angular_distance,
)


def compose_planar_transforms(
    parent_to_middle: Mapping[str, object],
    middle_to_child: Mapping[str, object],
) -> dict[str, float]:
    """Compose two x/y/yaw transforms without depending on ROS or TF."""
    first_yaw = float(parent_to_middle['yaw'])
    second_x = float(middle_to_child['x'])
    second_y = float(middle_to_child['y'])
    cosine = math.cos(first_yaw)
    sine = math.sin(first_yaw)
    return {
        'x': (
            float(parent_to_middle['x'])
            + cosine * second_x
            - sine * second_y
        ),
        'y': (
            float(parent_to_middle['y'])
            + sine * second_x
            + cosine * second_y
        ),
        'yaw': first_yaw + float(middle_to_child['yaw']),
    }


def accumulated_yaw(samples: Iterable[float]) -> float:
    """Return signed accumulated yaw while handling the +/-pi boundary."""
    values = [float(value) for value in samples]
    return sum(
        shortest_angular_distance(previous, current)
        for previous, current in zip(values, values[1:])
    )


def integrate_series(
    samples: Iterable[tuple[float, float]],
) -> float:
    """Integrate a timestamped scalar series with the trapezoid rule."""
    values = [(float(stamp), float(value)) for stamp, value in samples]
    total = 0.0
    for (first_time, first_value), (second_time, second_value) in zip(
        values, values[1:]
    ):
        delta = second_time - first_time
        if 0.0 < delta <= 1.0:
            total += delta * (first_value + second_value) / 2.0
    return total


def _yaw_delta(
    samples: list[Mapping[str, object]], key: str
) -> float | None:
    yaws = []
    for sample in samples:
        pose = sample.get(key)
        if isinstance(pose, Mapping) and pose.get('yaw') is not None:
            yaws.append(float(pose['yaw']))
    return accumulated_yaw(yaws) if len(yaws) >= 2 else None


def _integrated_value(
    samples: list[Mapping[str, object]],
    container_key: str,
    value_key: str,
) -> float | None:
    series = []
    for sample in samples:
        container = sample.get(container_key)
        if not isinstance(container, Mapping):
            continue
        value = container.get(value_key)
        elapsed = sample.get('elapsed_seconds')
        if value is not None and elapsed is not None:
            series.append((float(elapsed), float(value)))
    return integrate_series(series) if len(series) >= 2 else None


def _nested_stamp(sample: Mapping[str, object], path: tuple[str, ...]):
    value: object = sample
    for key in path:
        if not isinstance(value, Mapping):
            return None
        value = value.get(key)
    return int(value) if value is not None else None


def _stamp_pair_stats(
    samples: list[Mapping[str, object]],
    first_path: tuple[str, ...],
    second_path: tuple[str, ...],
) -> dict[str, float | int | None]:
    deltas = []
    for sample in samples:
        first = _nested_stamp(sample, first_path)
        second = _nested_stamp(sample, second_path)
        if first and second:
            deltas.append(abs(first - second) / 1_000_000.0)
    if not deltas:
        return {'sample_count': 0, 'maximum_absolute_ms': None}
    return {
        'sample_count': len(deltas),
        'maximum_absolute_ms': round(max(deltas), 3),
    }


def _command_stage_peaks(
    samples: list[Mapping[str, object]],
) -> dict[str, dict[str, float | int | None]]:
    result = {}
    for stage in (
        'cmd_vel_nav_raw',
        'cmd_vel_nav',
        'cmd_vel_base_raw',
        'cmd_vel_safety_checked',
        'controller_cmd_vel',
    ):
        linear_values = []
        angular_values = []
        for sample in samples:
            commands = sample.get('commands')
            if not isinstance(commands, Mapping):
                continue
            linear = commands.get(f'{stage}_linear_x')
            angular = commands.get(f'{stage}_angular_z')
            if linear is not None:
                linear_values.append(abs(float(linear)))
            if angular is not None:
                angular_values.append(abs(float(angular)))
        result[stage] = {
            'sample_count': min(len(linear_values), len(angular_values)),
            'maximum_absolute_linear_x': (
                round(max(linear_values), 4) if linear_values else None
            ),
            'maximum_absolute_angular_z': (
                round(max(angular_values), 4) if angular_values else None
            ),
        }
    return result


def summarize_rotation_event(
    samples: Iterable[Mapping[str, object]],
    meaningful_yaw_degrees: float = 10.0,
) -> dict[str, object]:
    """Separate physical yaw from global-localization correction."""
    values = list(samples)
    odom_delta = _yaw_delta(values, 'odom_to_base')
    correction_delta = _yaw_delta(values, 'map_to_odom')
    global_delta = _yaw_delta(values, 'map_to_base')
    commanded_delta = _integrated_value(
        values, 'commands', 'cmd_vel_safety_checked_angular_z'
    )
    imu_delta = _integrated_value(values, 'imu', 'angular_velocity_z')

    threshold = math.radians(max(0.0, float(meaningful_yaw_degrees)))
    physical = abs(odom_delta) if odom_delta is not None else None
    correction = (
        abs(correction_delta) if correction_delta is not None else None
    )
    if physical is None or correction is None:
        classification = 'insufficient_tf_data'
    elif physical >= threshold and correction < physical * 0.5:
        classification = 'physical_chassis_rotation'
    elif correction >= threshold and physical < correction * 0.5:
        classification = 'localization_correction'
    elif physical >= threshold or correction >= threshold:
        classification = 'mixed_physical_and_localization_rotation'
    else:
        classification = 'no_meaningful_rotation_in_recorded_window'

    def degrees(value: float | None) -> float | None:
        return None if value is None else round(math.degrees(value), 3)

    return {
        'classification': classification,
        'sample_count': len(values),
        'odom_to_base_yaw_degrees': degrees(odom_delta),
        'map_to_odom_yaw_degrees': degrees(correction_delta),
        'map_to_base_yaw_degrees': degrees(global_delta),
        'safety_checked_command_integral_degrees': degrees(commanded_delta),
        'imu_yaw_integral_degrees': degrees(imu_delta),
        'command_stage_peaks': _command_stage_peaks(values),
        'timestamp_alignment': {
            'rgb_to_depth': _stamp_pair_stats(
                values,
                ('sensors', 'rgb', 'stamp_ns'),
                ('sensors', 'depth', 'stamp_ns'),
            ),
            'imu_to_raw_odom': _stamp_pair_stats(
                values,
                ('imu', 'stamp_ns'),
                ('sensors', 'odom_raw', 'stamp_ns'),
            ),
            'rgb_to_imu': _stamp_pair_stats(
                values,
                ('sensors', 'rgb', 'stamp_ns'),
                ('imu', 'stamp_ns'),
            ),
            'depth_to_imu': _stamp_pair_stats(
                values,
                ('sensors', 'depth', 'stamp_ns'),
                ('imu', 'stamp_ns'),
            ),
        },
    }
