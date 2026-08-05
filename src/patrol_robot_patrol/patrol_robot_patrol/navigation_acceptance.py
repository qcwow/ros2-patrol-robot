"""Pure configuration and result evaluation for navigation acceptance."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping


@dataclass(frozen=True)
class NavigationAcceptanceLimits:
    """Safety and quality limits for one real-car route run."""

    expected_terminal_state: str = 'COMPLETE'
    readiness_stable_seconds: float = 5.0
    timeout_seconds: float = 600.0
    maximum_linear_speed: float = 0.08
    maximum_angular_speed: float = 0.20
    maximum_retry_count: int = 1
    maximum_health_false_events: int = 0
    maximum_map_correction_translation: float = 0.08
    maximum_map_correction_yaw_degrees: float = 5.0
    minimum_completed_inspections: int = 0

    @classmethod
    def from_mapping(
        cls, values: Mapping[str, Any]
    ) -> 'NavigationAcceptanceLimits':
        limits = cls(
            expected_terminal_state=str(
                values.get('expected_terminal_state', 'COMPLETE')
            ).upper(),
            readiness_stable_seconds=float(
                values.get('readiness_stable_seconds', 5.0)
            ),
            timeout_seconds=float(values.get('timeout_seconds', 600.0)),
            maximum_linear_speed=float(
                values.get('maximum_linear_speed', 0.08)
            ),
            maximum_angular_speed=float(
                values.get('maximum_angular_speed', 0.20)
            ),
            maximum_retry_count=int(
                values.get('maximum_retry_count', 1)
            ),
            maximum_health_false_events=int(
                values.get('maximum_health_false_events', 0)
            ),
            maximum_map_correction_translation=float(
                values.get('maximum_map_correction_translation', 0.08)
            ),
            maximum_map_correction_yaw_degrees=float(
                values.get('maximum_map_correction_yaw_degrees', 5.0)
            ),
            minimum_completed_inspections=int(
                values.get('minimum_completed_inspections', 0)
            ),
        )
        limits.validate()
        return limits

    def validate(self) -> None:
        if self.expected_terminal_state not in {
            'COMPLETE', 'BLOCKED', 'ESTOP'
        }:
            raise ValueError('expected_terminal_state 必须为 COMPLETE/BLOCKED/ESTOP')
        positive = {
            'readiness_stable_seconds': self.readiness_stable_seconds,
            'timeout_seconds': self.timeout_seconds,
            'maximum_linear_speed': self.maximum_linear_speed,
            'maximum_angular_speed': self.maximum_angular_speed,
            'maximum_map_correction_translation': (
                self.maximum_map_correction_translation
            ),
            'maximum_map_correction_yaw_degrees': (
                self.maximum_map_correction_yaw_degrees
            ),
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} 必须为有限正数')
        for name, value in {
            'maximum_retry_count': self.maximum_retry_count,
            'maximum_health_false_events': self.maximum_health_false_events,
            'minimum_completed_inspections': self.minimum_completed_inspections,
        }.items():
            if value < 0:
                raise ValueError(f'{name} 不能为负数')


def completed_inspection_count(status: Mapping[str, Any]) -> int:
    """Count action-confirmed completed inspection visits."""
    total = 0
    tasks = status.get('waypoint_tasks', [])
    if not isinstance(tasks, list):
        return 0
    for task in tasks:
        if not isinstance(task, Mapping) or not task.get('count_as_task'):
            continue
        total += max(0, int(task.get('completed_visits') or 0))
    return total


def evaluate_navigation_acceptance(
    report: Mapping[str, Any],
    limits: NavigationAcceptanceLimits,
) -> dict[str, Any]:
    """Evaluate a completed run and return auditable per-limit checks."""

    def at_most(field: str, maximum: float | int) -> dict[str, Any]:
        actual = report.get(field)
        passed = actual is not None and float(actual) <= float(maximum)
        return {'passed': passed, 'actual': actual, 'maximum': maximum}

    terminal = str(report.get('terminal_state', 'UNKNOWN')).upper()
    completed = int(report.get('completed_inspections', 0))
    checks = {
        'terminal_state': {
            'passed': terminal == limits.expected_terminal_state,
            'actual': terminal,
            'expected': limits.expected_terminal_state,
        },
        'elapsed_seconds': at_most(
            'elapsed_seconds', limits.timeout_seconds
        ),
        'maximum_linear_speed': at_most(
            'maximum_linear_speed', limits.maximum_linear_speed
        ),
        'maximum_angular_speed': at_most(
            'maximum_angular_speed', limits.maximum_angular_speed
        ),
        'maximum_retry_count': at_most(
            'maximum_retry_count', limits.maximum_retry_count
        ),
        'health_false_events': at_most(
            'health_false_events', limits.maximum_health_false_events
        ),
        'map_correction_translation': at_most(
            'maximum_map_correction_translation',
            limits.maximum_map_correction_translation,
        ),
        'map_correction_yaw_degrees': at_most(
            'maximum_map_correction_yaw_degrees',
            limits.maximum_map_correction_yaw_degrees,
        ),
        'completed_inspections': {
            'passed': completed >= limits.minimum_completed_inspections,
            'actual': completed,
            'minimum': limits.minimum_completed_inspections,
        },
        'final_stop': {
            'passed': bool(report.get('final_stop_verified')),
            'actual': bool(report.get('final_stop_verified')),
            'expected': True,
        },
    }
    return {
        'passed': all(check['passed'] for check in checks.values()),
        'checks': checks,
    }
