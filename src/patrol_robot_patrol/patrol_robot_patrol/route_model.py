"""Validated semantic route model for patrol navigation."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any


WAYPOINT_TYPES = frozenset({'HOME', 'TRANSIT', 'INSPECTION'})
RECOVERY_POLICIES = frozenset({'standard', 'no_spin', 'restricted'})


@dataclass(frozen=True)
class Waypoint:
    waypoint_id: str
    name: str
    x: float
    y: float
    yaw: float
    dwell: float
    waypoint_type: str
    position_tolerance: float
    yaw_tolerance: float
    speed_limit: float
    recovery_policy: str
    required_sensor: str
    count_as_task: bool
    route_id: str

    @property
    def role(self) -> str:
        return self.waypoint_type.lower()


@dataclass(frozen=True)
class PatrolRoute:
    frame_id: str
    route_id: str
    waypoints: tuple[Waypoint, ...]
    home_index: int
    ordered_indices: tuple[int, ...]
    task_indices: tuple[int, ...]


def _bounded_float(
    value: Any,
    field: str,
    minimum: float,
    maximum: float,
) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{field} 必须为数字') from error
    if not minimum <= parsed <= maximum:
        raise ValueError(f'{field} 必须在 {minimum}～{maximum} 之间')
    return parsed


def _waypoint_defaults(waypoint_type: str, default_dwell: float) -> dict[str, Any]:
    if waypoint_type == 'HOME':
        return {
            'dwell': 0.0,
            'position_tolerance': 0.08,
            'yaw_tolerance': 0.10,
            'speed_limit': 0.15,
            'recovery_policy': 'restricted',
            'count_as_task': False,
        }
    if waypoint_type == 'TRANSIT':
        return {
            'dwell': 0.0,
            'position_tolerance': 0.20,
            'yaw_tolerance': 0.25,
            'speed_limit': 0.35,
            'recovery_policy': 'no_spin',
            'count_as_task': False,
        }
    return {
        'dwell': default_dwell,
        'position_tolerance': 0.08,
        'yaw_tolerance': 0.10,
        'speed_limit': 0.15,
        'recovery_policy': 'restricted',
        'count_as_task': True,
    }


def parse_route(document: Any, default_dwell: float = 2.0) -> PatrolRoute:
    if not isinstance(document, dict):
        raise ValueError('路线文件顶层必须为对象')

    frame_id = str(document.get('frame_id', 'map')).strip() or 'map'
    route_id = str(document.get('route_id', 'default_route')).strip()
    if not route_id:
        raise ValueError('route_id 不能为空')

    raw_waypoints = document.get('waypoints')
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError('路线必须包含非空的 waypoints 列表')

    explicit_types = any(
        isinstance(item, dict) and ('type' in item or 'waypoint_type' in item)
        for item in raw_waypoints
    )
    waypoints: list[Waypoint] = []
    waypoint_ids: set[str] = set()

    for index, item in enumerate(raw_waypoints):
        if not isinstance(item, dict):
            raise ValueError(f'第 {index + 1} 个路线点不是对象')

        fallback_type = 'HOME' if index == 0 and not explicit_types else 'INSPECTION'
        waypoint_type = str(
            item.get('type', item.get('waypoint_type', fallback_type))
        ).strip().upper()
        if waypoint_type not in WAYPOINT_TYPES:
            raise ValueError(
                f'第 {index + 1} 个路线点 type 必须是 '
                'HOME、TRANSIT 或 INSPECTION'
            )

        waypoint_id = str(
            item.get('id', item.get('waypoint_id', f'waypoint_{index + 1}'))
        ).strip()
        if not waypoint_id:
            raise ValueError(f'第 {index + 1} 个路线点 id 不能为空')
        if waypoint_id in waypoint_ids:
            raise ValueError(f'路线点 id 重复: {waypoint_id}')
        waypoint_ids.add(waypoint_id)

        defaults = _waypoint_defaults(waypoint_type, default_dwell)
        tolerance = item.get('tolerance', {})
        if tolerance is None:
            tolerance = {}
        if not isinstance(tolerance, dict):
            raise ValueError(f'第 {index + 1} 个路线点 tolerance 必须为对象')

        position_tolerance = item.get(
            'position_tolerance',
            tolerance.get('position', defaults['position_tolerance']),
        )
        yaw_tolerance = item.get(
            'yaw_tolerance',
            tolerance.get('yaw', defaults['yaw_tolerance']),
        )
        count_as_task = bool(item.get('count_as_task', defaults['count_as_task']))
        if waypoint_type != 'INSPECTION' and count_as_task:
            raise ValueError(
                f'第 {index + 1} 个路线点只有 INSPECTION 才能计为巡检任务'
            )

        recovery_policy = str(
            item.get('recovery_policy', defaults['recovery_policy'])
        ).strip().lower()
        if recovery_policy not in RECOVERY_POLICIES:
            raise ValueError(
                f'第 {index + 1} 个路线点 recovery_policy 必须是 '
                'standard、no_spin 或 restricted'
            )

        point_route_id = str(item.get('route_id', route_id)).strip()
        if point_route_id != route_id:
            raise ValueError(
                f'第 {index + 1} 个路线点 route_id 与路线顶层不一致'
            )

        try:
            waypoint = Waypoint(
                waypoint_id=waypoint_id,
                name=str(item.get('name', f'waypoint_{index + 1}')).strip()
                or f'waypoint_{index + 1}',
                x=_bounded_float(
                    item['x'], f'第 {index + 1} 个路线点 x', -10000.0, 10000.0
                ),
                y=_bounded_float(
                    item['y'], f'第 {index + 1} 个路线点 y', -10000.0, 10000.0
                ),
                yaw=_bounded_float(
                    item.get('yaw', 0.0),
                    f'第 {index + 1} 个路线点 yaw',
                    -6.283186,
                    6.283186,
                ),
                dwell=_bounded_float(
                    item.get('dwell', defaults['dwell']),
                    f'第 {index + 1} 个路线点 dwell',
                    0.0,
                    3600.0,
                ),
                waypoint_type=waypoint_type,
                position_tolerance=_bounded_float(
                    position_tolerance,
                    f'第 {index + 1} 个路线点位置容差',
                    0.01,
                    1.0,
                ),
                yaw_tolerance=_bounded_float(
                    yaw_tolerance,
                    f'第 {index + 1} 个路线点朝向容差',
                    0.01,
                    3.141593,
                ),
                speed_limit=_bounded_float(
                    item.get('speed_limit', defaults['speed_limit']),
                    f'第 {index + 1} 个路线点 speed_limit',
                    0.05,
                    1.5,
                ),
                recovery_policy=recovery_policy,
                required_sensor=str(item.get('required_sensor', 'none')).strip().lower()
                or 'none',
                count_as_task=count_as_task,
                route_id=route_id,
            )
        except (KeyError, TypeError, ValueError) as error:
            if isinstance(error, ValueError) and str(error).startswith('第 '):
                raise
            raise ValueError(f'第 {index + 1} 个路线点格式错误: {error}') from error
        if waypoint.required_sensor not in {'none', 'lidar', 'rgbd', 'fusion'}:
            raise ValueError(
                f'第 {index + 1} 个路线点 required_sensor 必须是 '
                'none、lidar、rgbd 或 fusion'
            )
        waypoints.append(waypoint)

    home_indices = [
        index for index, waypoint in enumerate(waypoints)
        if waypoint.waypoint_type == 'HOME'
    ]
    if len(home_indices) != 1:
        raise ValueError(
            f'路线必须且只能包含一个 HOME，当前为 {len(home_indices)} 个'
        )

    home_index = home_indices[0]
    waypoint_count = len(waypoints)
    ordered_indices = tuple(
        (home_index + offset) % waypoint_count
        for offset in range(waypoint_count)
    )
    task_indices = tuple(
        index for index, waypoint in enumerate(waypoints)
        if waypoint.count_as_task
    )
    return PatrolRoute(
        frame_id=frame_id,
        route_id=route_id,
        waypoints=tuple(waypoints),
        home_index=home_index,
        ordered_indices=ordered_indices,
        task_indices=task_indices,
    )


def load_route(path: str, default_dwell: float = 2.0) -> PatrolRoute:
    import yaml

    route_path = Path(path).expanduser()
    if not route_path.is_file():
        raise FileNotFoundError(f'巡航路线文件不存在: {route_path}')
    with route_path.open('r', encoding='utf-8') as stream:
        return parse_route(yaml.safe_load(stream) or {}, default_dwell)
