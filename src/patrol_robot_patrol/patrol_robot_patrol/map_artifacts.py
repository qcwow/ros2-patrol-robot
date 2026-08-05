"""Pure validation helpers for Nav2 map YAML and PGM artifacts."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

import yaml


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _pgm_dimensions(path: Path) -> tuple[int, int, int]:
    with path.open('rb') as stream:
        magic = stream.readline().strip()
        if magic not in {b'P2', b'P5'}:
            raise ValueError('地图图像必须是 PGM P2/P5')

        tokens: list[bytes] = []
        while len(tokens) < 3:
            line = stream.readline()
            if not line:
                break
            line = line.split(b'#', 1)[0]
            tokens.extend(line.split())
    if len(tokens) < 3:
        raise ValueError('PGM 头不完整')
    width, height, maximum = (int(value) for value in tokens[:3])
    if width <= 0 or height <= 0 or not 0 < maximum <= 65535:
        raise ValueError('PGM 尺寸或灰度范围无效')
    return width, height, maximum


def validate_map_artifact(map_yaml: str | Path) -> dict[str, Any]:
    """Validate a saved Nav2 map and return a reproducibility manifest."""
    yaml_path = Path(map_yaml).expanduser().resolve()
    if not yaml_path.is_file():
        raise ValueError(f'地图 YAML 不存在: {yaml_path}')
    document = yaml.safe_load(yaml_path.read_text(encoding='utf-8')) or {}
    if not isinstance(document, dict):
        raise ValueError('地图 YAML 顶层必须是对象')

    image_value = str(document.get('image', '')).strip()
    if not image_value:
        raise ValueError('地图 YAML 缺少 image')
    image_path = Path(image_value).expanduser()
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    image_path = image_path.resolve()
    if not image_path.is_file():
        raise ValueError(f'地图图像不存在: {image_path}')

    resolution = float(document.get('resolution', 0.0))
    origin = document.get('origin')
    occupied = float(document.get('occupied_thresh', -1.0))
    free = float(document.get('free_thresh', -1.0))
    if resolution <= 0.0:
        raise ValueError('resolution 必须为正数')
    if (
        not isinstance(origin, list)
        or len(origin) != 3
        or not all(isinstance(value, (int, float)) for value in origin)
    ):
        raise ValueError('origin 必须包含三个数值')
    if not 0.0 <= free < occupied <= 1.0:
        raise ValueError('必须满足 0 <= free_thresh < occupied_thresh <= 1')

    width, height, maximum = _pgm_dimensions(image_path)
    return {
        'schema_version': 1,
        'valid': True,
        'yaml_path': str(yaml_path),
        'image_path': str(image_path),
        'yaml_sha256': _sha256(yaml_path),
        'image_sha256': _sha256(image_path),
        'resolution': resolution,
        'origin': [float(value) for value in origin],
        'occupied_thresh': occupied,
        'free_thresh': free,
        'negate': int(document.get('negate', 0)),
        'mode': str(document.get('mode', 'trinary')),
        'image_width': width,
        'image_height': height,
        'image_maximum': maximum,
        'map_width_meters': round(width * resolution, 4),
        'map_height_meters': round(height * resolution, 4),
    }


def validate_map_manifest(
    map_yaml: str | Path,
    required_profile: str,
) -> dict[str, Any]:
    """Require a matching saved manifest before loading a real-car map."""
    current = validate_map_artifact(map_yaml)
    manifest_path = Path(current['yaml_path']).with_suffix('.manifest.json')
    if not manifest_path.is_file():
        raise ValueError(f'地图缺少校验清单: {manifest_path}')
    try:
        saved = json.loads(manifest_path.read_text(encoding='utf-8'))
    except (json.JSONDecodeError, OSError) as error:
        raise ValueError(f'地图校验清单无效: {error}') from error
    if saved.get('profile') != required_profile:
        raise ValueError(
            f"地图 profile 必须为 {required_profile}，实际为 "
            f"{saved.get('profile')!r}"
        )
    for field in ('yaml_sha256', 'image_sha256'):
        if saved.get(field) != current[field]:
            raise ValueError(f'地图清单哈希不匹配: {field}')
    current['profile'] = required_profile
    current['manifest_path'] = str(manifest_path)
    current['manifest_verified'] = True
    return current
