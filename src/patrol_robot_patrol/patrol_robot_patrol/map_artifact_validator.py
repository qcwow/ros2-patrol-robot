"""CLI for validating and checksumming a saved Nav2 map."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from patrol_robot_patrol.map_artifacts import (
    validate_map_artifact,
    validate_map_manifest,
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('map_yaml')
    parser.add_argument('--write-manifest', action='store_true')
    parser.add_argument('--profile', default='')
    parser.add_argument('--require-manifest-profile', default='')
    arguments = parser.parse_args()
    if arguments.require_manifest_profile:
        manifest = validate_map_manifest(
            arguments.map_yaml, arguments.require_manifest_profile
        )
    else:
        manifest = validate_map_artifact(arguments.map_yaml)
    if arguments.write_manifest:
        if not arguments.profile:
            parser.error('--write-manifest 必须同时设置 --profile')
        manifest['profile'] = arguments.profile
        output = Path(manifest['yaml_path']).with_suffix('.manifest.json')
        output.write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8',
        )
        manifest['manifest_path'] = str(output)
    print(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
