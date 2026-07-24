#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

echo "正在只读检查 Ubuntu 网页建图接口。"
"${SSH[@]}" "${VM_TARGET}" \
  "python3 -c \"
import json
from urllib.request import urlopen

base = 'http://127.0.0.1:8765'
health = json.load(urlopen(base + '/api/health', timeout=3))
snapshot = json.load(urlopen(base + '/api/mapping/map', timeout=5))
library = json.load(urlopen(base + '/api/mapping/maps', timeout=3))
summary = {
    'gateway': health.get('service'),
    'frame_id': snapshot.get('frame_id'),
    'map_size': [
        snapshot.get('width'),
        snapshot.get('height'),
    ],
    'resolution': snapshot.get('resolution'),
    'coverage': snapshot.get('coverage'),
    'known_cells': snapshot.get('known_cells'),
    'robot': snapshot.get('robot'),
    'mapping': snapshot.get('mapping'),
    'camera': snapshot.get('camera'),
    'saved_maps': len(library.get('maps', [])),
}
print(json.dumps(summary, ensure_ascii=False, indent=2))
\""
