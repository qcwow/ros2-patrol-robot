#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_real_car.sh"

MAP_NAME="${1:-}"
if [[ ! "${MAP_NAME}" =~ ^[A-Za-z0-9_-]+$ ]]; then
  echo "用法：scripts/save_real_car_map.sh 地图名"
  echo "地图名只能包含字母、数字、下划线和连字符。"
  exit 8
fi

MAP_DIR="${REAL_CAR_MAP_DIR:-${REAL_CAR_WORKSPACE}/src/patrol_robot_navigation/maps}"
mkdir -p "${MAP_DIR}"
MAP_PREFIX="${MAP_DIR}/${MAP_NAME}"

ros2 run nav2_map_server map_saver_cli \
  -f "${MAP_PREFIX}" \
  --ros-args \
  -p map_subscribe_transient_local:=true \
  -p save_map_timeout:=10.0

ros2 run patrol_robot_patrol map_artifact_validator \
  "${MAP_PREFIX}.yaml" --write-manifest --profile real_car

echo "真车地图已保存并校验：${MAP_PREFIX}.yaml / ${MAP_PREFIX}.pgm"
echo "校验清单：${MAP_PREFIX}.manifest.json"
