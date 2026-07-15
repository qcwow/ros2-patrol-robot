#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

MAP_NAME="${1:-pipeline_map_new}"
if [[ ! "${MAP_NAME}" =~ ^[A-Za-z0-9_-]+$ ]]; then
  echo "地图名称只能包含字母、数字、下划线和连字符。"
  exit 1
fi

REMOTE_MAP_DIR="${VM_WORKSPACE}/generated_maps"
LOCAL_MAP_DIR="${PROJECT_ROOT}/src/patrol_robot_navigation/maps"

"${SSH[@]}" "${VM_TARGET}" \
  "mkdir -p '${REMOTE_MAP_DIR}' && \
   source /opt/ros/jazzy/setup.bash && \
   source '${VM_WORKSPACE}/install/setup.bash' && \
   ros2 run nav2_map_server map_saver_cli \
     -f '${REMOTE_MAP_DIR}/${MAP_NAME}' --ros-args -p save_map_timeout:=10.0"

rsync -az -e "ssh -p ${VM_PORT}" \
  "${VM_TARGET}:${REMOTE_MAP_DIR}/${MAP_NAME}.pgm" \
  "${VM_TARGET}:${REMOTE_MAP_DIR}/${MAP_NAME}.yaml" \
  "${LOCAL_MAP_DIR}/"

echo "地图已保存到 ${LOCAL_MAP_DIR}/${MAP_NAME}.yaml"

