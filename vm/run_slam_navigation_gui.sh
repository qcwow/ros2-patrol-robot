#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="${HOME}/slam_ws"
MAP_FILE="${1:-${WORKSPACE_DIR}/src/inspection_slam_sim/maps/inspection_map.yaml}"

if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  echo "错误：独立 SLAM 工作空间尚未构建。"
  exit 2
fi
if [[ ! -f "${MAP_FILE}" ]]; then
  echo "错误：找不到地图 ${MAP_FILE}"
  exit 2
fi

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
exec ros2 launch inspection_slam_sim navigation.launch.py map:="${MAP_FILE}"
