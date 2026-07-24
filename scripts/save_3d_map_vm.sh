#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

MAP_NAME="${1:-pipeline_map_3d}"
if [[ ! "${MAP_NAME}" =~ ^[A-Za-z0-9_-]+$ ]]; then
  echo "地图名称只能包含字母、数字、下划线和连字符。"
  exit 1
fi

REMOTE_MAP_DIR="${VM_WORKSPACE}/generated_maps/3d"
LOCAL_MAP_DIR="${PROJECT_ROOT}/src/patrol_robot_navigation/maps/3d"

"${SSH[@]}" "${VM_TARGET}" \
  "mkdir -p '${REMOTE_MAP_DIR}' && \
   source /opt/ros/jazzy/setup.bash && \
   source '${VM_WORKSPACE}/install/setup.bash' && \
   if ! ros2 node list | grep -qx '/octomap_server'; then \
     echo '错误：OctoMap 未运行。请先启动 3D 建图。'; \
     exit 2; \
   fi && \
   timeout 20s ros2 run octomap_server octomap_saver_node \
     --ros-args -p octomap_path:='${REMOTE_MAP_DIR}/${MAP_NAME}.ot' && \
   test -s '${REMOTE_MAP_DIR}/${MAP_NAME}.ot'"

mkdir -p "${LOCAL_MAP_DIR}"
RSYNC_SSH=(ssh -p "${VM_PORT}")
if [[ -n "${VM_SSH_KEY}" ]]; then
  RSYNC_SSH+=( -i "${VM_SSH_KEY}" -o IdentitiesOnly=yes )
fi
printf -v RSYNC_SSH_COMMAND '%q ' "${RSYNC_SSH[@]}"

rsync -az -e "${RSYNC_SSH_COMMAND% }" \
  "${VM_TARGET}:${REMOTE_MAP_DIR}/${MAP_NAME}.ot" \
  "${LOCAL_MAP_DIR}/"

echo "彩色 3D 体素地图已保存到 ${LOCAL_MAP_DIR}/${MAP_NAME}.ot"
