#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

LOCAL_MAP_DIR="${PROJECT_ROOT}/../../slam/maps"
REMOTE_MAP_DIR="${SLAM_VM_WORKSPACE}/src/inspection_slam_sim/maps"
MAP_NAME="${1:-inspection_map}"

"${SSH[@]}" "${VM_TARGET}" \
  "mkdir -p '${REMOTE_MAP_DIR}' && \
   source /opt/ros/humble/setup.bash && \
   source '${SLAM_VM_WORKSPACE}/install/setup.bash' && \
   ros2 run nav2_map_server map_saver_cli \
     -f '${REMOTE_MAP_DIR}/${MAP_NAME}' \
     --ros-args -p save_map_timeout:=20.0"

mkdir -p "${LOCAL_MAP_DIR}"
RSYNC_SSH=(ssh -p "${VM_PORT}")
if [[ -n "${VM_SSH_KEY}" ]]; then
  RSYNC_SSH+=( -i "${VM_SSH_KEY}" -o IdentitiesOnly=yes )
fi
printf -v RSYNC_SSH_COMMAND '%q ' "${RSYNC_SSH[@]}"
rsync -az -e "${RSYNC_SSH_COMMAND% }" \
  "${VM_TARGET}:${REMOTE_MAP_DIR}/${MAP_NAME}.pgm" \
  "${VM_TARGET}:${REMOTE_MAP_DIR}/${MAP_NAME}.yaml" \
  "${LOCAL_MAP_DIR}/"

echo "地图已保存并同步到 ${LOCAL_MAP_DIR}/${MAP_NAME}.{yaml,pgm}"
