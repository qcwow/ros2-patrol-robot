#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

LOCAL_MAP_DIR="${PROJECT_ROOT}/../../slam/maps"
REMOTE_MAP_DIR="${SLAM_VM_WORKSPACE}/src/inspection_slam_sim/maps"
PCD_FILE="${REMOTE_MAP_DIR}/point_lio_map.pcd"
MAP_NAME="inspection_map"

echo "正在保存Point-LIO实验性3D点云。"
"${SSH[@]}" "${VM_TARGET}" \
  "source /opt/ros/humble/setup.bash && \
   source '${SLAM_VM_WORKSPACE}/install/setup.bash' && \
   ros2 service call /funny_lidar_slam/save_map std_srvs/srv/Trigger '{}'"

echo "正在保存供Nav2/AMCL使用的二维地图。"
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
  "${VM_TARGET}:${PCD_FILE}" \
  "${VM_TARGET}:${REMOTE_MAP_DIR}/${MAP_NAME}.pgm" \
  "${VM_TARGET}:${REMOTE_MAP_DIR}/${MAP_NAME}.yaml" \
  "${LOCAL_MAP_DIR}/"

echo "3D地图已同步到 ${LOCAL_MAP_DIR}/point_lio_map.pcd"
echo "AMCL地图已同步到 ${LOCAL_MAP_DIR}/${MAP_NAME}.{yaml,pgm}"
