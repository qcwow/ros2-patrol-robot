#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

LOCAL_MAP_DIR="${PROJECT_ROOT}/../../slam/maps"
REMOTE_MAP_DIR="${SLAM_VM_WORKSPACE}/src/inspection_slam_sim/maps"
DATABASE_FILE="/home/${VM_USER}/.ros/inspection_rtabmap_3d.db"
EXPORT_STAMP="$(date +%Y%m%d_%H%M%S)"
EXPORT_NAME="inspection_map_3d_${EXPORT_STAMP}"

echo "请确认3D建图程序已经停止，以便数据库完整写入。"
"${SSH[@]}" "${VM_TARGET}" \
  "test -s '${DATABASE_FILE}' && \
   mkdir -p '${REMOTE_MAP_DIR}' && \
   source /opt/ros/humble/setup.bash && \
   rtabmap-export \
     --cloud \
     --scan \
     --opt 2 \
     --voxel 0.05 \
     --filter_floor -0.10 \
     --filter_ceiling 3.00 \
     --output '${EXPORT_NAME}' \
     --output_dir '${REMOTE_MAP_DIR}' \
     '${DATABASE_FILE}'"

mkdir -p "${LOCAL_MAP_DIR}"
RSYNC_SSH=(ssh -p "${VM_PORT}")
if [[ -n "${VM_SSH_KEY}" ]]; then
  RSYNC_SSH+=( -i "${VM_SSH_KEY}" -o IdentitiesOnly=yes )
fi
printf -v RSYNC_SSH_COMMAND '%q ' "${RSYNC_SSH[@]}"

rsync -az -e "${RSYNC_SSH_COMMAND% }" \
  "${VM_TARGET}:${REMOTE_MAP_DIR}/${EXPORT_NAME}"'*.ply' \
  "${LOCAL_MAP_DIR}/"
rsync -az -e "${RSYNC_SSH_COMMAND% }" \
  "${VM_TARGET}:${DATABASE_FILE}" \
  "${LOCAL_MAP_DIR}/inspection_rtabmap_3d.db"

if "${SSH[@]}" "${VM_TARGET}" \
  "test -s '${REMOTE_MAP_DIR}/inspection_map.yaml' && \
   test -s '${REMOTE_MAP_DIR}/inspection_map.pgm'"; then
  rsync -az -e "${RSYNC_SSH_COMMAND% }" \
    "${VM_TARGET}:${REMOTE_MAP_DIR}/inspection_map.yaml" \
    "${VM_TARGET}:${REMOTE_MAP_DIR}/inspection_map.pgm" \
    "${LOCAL_MAP_DIR}/"
  echo "自动生成的AMCL二维地图已同步到 ${LOCAL_MAP_DIR}/inspection_map.{yaml,pgm}"
fi

echo "三维点云已同步到 ${LOCAL_MAP_DIR}/${EXPORT_NAME}*.ply"
echo "RTAB-Map数据库已同步到 ${LOCAL_MAP_DIR}/inspection_rtabmap_3d.db"
