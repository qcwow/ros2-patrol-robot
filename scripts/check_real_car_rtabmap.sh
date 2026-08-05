#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_real_car.sh"

failed=0
nodes="$(ros2 node list 2>/dev/null || true)"

for node_name in /rtabmap /rgbd_sync /obstacles_detection; do
  if ! grep -qx "${node_name}" <<<"${nodes}"; then
    echo "缺少节点：${node_name}"
    failed=1
  fi
done

for conflicting_node in /slam_toolbox /amcl; do
  if grep -qx "${conflicting_node}" <<<"${nodes}"; then
    echo "TF 冲突风险：RTAB-Map 建图时不应运行 ${conflicting_node}"
    failed=1
  fi
done

for topic_name in \
  /scan_raw \
  /odom \
  /rtabmap/rgbd_image \
  /camera/obstacles \
  /map \
  /rtabmap/cloud_map; do
  if ! ros2 topic info "${topic_name}" >/dev/null 2>&1; then
    echo "缺少话题：${topic_name}"
    failed=1
  fi
done

map_publishers="$(
  ros2 topic info /map 2>/dev/null \
    | awk '/Publisher count:/ {print $3}'
)"
if [[ "${map_publishers:-0}" != "1" ]]; then
  echo "异常：/map 发布者数量为 ${map_publishers:-0}，预期为 1。"
  failed=1
fi

if [[ "${failed}" -ne 0 ]]; then
  echo "RTAB-Map 启动检查失败，禁止落地自主运动。"
  exit 9
fi

echo "RTAB-Map 节点、输入输出话题和单一 /map 发布者检查通过。"
echo "仍需确认 RGB/Depth 配准、map->odom 稳定和 /camera/obstacles 中没有地面点。"
ros2 topic info /map --verbose
