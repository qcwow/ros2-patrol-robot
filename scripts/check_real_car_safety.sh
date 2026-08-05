#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_real_car.sh"

failed=0
for topic_name in /scan_raw /cmd_vel_safety_checked /controller/cmd_vel; do
  if ! ros2 topic info "${topic_name}" >/dev/null 2>&1; then
    echo "缺少话题：${topic_name}"
    failed=1
  fi
done

lidar_safety_found=false
base_watchdog_found=false
while IFS= read -r node_name; do
  if [[ "${node_name}" == /manual_lidar_safety ]]; then
    lidar_safety_found=true
  elif [[ "${node_name}" == /base_command_watchdog ]]; then
    base_watchdog_found=true
  fi
done < <(ros2 node list)
if [[ "${lidar_safety_found}" != true ]]; then
  echo "缺少节点：/manual_lidar_safety"
  failed=1
fi
if [[ "${base_watchdog_found}" != true ]]; then
  echo "缺少节点：/base_command_watchdog"
  failed=1
fi

if [[ "${failed}" -ne 0 ]]; then
  echo "安全链检查失败，禁止落地运动。"
  exit 7
fi

echo "安全节点与基础话题存在。仍需架空车轮测试雷达遮挡和速度源失联停车。"
ros2 topic info /controller/cmd_vel --verbose
