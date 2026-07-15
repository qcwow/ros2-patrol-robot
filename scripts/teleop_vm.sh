#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

echo "使用键盘控制机器人；按 Ctrl-C 退出。"
"${SSH[@]}" -t "${VM_TARGET}" \
  "source /opt/ros/jazzy/setup.bash; \
   source '${VM_WORKSPACE}/install/setup.bash'; \
   ros2 run teleop_twist_keyboard teleop_twist_keyboard"

