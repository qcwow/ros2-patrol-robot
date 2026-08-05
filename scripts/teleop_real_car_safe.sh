#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_real_car.sh"

RAW_TOPIC="${REAL_CAR_RAW_CMD_TOPIC:-/cmd_vel_manual_raw}"
echo "键盘速度将发送到 ${RAW_TOPIC}，不会直接发送到底盘。"
echo "必须同时运行真车安全建图/安全过滤启动链。"
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:="${RAW_TOPIC}"
