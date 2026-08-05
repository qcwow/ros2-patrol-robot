#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_real_car.sh"

require_real_car_packages
cd "${REAL_CAR_WORKSPACE}"
colcon build --symlink-install \
  --packages-up-to \
    patrol_robot_bringup \
    patrol_robot_navigation \
    patrol_robot_patrol \
    patrol_robot_web_bridge

echo "真车 overlay 构建完成：${REAL_CAR_WORKSPACE}/install/setup.bash"
