#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

"${SCRIPT_DIR}/sync_to_vm.sh"

echo "即将在虚拟机安装 ROS 2 Humble、Nav2、Gazebo 和 SLAM Toolbox。"
"${SSH[@]}" -t "${VM_TARGET}" "bash '${VM_WORKSPACE}/vm/install_ros2_humble.sh'"
