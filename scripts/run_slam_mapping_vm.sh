#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

echo "Gazebo/RViz 需要 Ubuntu 桌面显示授权，不能从普通 SSH 会话可靠打开。"
echo "请在虚拟机桌面的终端中执行："
echo "  cd ${VM_WORKSPACE}"
echo "  ./vm/run_mapping_gui.sh"
echo "该入口会启动 SLAM Toolbox、Nav2、巡检网页网关和 RViz。"
