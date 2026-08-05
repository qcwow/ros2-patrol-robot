#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

echo "SLAM Toolbox 已并入巡检工作空间，转交统一构建脚本。"
exec "${SCRIPT_DIR}/build_vm.sh"
