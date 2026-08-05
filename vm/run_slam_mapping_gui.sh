#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

echo "独立 SLAM 入口已合并到统一 SLAM Toolbox 启动入口。"
exec "${SCRIPT_DIR}/run_mapping_gui.sh"
