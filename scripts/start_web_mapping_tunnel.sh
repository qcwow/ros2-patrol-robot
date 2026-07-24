#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

LOCAL_PORT="${WEB_MAPPING_LOCAL_PORT:-8765}"

if [[ ! "${LOCAL_PORT}" =~ ^[0-9]+$ ]]; then
  echo "WEB_MAPPING_LOCAL_PORT 必须是数字。"
  exit 1
fi

echo "正在建立网页建图安全通道："
echo "  Ubuntu: ${VM_TARGET}:8765"
echo "  本机:   http://127.0.0.1:${LOCAL_PORT}"
echo
echo "请保持此窗口开启，然后访问："
echo "http://localhost:3000/?robot=http%3A%2F%2F127.0.0.1%3A${LOCAL_PORT}"
echo
echo "按 Ctrl+C 可关闭通道（不会停止 Ubuntu 仿真）。"

exec "${SSH[@]}" \
  -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=15 \
  -o ServerAliveCountMax=3 \
  -N \
  -L "${LOCAL_PORT}:127.0.0.1:8765" \
  "${VM_TARGET}"
