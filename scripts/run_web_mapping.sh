#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

FRONTEND_DIR="${PROJECT_ROOT}/src/patrol_robot_web"
LOCAL_GATEWAY_PORT="${WEB_MAPPING_LOCAL_PORT:-8765}"
FRONTEND_PORT="${WEB_FRONTEND_PORT:-3000}"
FRONTEND_URL="http://localhost:${FRONTEND_PORT}/?robot=http%3A%2F%2F127.0.0.1%3A${LOCAL_GATEWAY_PORT}"
TUNNEL_PID=""

cleanup() {
  if [[ -n "${TUNNEL_PID}" ]]; then
    kill "${TUNNEL_PID}" >/dev/null 2>&1 || true
    wait "${TUNNEL_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if ! command -v node >/dev/null 2>&1 || ! command -v npm >/dev/null 2>&1; then
  echo "错误：Mac 尚未安装 Node.js 22.13 或更高版本。"
  echo "可使用 Homebrew 安装：brew install node"
  exit 2
fi

if ! curl -fsS --max-time 1 \
    "http://127.0.0.1:${LOCAL_GATEWAY_PORT}/api/health" >/dev/null 2>&1; then
  echo "正在把 Ubuntu ROS 网关连接到 Mac 端口 ${LOCAL_GATEWAY_PORT}。"
  "${SSH[@]}" \
    -o ExitOnForwardFailure=yes \
    -o ServerAliveInterval=15 \
    -o ServerAliveCountMax=3 \
    -N \
    -L "${LOCAL_GATEWAY_PORT}:127.0.0.1:8765" \
    "${VM_TARGET}" &
  TUNNEL_PID=$!
  sleep 1
  if ! kill -0 "${TUNNEL_PID}" >/dev/null 2>&1; then
    wait "${TUNNEL_PID}"
  fi
fi

if ! curl -fsS --max-time 3 \
    "http://127.0.0.1:${LOCAL_GATEWAY_PORT}/api/health" >/dev/null; then
  echo "错误：Ubuntu 的 ROS Web 网关尚未就绪。"
  echo "请先在 Ubuntu 桌面运行：~/robot_patrol_ws/vm/run_mapping_gui.sh"
  exit 2
fi

cd "${FRONTEND_DIR}"
if [[ ! -d node_modules ]]; then
  echo "首次运行，正在安装 Mac 前端依赖。"
  npm install
fi

echo "前端即将在 Mac 启动：${FRONTEND_URL}"
echo "保持此终端运行；按 Ctrl+C 会同时关闭前端和 SSH 通道。"
if [[ "${WEB_OPEN_BROWSER:-true}" == "true" ]] && \
    command -v open >/dev/null 2>&1; then
  (
    sleep 2
    open "${FRONTEND_URL}"
  ) &
fi

npm run dev -- --port "${FRONTEND_PORT}"
