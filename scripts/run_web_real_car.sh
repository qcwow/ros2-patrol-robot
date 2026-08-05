#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(CDPATH= cd -- "${SCRIPT_DIR}/.." && pwd)"
FRONTEND_DIR="${PROJECT_ROOT}/src/patrol_robot_web"
REAL_CAR_HOST="${1:-${REAL_CAR_HOST:-192.168.100.137}}"
REAL_CAR_GATEWAY_PORT="${REAL_CAR_WEB_PORT:-8765}"
FRONTEND_PORT="${WEB_FRONTEND_PORT:-3000}"

if [[ ! "${REAL_CAR_GATEWAY_PORT}" =~ ^[0-9]+$ \
      || ! "${FRONTEND_PORT}" =~ ^[0-9]+$ ]]; then
  echo "错误：网关端口和前端端口必须是数字。"
  exit 2
fi

if [[ ! "${REAL_CAR_HOST}" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "错误：真车主机名/IP 格式无效。"
  exit 2
fi

if ! command -v node >/dev/null 2>&1 || ! command -v npm >/dev/null 2>&1; then
  echo "错误：Mac 尚未安装 Node.js 22.13 或更高版本。"
  exit 3
fi

GATEWAY_URL="http://${REAL_CAR_HOST}:${REAL_CAR_GATEWAY_PORT}"
if ! curl -fsS --max-time 3 "${GATEWAY_URL}/api/health" >/dev/null; then
  echo "错误：无法连接真车网页网关 ${GATEWAY_URL}。"
  echo "请确认真车已启动 real_car_mapping 或 real_car_navigation，且 Mac 与车在同一可信网络。"
  exit 4
fi

ENCODED_GATEWAY="http%3A%2F%2F${REAL_CAR_HOST}%3A${REAL_CAR_GATEWAY_PORT}"
FRONTEND_URL="http://localhost:${FRONTEND_PORT}/?robot=${ENCODED_GATEWAY}"

cd "${FRONTEND_DIR}"
if [[ ! -d node_modules ]]; then
  echo "首次运行，正在安装前端依赖。"
  npm install
fi

echo "网页将连接真车网关：${GATEWAY_URL}"
echo "控制地址：${FRONTEND_URL}"
echo "仅允许在可信内网使用；不要将 8765 端口暴露到互联网。"
if [[ "${WEB_OPEN_BROWSER:-true}" == true ]] \
    && command -v open >/dev/null 2>&1; then
  (
    sleep 2
    open "${FRONTEND_URL}"
  ) &
fi

exec npm run dev -- --port "${FRONTEND_PORT}"
