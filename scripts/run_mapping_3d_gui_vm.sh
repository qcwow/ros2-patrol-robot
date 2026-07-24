#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

LOCAL_GATEWAY_PORT="${WEB_MAPPING_LOCAL_PORT:-8765}"
if [[ ! "${LOCAL_GATEWAY_PORT}" =~ ^[0-9]+$ ]]; then
  echo "WEB_MAPPING_LOCAL_PORT 必须是数字。"
  exit 1
fi

echo "正在复用 Ubuntu 桌面，以 CPU 软件渲染启动 Gazebo、RViz 和 3D 建图。"
echo "车辆网关同步转发到：http://127.0.0.1:${LOCAL_GATEWAY_PORT}"
echo "网页入口：http://localhost:3000/?robot=http%3A%2F%2F127.0.0.1%3A${LOCAL_GATEWAY_PORT}"
"${SSH[@]}" \
  -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=15 \
  -o ServerAliveCountMax=3 \
  -L "${LOCAL_GATEWAY_PORT}:127.0.0.1:8765" \
  -t "${VM_TARGET}" \
  "set -e; \
   runtime_dir=\"/run/user/\$(id -u)\"; \
   xauthority=\"\$(find \"\${runtime_dir}\" -maxdepth 1 \
     -name '.mutter-Xwaylandauth.*' -print -quit)\"; \
   if [[ -z \"\${xauthority}\" || ! -S \"\${runtime_dir}/wayland-0\" ]]; then \
     echo '错误：Ubuntu 图形桌面尚未登录。请先进入虚拟机桌面后重试。'; \
     exit 2; \
   fi; \
   export DISPLAY='${VM_DISPLAY}'; \
   export XAUTHORITY=\"\${xauthority}\"; \
   export XDG_RUNTIME_DIR=\"\${runtime_dir}\"; \
   export DBUS_SESSION_BUS_ADDRESS=\"unix:path=\${runtime_dir}/bus\"; \
   export LIBGL_ALWAYS_SOFTWARE=1; \
   export LIBGL_DRI3_DISABLE=1; \
   export MESA_GL_VERSION_OVERRIDE=3.3; \
   cd '${VM_WORKSPACE}'; \
   exec ./vm/run_mapping_3d_gui.sh"
