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

echo "正在虚拟机启动完整二维集成系统：车辆网关、SLAM Toolbox、Nav2、前沿探索与巡检管理。"
echo "车辆网关将监听 0.0.0.0:8765；启动后保持人工模式，不会自动建图或巡检。"
echo "网页入口：http://localhost:3000/?robot=http%3A%2F%2F127.0.0.1%3A${LOCAL_GATEWAY_PORT}"
echo "如需 Gazebo/RViz 窗口，请在虚拟机桌面终端运行 vm/run_mapping_gui.sh。"
"${SSH[@]}" \
  -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=15 \
  -o ServerAliveCountMax=3 \
  -L "${LOCAL_GATEWAY_PORT}:127.0.0.1:8765" \
  -t "${VM_TARGET}" \
  "export RCUTILS_COLORIZED_OUTPUT=1; \
   export LIBGL_ALWAYS_SOFTWARE=1; \
   source /opt/ros/humble/setup.bash; \
   if [[ ! -f '${VM_WORKSPACE}/install/setup.bash' ]]; then \
     echo '错误：虚拟机工作空间尚未编译。请先在 Mac 执行 ./scripts/build_vm.sh'; \
     exit 2; \
   fi; \
   source '${VM_WORKSPACE}/install/setup.bash'; \
   ros2 launch patrol_robot_bringup simulation_mapping.launch.py \
     use_gazebo:=false start_rviz:=false \
     enable_autonomous_exploration:=true autonomous_exploration_autostart:=false \
     patrol_autostart:=false"
