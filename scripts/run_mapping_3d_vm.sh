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

echo "正在虚拟机启动完整 3D 集成系统：车辆网关、Gazebo、SLAM、Nav2、前沿探索和真实 RGB-D OctoMap。"
echo "车辆网关将监听 0.0.0.0:8765；启动后保持人工模式。"
echo "点击网页“一键自主探路”后才会开始探索，3D 体素页显示真实 OctoMap 数据。"
echo "无界面 RGB-D 传感器将使用 Mesa 软件 OpenGL，避免 VMware 的 EGL 设备段错误。"
echo "网页入口：http://localhost:3000/?robot=http%3A%2F%2F127.0.0.1%3A${LOCAL_GATEWAY_PORT}"
"${SSH[@]}" \
  -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=15 \
  -o ServerAliveCountMax=3 \
  -L "${LOCAL_GATEWAY_PORT}:127.0.0.1:8765" \
  -t "${VM_TARGET}" \
  "export RCUTILS_COLORIZED_OUTPUT=1; \
   export LIBGL_ALWAYS_SOFTWARE=1; \
   export LIBGL_DRI3_DISABLE=1; \
   export MESA_GL_VERSION_OVERRIDE=4.3; \
   export QT_QPA_PLATFORM=offscreen; \
   source /opt/ros/jazzy/setup.bash; \
   if [[ ! -f '${VM_WORKSPACE}/install/setup.bash' ]]; then \
     echo '错误：虚拟机工作空间尚未编译。请先在 Mac 执行 ./scripts/build_vm.sh'; \
     exit 2; \
   fi; \
   source '${VM_WORKSPACE}/install/setup.bash'; \
   ros2 launch patrol_robot_bringup simulation_mapping.launch.py \
     use_gazebo:=true enable_3d_mapping:=true \
     enable_autonomous_exploration:=true autonomous_exploration_autostart:=false \
     patrol_autostart:=false \
     ground_truth_odometry:=false \
     headless:=true start_rviz:=false"
