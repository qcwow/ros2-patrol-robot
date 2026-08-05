#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

echo "正在虚拟机启动轻量二维仿真器、Nav2 和自动巡航。"
echo "如需 Gazebo/RViz 窗口，请在虚拟机桌面终端运行 vm/run_navigation_gui.sh。"
"${SSH[@]}" -t "${VM_TARGET}" \
  "export RCUTILS_COLORIZED_OUTPUT=1; \
   export LIBGL_ALWAYS_SOFTWARE=1; \
   source /opt/ros/humble/setup.bash; \
   if [[ ! -f '${VM_WORKSPACE}/install/setup.bash' ]]; then \
     echo '错误：虚拟机工作空间尚未编译。请先在 Mac 执行 ./scripts/build_vm.sh'; \
     exit 2; \
   fi; \
   source '${VM_WORKSPACE}/install/setup.bash'; \
   ros2 launch patrol_robot_bringup simulation_navigation.launch.py \
     patrol_autostart:=true loop:=true use_gazebo:=false start_rviz:=false"
