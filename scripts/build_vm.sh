#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

"${SCRIPT_DIR}/sync_to_vm.sh"

echo "正在虚拟机内编译 Humble 工作空间。"
"${SSH[@]}" -t "${VM_TARGET}" \
  "source /opt/ros/humble/setup.bash && \
   cd '${VM_WORKSPACE}' && \
   if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then \
     echo 'rosdep 尚未初始化，正在执行 sudo rosdep init'; \
     sudo rosdep init; \
   fi && \
   export ROSDISTRO_INDEX_URL='https://mirrors.ustc.edu.cn/rosdistro/index-v4.yaml'; \
   if [[ ! -d "\$HOME/.ros/rosdep/sources.cache" ]]; then \
     echo '错误：rosdep 尚无本地索引，请先运行 ./scripts/setup_vm.sh。'; \
     exit 1; \
   fi; \
   # Dependency installation is intentionally kept in setup_vm.sh because it
   # may request the Ubuntu sudo password. Daily Mac-side builds stay
   # unattended and report missing packages through the build output.
   colcon build --executor sequential --symlink-install \
     --event-handlers console_direct+ \
     --cmake-args -DCMAKE_BUILD_TYPE=Release && \
   test -f install/setup.bash && \
   source install/setup.bash && \
   ros2 pkg prefix patrol_robot_camera && \
   ros2 pkg prefix patrol_robot_simulator && \
   ros2 pkg prefix slam_toolbox && \
   ros2 pkg prefix patrol_robot_bringup && \
   echo '编译验证通过：SLAM Toolbox、Nav2、轻量仿真器和总启动包均已安装。'"
