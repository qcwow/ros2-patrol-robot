#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

"${SCRIPT_DIR}/sync_to_vm.sh"

echo "正在虚拟机内安装工作空间依赖并编译。"
"${SSH[@]}" -t "${VM_TARGET}" \
  "source /opt/ros/jazzy/setup.bash && \
   cd '${VM_WORKSPACE}' && \
   if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then \
     echo 'rosdep 尚未初始化，正在执行 sudo rosdep init'; \
     sudo rosdep init; \
   fi && \
   if grep -q 'raw.githubusercontent.com/ros/rosdistro/master' \
       /etc/ros/rosdep/sources.list.d/20-default.list; then \
     echo '正在将 rosdep 切换到中科大 ROS Distro 镜像'; \
     sudo cp -n /etc/ros/rosdep/sources.list.d/20-default.list \
       /etc/ros/rosdep/sources.list.d/20-default.list.github-backup; \
     sudo sed -i \
       's#raw.githubusercontent.com/ros/rosdistro/master#mirrors.ustc.edu.cn/rosdistro#g' \
       /etc/ros/rosdep/sources.list.d/20-default.list; \
   fi; \
   if grep -q '/releases/fuerte.yaml' \
       /etc/ros/rosdep/sources.list.d/20-default.list; then \
     echo '正在移除与 ROS 2 Jazzy 无关的 Fuerte rosdep 索引'; \
     sudo sed -i '\#/releases/fuerte.yaml#d' \
       /etc/ros/rosdep/sources.list.d/20-default.list; \
   fi; \
   export ROSDISTRO_INDEX_URL='https://mirrors.ustc.edu.cn/rosdistro/index-v4.yaml'; \
   if ! rosdep update; then \
     if [[ -d "\$HOME/.ros/rosdep/sources.cache" ]]; then \
       echo '警告：rosdep 镜像部分超时，将使用刚刚更新的本地缓存继续。'; \
     else \
       echo '错误：rosdep 更新失败，并且没有可用缓存。'; \
       exit 1; \
     fi; \
   fi; \
   # ament_python is a colcon build type exported in package.xml, not a
   # system dependency key in the Jazzy rosdep index. It is already supplied
   # by the ROS installation, so skip it without hiding real dependency errors.
   rosdep install --from-paths src --ignore-src --skip-keys ament_python -y && \
   colcon build --symlink-install --event-handlers console_direct+ && \
   test -f install/setup.bash && \
   source install/setup.bash && \
   ros2 pkg prefix patrol_robot_camera && \
   ros2 pkg prefix patrol_robot_simulator && \
   ros2 pkg prefix patrol_robot_bringup && \
   echo '编译验证通过：相机处理器、轻量仿真器和总启动包均已安装。'"
