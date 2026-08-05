#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(CDPATH= cd -- "${SCRIPT_DIR}/.." && pwd)"

if [[ -z "${DISPLAY:-}" ]]; then
  echo "错误：当前终端没有图形桌面环境。"
  echo "请在 Ubuntu 虚拟机桌面中打开终端后运行本脚本，不要通过 SSH 运行。"
  exit 2
fi

if pgrep -x rviz2 >/dev/null 2>&1 || \
   pgrep -f 'ros2 launch patrol_robot_bringup simulation_(navigation|mapping)\.launch\.py' \
     >/dev/null 2>&1; then
  echo "错误：检测到上一轮 RViz 或巡航仿真仍在运行。"
  echo "请回到原来的终端按 Ctrl+C，等待窗口关闭后再启动。"
  exit 3
fi

if [[ ! -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  echo "错误：工作空间尚未编译，请先在 Mac 执行 ./scripts/build_vm.sh。"
  exit 2
fi

export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1
export RCUTILS_COLORIZED_OUTPUT=1

source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WORKSPACE_ROOT}/install/setup.bash"

missing_apt_packages=()
if ! command -v xacro >/dev/null 2>&1; then
  missing_apt_packages+=(ros-humble-xacro)
fi
if ! ros2 pkg prefix nav2_controller >/dev/null 2>&1; then
  missing_apt_packages+=(ros-humble-navigation2 ros-humble-nav2-bringup)
fi
if (( ${#missing_apt_packages[@]} > 0 )); then
  echo "错误：Ubuntu 尚未安装完整的 Humble 运行依赖。"
  echo "请执行：sudo apt update && sudo apt install -y ${missing_apt_packages[*]}"
  exit 2
fi

if ! ros2 pkg prefix patrol_robot_simulator >/dev/null 2>&1; then
  echo "错误：patrol_robot_simulator 尚未安装。"
  echo "请回到 Mac 项目目录执行 ./scripts/build_vm.sh，等待6个功能包全部完成。"
  exit 2
fi

echo "使用无 OpenGL 依赖的轻量二维仿真器，并打开 RViz 导航界面。"
exec ros2 launch patrol_robot_bringup simulation_navigation.launch.py \
  patrol_autostart:=true loop:=true use_gazebo:=false start_rviz:=true
