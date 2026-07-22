#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(CDPATH= cd -- "${SCRIPT_DIR}/.." && pwd)"

if [[ -z "${DISPLAY:-}" ]]; then
  echo "错误：当前终端没有图形桌面环境。"
  echo "请在 Ubuntu 虚拟机桌面中打开终端后运行本脚本。"
  exit 2
fi

if pgrep -f 'ros2 launch patrol_robot_bringup simulation_navigation.launch.py' >/dev/null 2>&1; then
  echo "错误：检测到上一轮导航仿真仍在运行，请先在原终端按 Ctrl+C。"
  exit 3
fi

if [[ ! -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  echo "错误：工作空间尚未编译，请先在 Mac 执行 ./scripts/build_vm.sh。"
  exit 2
fi

export QT_X11_NO_MITSHM=1
export RCUTILS_COLORIZED_OUTPUT=1

source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "${WORKSPACE_ROOT}/install/setup.bash"

if ! ros2 pkg prefix ros_gz_sim >/dev/null 2>&1; then
  echo "错误：Gazebo ROS 组件未安装，请重新执行 ./scripts/setup_vm.sh。"
  exit 2
fi

# `gz sim` starts separate server and GUI processes. On some VMware images the
# Ruby launcher exits before those children, leaving them to consume CPU after
# Ctrl+C. Remember pre-existing Gazebo processes and clean up only the ones
# created by this script.
GAZEBO_PIDS_BEFORE=" $( (pgrep -f '^gz sim' 2>/dev/null || true) | tr '\n' ' ') "
cleanup_gazebo_children() {
  local new_pids=()
  local pid
  while read -r pid; do
    [[ -z "${pid}" ]] && continue
    case "${GAZEBO_PIDS_BEFORE}" in
      *" ${pid} "*) ;;
      *) new_pids+=("${pid}") ;;
    esac
  done < <(pgrep -f '^gz sim' 2>/dev/null || true)
  if ((${#new_pids[@]} == 0)); then
    return
  fi
  kill -TERM "${new_pids[@]}" 2>/dev/null || true
  sleep 2
  for pid in "${new_pids[@]}"; do
    kill -0 "${pid}" 2>/dev/null && kill -KILL "${pid}" 2>/dev/null || true
  done
}
trap cleanup_gazebo_children EXIT

echo "启动 Gazebo 3D 场景、RGB-D 处理、Nav2 和自动循环巡检。"
echo "为给网页控制台留出资源，默认不启动 RViz；需要时追加 start_rviz:=true。"
echo "相机彩色点云发布到 /camera/points/filtered；可在网页切换雷达、视觉或融合避障。"
ros2 launch patrol_robot_bringup simulation_navigation.launch.py \
  patrol_autostart:=true loop:=true use_gazebo:=true \
  headless:=false start_rviz:=false "$@"
