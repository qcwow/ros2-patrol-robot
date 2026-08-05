#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(CDPATH= cd -- "${SCRIPT_DIR}/.." && pwd)"

if [[ -z "${DISPLAY:-}" ]]; then
  echo "错误：当前终端没有图形桌面环境。"
  echo "请在 Ubuntu 虚拟机桌面中打开终端后运行本脚本。"
  exit 2
fi

if pgrep -f 'ros2 launch patrol_robot_bringup simulation_(navigation|mapping)\.launch\.py' \
    >/dev/null 2>&1; then
  echo "错误：检测到上一轮导航或建图仿真仍在运行，请先在原终端按 Ctrl+C。"
  exit 3
fi

if [[ ! -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  echo "错误：工作空间尚未编译，请先在 Mac 执行 ./scripts/build_vm.sh。"
  exit 2
fi

export QT_X11_NO_MITSHM=1
export RCUTILS_COLORIZED_OUTPUT=1

source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WORKSPACE_ROOT}/install/setup.bash"

for required_package in ros_gz_sim octomap_server patrol_robot_camera; do
  if ! ros2 pkg prefix "${required_package}" >/dev/null 2>&1; then
    echo "错误：缺少 ${required_package}，请重新执行 ./scripts/build_vm.sh。"
    exit 2
  fi
done

# Gazebo uses child processes that may outlive the launch process on VMware.
# Track only children created by this run and clean them up on exit.
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

echo "启动 Gazebo、自主前沿探索、激光 SLAM、RGB-D 彩色 OctoMap 和 RViz。"
echo "定位使用轮速里程计 + IMU EKF；体素地图话题为 /octomap_full。"
echo "另开终端运行 teleop 后，沿闭环路线缓慢行驶即可边定位边建立 3D 地图。"
ros2 launch patrol_robot_bringup simulation_mapping.launch.py \
  use_gazebo:=true enable_3d_mapping:=true \
  enable_autonomous_exploration:=true ground_truth_odometry:=false \
  headless:=false start_rviz:=true "$@"
