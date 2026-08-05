#!/usr/bin/env bash
set -eo pipefail

# ROS 2 Humble setup files may inspect optional variables before defining
# them. Callers enable nounset before sourcing this helper, so suspend it only
# while loading the underlay/overlay environments and restore it afterwards.
set +u

REAL_CAR_SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REAL_CAR_WORKSPACE="$(CDPATH= cd -- "${REAL_CAR_SCRIPT_DIR}/.." && pwd)"
REAL_CAR_USER_BIN="${HOME}/.local/bin"

if [[ -d "${REAL_CAR_USER_BIN}" ]]; then
  export PATH="${REAL_CAR_USER_BIN}:${PATH}"
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "错误：真车需要 ROS 2 Humble，未找到 /opt/ros/humble/setup.bash。"
  exit 2
fi

REAL_CAR_FACTORY_CONFIG="${REAL_CAR_FACTORY_CONFIG:-/home/ubuntu/ros2_ws/.typerc}"
if [[ -f "${REAL_CAR_FACTORY_CONFIG}" ]]; then
  # The vendor launch files require LIDAR_TYPE, DEPTH_CAMERA_TYPE,
  # MACHINE_TYPE, HOST and MASTER from this hardware profile.
  # shellcheck disable=SC1090
  source "${REAL_CAR_FACTORY_CONFIG}" >/dev/null
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

REAL_CAR_FACTORY_SETUP="${REAL_CAR_FACTORY_SETUP:-/home/ubuntu/ros2_ws/install/setup.bash}"
if [[ -f "${REAL_CAR_FACTORY_SETUP}" \
      && "${REAL_CAR_FACTORY_SETUP}" != "${REAL_CAR_WORKSPACE}/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "${REAL_CAR_FACTORY_SETUP}"
fi

# The factory underlay may prepend an independently built RTAB-Map overlay.
# That copy is linked against the vendor's old librealsense ABI and prevents
# the supported Humble packages in /opt/ros/humble from starting. Keep the
# rest of the factory underlay, but remove only this stale RTAB-Map prefix.
remove_path_entries_containing() {
  local variable_name="$1"
  local rejected_fragment="$2"
  local original_value="${!variable_name:-}"
  local entry
  local filtered_value=''
  local -a path_entries=()

  IFS=':' read -r -a path_entries <<< "${original_value}"
  for entry in "${path_entries[@]}"; do
    if [[ -n "${entry}" && "${entry}" != *"${rejected_fragment}"* ]]; then
      if [[ -n "${filtered_value}" ]]; then
        filtered_value+=":"
      fi
      filtered_value+="${entry}"
    fi
  done
  printf -v "${variable_name}" '%s' "${filtered_value}"
  export "${variable_name}"
}

REAL_CAR_STALE_RTABMAP_PREFIX='/home/ubuntu/third_party/rtabmap_ws/install'
for path_variable in \
    AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH \
    PATH PYTHONPATH PKG_CONFIG_PATH; do
  remove_path_entries_containing \
    "${path_variable}" "${REAL_CAR_STALE_RTABMAP_PREFIX}"
done

if [[ -f "${REAL_CAR_WORKSPACE}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${REAL_CAR_WORKSPACE}/install/setup.bash"
fi

# A colcon overlay caches its parent prefixes when it is built, so loading the
# project setup may restore the stale RTAB-Map paths. Sanitize once more while
# preserving every other vendor dependency.
for path_variable in \
    AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH \
    PATH PYTHONPATH PKG_CONFIG_PATH; do
  remove_path_entries_containing \
    "${path_variable}" "${REAL_CAR_STALE_RTABMAP_PREFIX}"
done

set -u

require_real_car_packages() {
  local package_name
  for package_name in controller peripherals slam; do
    if ! ros2 pkg prefix "${package_name}" >/dev/null 2>&1; then
      echo "错误：未找到厂家包 ${package_name}。请在厂家 ros2_ws 中构建本工作区或先 source 厂家 underlay。"
      exit 3
    fi
  done
}

require_rtabmap_packages() {
  local package_name
  for package_name in rtabmap_slam rtabmap_sync rtabmap_util; do
    if ! ros2 pkg prefix "${package_name}" >/dev/null 2>&1; then
      echo "错误：未找到 ${package_name}。请先安装：sudo apt install ros-humble-rtabmap-ros"
      exit 7
    fi
  done
}

acquire_real_car_stack_guard() {
  local running_stacks
  local lock_path="${REAL_CAR_STACK_LOCK:-/tmp/patrol_robot_real_car_stack.lock}"

  running_stacks="$(
    ps -eo pid=,ppid=,etime=,comm=,args= \
      | awk '
        $5 ~ /\/(python[0-9.]*|Python|ros2)$/ &&
        /\/opt\/ros\/humble\/bin\/ros2 launch patrol_robot_bringup real_car_(mapping|navigation)\.launch\.py/ {
          print
        }
      '
  )"
  if [[ -n "${running_stacks}" ]]; then
    echo "错误：检测到真车建图或导航栈已经运行，拒绝重复启动同名 ROS 节点："
    printf '%s\n' "${running_stacks}"
    echo "请先在原终端按 Ctrl-C，并确认旧进程退出后再启动。"
    exit 10
  fi

  if command -v flock >/dev/null 2>&1; then
    exec 9>"${lock_path}"
    if ! flock -n 9; then
      echo "错误：另一真车建图或导航启动正在进行，拒绝并发启动。"
      exit 10
    fi
  fi
}
