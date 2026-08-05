#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_real_car.sh"

require_real_car_packages
MAPPING_BACKEND="${REAL_CAR_MAPPING_BACKEND:-rtabmap}"
if [[ "${MAPPING_BACKEND}" != "rtabmap" \
      && "${MAPPING_BACKEND}" != "slam_toolbox" ]]; then
  echo "错误：REAL_CAR_MAPPING_BACKEND 只能是 rtabmap 或 slam_toolbox。"
  exit 8
fi
require_rtabmap_packages
if ! ros2 pkg prefix patrol_robot_bringup >/dev/null 2>&1; then
  echo "错误：尚未构建本工作区，请先执行 scripts/build_real_car.sh。"
  exit 4
fi

acquire_real_car_stack_guard

echo "启动真车安全建图（${MAPPING_BACKEND}）、导航和网页车辆网关：网页调速范围 0.05～0.15 m/s。"
echo "启动后先架空车轮验证；雷达或手柄失联会输出零速。"
exec ros2 launch patrol_robot_bringup real_car_mapping.launch.py \
  start_hardware:="${REAL_CAR_START_HARDWARE:-true}" \
  start_web_bridge:="${REAL_CAR_START_WEB_BRIDGE:-true}" \
  start_rviz:="${REAL_CAR_START_RVIZ:-false}" \
  start_local_grid:="${REAL_CAR_START_LOCAL_GRID:-false}" \
  start_rotation_diagnostics:="${REAL_CAR_START_ROTATION_DIAGNOSTICS:-false}" \
  web_port:="${REAL_CAR_WEB_PORT:-8765}" \
  start_joystick:="${REAL_CAR_START_JOYSTICK:-false}" \
  mapping_backend:="${MAPPING_BACKEND}" \
  reset_rtabmap_database:="${REAL_CAR_RESET_RTABMAP_DATABASE:-true}" \
  rtabmap_database_path:="${REAL_CAR_RTABMAP_DATABASE:-~/.ros/rtabmap.db}" \
  rtabmap_params_file:="${REAL_CAR_RTABMAP_PARAMS_FILE:-${REAL_CAR_WORKSPACE}/src/patrol_robot_navigation/config/rtabmap_real_car.yaml}" \
  rtabmap_imu_topic:="${REAL_CAR_RTABMAP_IMU_TOPIC:-/rtabmap/imu_disabled}" \
  rtabmap_rgb_topic:="${REAL_CAR_RGB_TOPIC:-/depth_cam/rgb0/image_raw}" \
  rtabmap_depth_topic:="${REAL_CAR_DEPTH_TOPIC:-/depth_cam/depth0/image_raw}" \
  rtabmap_camera_info_topic:="${REAL_CAR_CAMERA_INFO_TOPIC:-/depth_cam/rgb0/camera_info}" \
  max_linear_speed:="${REAL_CAR_MAPPING_LINEAR_LIMIT:-0.15}" \
  max_angular_speed:="${REAL_CAR_MAPPING_ANGULAR_LIMIT:-0.45}"
