#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_real_car.sh"

require_real_car_packages
require_rtabmap_packages
if ! ros2 pkg prefix patrol_robot_bringup >/dev/null 2>&1; then
  echo "错误：尚未构建本工作区，请先执行 scripts/build_real_car.sh。"
  exit 4
fi

MAP_YAML="${1:-${REAL_CAR_MAP_YAML:-}}"
if [[ -z "${MAP_YAML}" || ! -f "${MAP_YAML}" ]]; then
  echo "错误：必须提供真车地图 YAML，禁止默认使用仿真地图。"
  echo "用法：scripts/run_real_car_navigation.sh /绝对路径/map.yaml"
  exit 5
fi

WAYPOINTS="${REAL_CAR_WAYPOINTS_YAML:-${REAL_CAR_WORKSPACE}/src/patrol_robot_patrol/config/waypoints_real_car_template.yaml}"
if [[ ! -f "${WAYPOINTS}" ]]; then
  echo "错误：巡检点文件不存在：${WAYPOINTS}"
  exit 6
fi

if [[ "${REAL_CAR_ALLOW_UNMANIFESTED_MAP:-false}" == "true" ]]; then
  echo "警告：正在使用未验证来源的地图；仅允许迁移旧地图时临时使用。"
  ros2 run patrol_robot_patrol map_artifact_validator "${MAP_YAML}"
else
  ros2 run patrol_robot_patrol map_artifact_validator "${MAP_YAML}" \
    --require-manifest-profile real_car
fi

acquire_real_car_stack_guard

echo "启动真车 AMCL/Nav2；巡检不会自动开始。"
echo "必须先在 RViz 使用 2D Pose Estimate 确认车辆实际初始位姿。"
exec ros2 launch patrol_robot_bringup real_car_navigation.launch.py \
  map:="${MAP_YAML}" \
  waypoints:="${WAYPOINTS}" \
  patrol_autostart:=false \
  start_hardware:="${REAL_CAR_START_HARDWARE:-true}" \
  start_rviz:="${REAL_CAR_START_RVIZ:-true}" \
  start_web_bridge:="${REAL_CAR_START_WEB_BRIDGE:-true}" \
  start_rotation_diagnostics:="${REAL_CAR_START_ROTATION_DIAGNOSTICS:-false}" \
  max_linear_speed:="${REAL_CAR_NAV_LINEAR_LIMIT:-0.15}" \
  max_angular_speed:="${REAL_CAR_NAV_ANGULAR_LIMIT:-0.45}"
