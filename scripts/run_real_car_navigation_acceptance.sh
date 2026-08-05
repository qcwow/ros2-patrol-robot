#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_real_car.sh"

EXPECTED_CONFIRMATION='SITE_SAFE_ROUTE_AND_INITIAL_POSE_VERIFIED'
if [[ "${REAL_CAR_ACCEPTANCE_CONFIRMATION:-}" != "${EXPECTED_CONFIRMATION}" ]]; then
  echo "错误：导航验收可能驱动车辆，拒绝启动。"
  echo "确认现场有人、硬件急停可用、路线坐标已核对且 AMCL 初始位姿已确认后，设置："
  echo "REAL_CAR_ACCEPTANCE_CONFIRMATION=${EXPECTED_CONFIRMATION}"
  exit 20
fi

SCENARIO_FILE="${1:-${REAL_CAR_ACCEPTANCE_SCENARIOS:-${REAL_CAR_WORKSPACE}/src/patrol_robot_patrol/config/navigation_acceptance_real_car.yaml}}"
SCENARIO_ID="${2:-${REAL_CAR_ACCEPTANCE_SCENARIO_ID:-normal_route_low_speed}}"
if [[ ! -f "${SCENARIO_FILE}" ]]; then
  echo "错误：验收场景文件不存在：${SCENARIO_FILE}"
  exit 21
fi

for required_node in /amcl /base_command_watchdog /navigation_health /patrol_manager; do
  if ! ros2 node list 2>/dev/null | grep -qx "${required_node}"; then
    echo "错误：缺少 ${required_node}，请先启动真车静态地图导航并设置初始位姿。"
    exit 22
  fi
done

echo "启动真车导航验收：${SCENARIO_ID}"
echo "执行器会等待健康门稳定、核对最终看门狗限速，并在异常时调用巡检停车。"
exec ros2 run patrol_robot_patrol navigation_acceptance_runner --ros-args \
  -p scenario_file:="${SCENARIO_FILE}" \
  -p scenario_id:="${SCENARIO_ID}" \
  -p armed:=true \
  -p output_directory:="${REAL_CAR_ACCEPTANCE_OUTPUT_DIR:-~/.ros/patrol_robot/navigation_acceptance}"
