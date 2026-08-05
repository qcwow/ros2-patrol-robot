#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

echo "正在读取 Ubuntu 自主建图状态（只读，不会停止仿真）。"
"${SSH[@]}" "${VM_TARGET}" \
  "source /opt/ros/humble/setup.bash; \
   source '${VM_WORKSPACE}/install/setup.bash'; \
   echo; \
   echo '核心节点'; \
   ros2 node list | grep -E \
     '/(async_slam_toolbox_node|bt_navigator|color_octomap_server_node|controller_server|frontier_explorer|octomap_server|planner_server|rgbd_processor|slam_toolbox)$' \
     | sort || true; \
   echo; \
   echo '自主探索状态'; \
   timeout 10s ros2 topic echo /frontier_explorer/status \
     --once --full-length; \
   echo; \
   echo '导航安全参数'; \
   ros2 param get /planner_server GridBased.allow_unknown; \
   ros2 param get /controller_server FollowPath.desired_linear_vel; \
   echo; \
   echo '数据源连接'; \
   ros2 topic info /camera/points/mapping; \
   ros2 topic info /occupied_cells_vis_array"
