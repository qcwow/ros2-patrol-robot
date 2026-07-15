#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

ACTION="${1:-}"
case "${ACTION}" in
  start|stop|reset) ;;
  *)
    echo "用法: $0 {start|stop|reset}"
    exit 1
    ;;
esac

"${SSH[@]}" "${VM_TARGET}" \
  "source /opt/ros/jazzy/setup.bash && \
   source '${VM_WORKSPACE}/install/setup.bash' && \
   ros2 service call '/patrol_manager/${ACTION}' std_srvs/srv/Trigger '{}'"

