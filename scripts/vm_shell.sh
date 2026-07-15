#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

"${SSH[@]}" -t "${VM_TARGET}" \
  "cd '${VM_WORKSPACE}' && exec bash -l"

