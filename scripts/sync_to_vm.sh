#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

echo "正在同步到 ${VM_TARGET}:${VM_WORKSPACE}"
"${SSH[@]}" "${VM_TARGET}" "mkdir -p '${VM_WORKSPACE}'"

RSYNC_SSH=(ssh -p "${VM_PORT}")
if [[ -n "${VM_SSH_KEY}" ]]; then
  RSYNC_SSH+=( -i "${VM_SSH_KEY}" -o IdentitiesOnly=yes )
fi

printf -v RSYNC_SSH_COMMAND '%q ' "${RSYNC_SSH[@]}"

rsync -az --delete --human-readable \
  --exclude '.git/' \
  --exclude '.vm.env' \
  --exclude '.DS_Store' \
  --exclude 'build/' \
  --exclude 'install/' \
  --exclude 'log/' \
  --exclude 'node_modules/' \
  --exclude 'dist/' \
  --exclude '.next/' \
  --exclude '.vinext/' \
  --exclude '.wrangler/' \
  --exclude '__pycache__/' \
  -e "${RSYNC_SSH_COMMAND% }" \
  "${PROJECT_ROOT}/" \
  "${VM_TARGET}:${VM_WORKSPACE}/"

echo "同步完成。"
