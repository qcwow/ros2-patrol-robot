#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(CDPATH= cd -- "${SCRIPT_DIR}/.." && pwd)"
ENV_FILE="${PROJECT_ROOT}/.vm.env"

if [[ ! -f "${ENV_FILE}" ]]; then
  echo "缺少 ${ENV_FILE}"
  echo "请先执行: cp .vm.env.example .vm.env，然后填写虚拟机信息。"
  exit 1
fi

# shellcheck disable=SC1090
source "${ENV_FILE}"

: "${VM_USER:?请在 .vm.env 中设置 VM_USER}"
: "${VM_HOST:?请在 .vm.env 中设置 VM_HOST}"
: "${VM_PORT:=22}"
: "${VM_WORKSPACE:?请在 .vm.env 中设置 VM_WORKSPACE}"
: "${VM_DISPLAY:=:0}"

if [[ ! "${VM_PORT}" =~ ^[0-9]+$ ]]; then
  echo "VM_PORT 必须是数字。"
  exit 1
fi

if [[ ! "${VM_WORKSPACE}" =~ ^/[A-Za-z0-9._/-]+$ ]]; then
  echo "VM_WORKSPACE 必须是不含空格的绝对 Linux 路径。"
  exit 1
fi

VM_TARGET="${VM_USER}@${VM_HOST}"
SSH=(ssh -p "${VM_PORT}")

