#!/usr/bin/env bash
# Cursor hook: delete autonomy_internnav __pycache__ after Python verification commands.
set -euo pipefail

HOOK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${HOOK_DIR}/../.." && pwd)"
CLEAN_SCRIPT="${PKG_ROOT}/scripts/clean_pycache.sh"

input="$(cat)"
command="$(
  printf '%s' "${input}" | "${PYTHON:-python3}" -c \
    'import json,sys; print(json.load(sys.stdin).get("command",""))' \
    2>/dev/null || true
)"

if [[ -z "${command}" ]]; then
  exit 0
fi

if [[ "${command}" == *autonomy_internnav* ]] && [[ "${command}" =~ (py_compile|verify_package\.sh|pytest|python3?[[:space:]]+-c) ]]; then
  if [[ -x "${CLEAN_SCRIPT}" ]]; then
    "${CLEAN_SCRIPT}" >/dev/null 2>&1 || true
  fi
fi

exit 0
