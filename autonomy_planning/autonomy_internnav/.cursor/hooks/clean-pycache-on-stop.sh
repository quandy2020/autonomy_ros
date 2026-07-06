#!/usr/bin/env bash
# Cursor hook: always remove autonomy_internnav __pycache__ when the agent stops.
set -euo pipefail

HOOK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${HOOK_DIR}/../.." && pwd)"
CLEAN_SCRIPT="${PKG_ROOT}/scripts/clean_pycache.sh"

if [[ -x "${CLEAN_SCRIPT}" ]]; then
  "${CLEAN_SCRIPT}" >/dev/null 2>&1 || true
fi

exit 0
