#!/bin/bash
# Offline NavDP bag replay (LeRobot v3 format).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

export PYTHONPATH="${PKG_ROOT}:${PYTHONPATH:-}"
if [[ -d "${PKG_ROOT}/InternNav/third_party/diffusion-policy" ]]; then
  export PYTHONPATH="${PKG_ROOT}/InternNav/third_party/diffusion-policy:${PYTHONPATH}"
fi

exec python "${PKG_ROOT}/autonomy_internnav/replay/scripts/run_navdp_replay.py" "$@"
