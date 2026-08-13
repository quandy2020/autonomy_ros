#!/usr/bin/env bash
# Deprecated alias → navrl.sh train … -w
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
target="${1:-s10}"
if [[ "${target}" == "go2w" || "${target}" == "s10" ]]; then
  shift || true
else
  target="s10"
fi
exec bash "${SCRIPT_DIR}/navrl.sh" train "${target}" -w "$@"
