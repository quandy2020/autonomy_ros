#!/usr/bin/env bash
# 零参数启动：默认读 configs/default.yaml
set -euo pipefail
cd "$(dirname "$0")/../../.."
export PYTHONPATH="$(pwd)/internnav/replay:$(pwd)/third_party/diffusion-policy:${PYTHONPATH:-}"
exec python internnav/replay/scripts/run_navdp_replay.py "$@"