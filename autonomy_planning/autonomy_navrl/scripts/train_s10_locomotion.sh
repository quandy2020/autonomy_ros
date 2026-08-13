#!/usr/bin/env bash
# Train S10 low-level velocity-tracking policy in Isaac Lab (RSL-RL), then export TorchScript.
#
# Prerequisites:
#   - Isaac Lab cloned at ${AUTONOMY_ROOT}/IsaacLab (or /workspace/autonomy/IsaacLab)
#   - S10 URDF prepared: bash scripts/navrl.sh setup s10 urdf
#
# Usage:
#   bash scripts/train_s10_locomotion.sh
#   bash scripts/train_s10_locomotion.sh --headless --num_envs 4096 --max_iterations 5000
# Resume (max_iterations = additional iters on top of checkpoint iter):
#   bash scripts/train_s10_locomotion.sh --headless --resume --load_run RUN_DIR --checkpoint model_XXXX.pt --max_iterations 2100
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
AUTONOMY_ROOT="$(cd "${PKG_ROOT}/../../../.." && pwd)"
ISAACLAB_DIR="${ISAACLAB_DIR:-${AUTONOMY_ROOT}/IsaacLab}"
if [[ ! -d "${ISAACLAB_DIR}" && -d "/workspace/isaaclab" ]]; then
  ISAACLAB_DIR="/workspace/isaaclab"
fi
export ISAACLAB_DIR
TASK_ID="Isaac-Velocity-Rough-S10-v0"
RUN_NAME="${S10_LOCO_RUN:-s10_velocity_rough}"
EXPORT_DIR="${PKG_ROOT}/weights/s10"
CKPT_ROOT="${AUTONOMY_ROOT}/checkpoints/s10_locomotion"

if [[ ! -d "${ISAACLAB_DIR}" ]]; then
  echo "error: Isaac Lab not found at ${ISAACLAB_DIR}" >&2
  echo "Clone: git clone https://github.com/isaac-sim/IsaacLab.git ${ISAACLAB_DIR}" >&2
  exit 1
fi

# Register S10 task extension on PYTHONPATH.
export PYTHONPATH="${PKG_ROOT}/isaaclab_ext:${PYTHONPATH:-}"

ISAACLAB_SH="${ISAACLAB_DIR}/isaaclab.sh"
if [[ ! -x "${ISAACLAB_SH}" ]]; then
  echo "error: ${ISAACLAB_SH} not found" >&2
  exit 1
fi

echo "[s10-loco] task=${TASK_ID} run=${RUN_NAME}"
echo "[s10-loco] IsaacLab=${ISAACLAB_DIR}"

# 1) Train with RSL-RL (registers S10 env via train_entry, then Isaac Lab train.py).
"${ISAACLAB_SH}" -p "${PKG_ROOT}/isaaclab_ext/s10_locomotion/train_entry.py" \
  --task "${TASK_ID}" \
  --run_name "${RUN_NAME}" \
  "$@"

# 2) Export latest checkpoint to TorchScript for autonomy_navrl s10_jit.
LOG_DIR=""
for _log_root in "${PKG_ROOT}/logs/rsl_rl" "${ISAACLAB_DIR}/logs/rsl_rl"; do
  _candidate="$(find "${_log_root}" -type d -name "${RUN_NAME}" 2>/dev/null | sort | tail -1)"
  if [[ -n "${_candidate}" ]]; then
    LOG_DIR="${_candidate}"
    break
  fi
done
if [[ -z "${LOG_DIR}" ]]; then
  echo "warning: training log dir not found under ${PKG_ROOT}/logs or ${ISAACLAB_DIR}/logs; skip export" >&2
  exit 0
fi

# Use the newest run subdirectory (timestamp prefix).
RUN_DIR="$(find "${LOG_DIR}" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | sort | tail -1)"
if [[ -z "${RUN_DIR}" ]]; then
  echo "warning: no run subdir under ${LOG_DIR}; skip export" >&2
  exit 0
fi

mkdir -p "${EXPORT_DIR}" "${CKPT_ROOT}/exported"
"${ISAACLAB_SH}" -p "${PKG_ROOT}/isaaclab_ext/s10_locomotion/export_policy.py" \
  --task "${TASK_ID}" \
  --load_run "${RUN_DIR}" \
  --output "${EXPORT_DIR}/policy.pt"

cp -f "${EXPORT_DIR}/policy.pt" "${CKPT_ROOT}/exported/policy.pt"
echo "[OK] S10 JIT policy: ${EXPORT_DIR}/policy.pt"
