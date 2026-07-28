#!/bin/bash
# Launch NavDP GRPO fine-tuning (aligned with JdInternNav launch_grpo_train.sh).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
TRAIN_PYTHON="$(bash "${PKG_ROOT}/scripts/_train_python.sh")"
TRAIN_BIN="$(dirname "${TRAIN_PYTHON}")"
TORCHRUN="${TRAIN_BIN}/torchrun"
if [[ ! -x "${TORCHRUN}" ]]; then
  TORCHRUN="${TRAIN_PYTHON} -m torch.distributed.run"
fi

NUM_GPUS="${NUM_GPUS:-1}"
CUDA_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"
DATASET_TYPE="${DATASET_TYPE:-lerobot_v3}"
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
  case $1 in
    --gpus)
      NUM_GPUS="$2"
      shift 2
      ;;
    --cuda-devices)
      CUDA_DEVICES="$2"
      shift 2
      ;;
    --dataset-type)
      DATASET_TYPE="$2"
      shift 2
      ;;
    *)
      EXTRA_ARGS+=("$1")
      shift
      ;;
  esac
done

export CUDA_VISIBLE_DEVICES="${CUDA_DEVICES}"
export DATASET_TYPE
export PYTHONPATH="${PKG_ROOT}:${PYTHONPATH:-}"

cd "${PKG_ROOT}"
TRAIN_SCRIPT="scripts/train/train_grpo.py"

echo "Starting GRPO: gpus=${NUM_GPUS}, dataset_type=${DATASET_TYPE}, python=${TRAIN_PYTHON}"

if [[ "${NUM_GPUS}" -gt 1 ]]; then
  ${TORCHRUN} \
    --nproc_per_node="${NUM_GPUS}" \
    --nnodes=1 \
    --node_rank=0 \
    --master_addr=localhost \
    --master_port=12346 \
    "${TRAIN_SCRIPT}" \
    --dataset-type "${DATASET_TYPE}" \
    "${EXTRA_ARGS[@]}"
else
  "${TRAIN_PYTHON}" "${TRAIN_SCRIPT}" --dataset-type "${DATASET_TYPE}" "${EXTRA_ARGS[@]}"
fi
