#!/bin/bash
# Launch autonomy_internnav training (NavDP and future baselines).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

NAME=navdp_train
MODEL=navdp
NUM_GPUS="${NUM_GPUS:-1}"
CUDA_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"

while [[ $# -gt 0 ]]; do
  case $1 in
    --name)
      NAME="$2"
      shift 2
      ;;
    --model)
      MODEL="$2"
      shift 2
      ;;
    --gpus)
      NUM_GPUS="$2"
      shift 2
      ;;
    --cuda-devices)
      CUDA_DEVICES="$2"
      shift 2
      ;;
    *)
      echo "Unknown parameter: $1"
      echo "Usage: $0 [--model MODEL] [--name NAME] [--gpus N] [--cuda-devices 0,1,...]"
      echo "  MODEL: navdp (default) | logoplanner | viplanner | vint | nomad"
      exit 1
      ;;
  esac
done

export CUDA_VISIBLE_DEVICES="${CUDA_DEVICES}"
export TORCH_SHOW_CPP_STACKTRACES=1
export TORCH_CPP_LOG_LEVEL=INFO
export NCCL_DEBUG=INFO

cd "${PKG_ROOT}"
export PYTHONPATH="${PKG_ROOT}:${PYTHONPATH:-}"

echo "Starting training: model=${MODEL}, name=${NAME}, gpus=${NUM_GPUS}, CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES}"

TRAIN_SCRIPT="scripts/train/train_internnav.py"
ARGS=(--model "${MODEL}" --name "${NAME}")

if [[ "${NUM_GPUS}" -gt 1 ]]; then
  torchrun \
    --nproc_per_node="${NUM_GPUS}" \
    --nnodes=1 \
    --node_rank=0 \
    --master_addr=localhost \
    --master_port=12345 \
    "${TRAIN_SCRIPT}" \
    "${ARGS[@]}"
else
  python "${TRAIN_SCRIPT}" "${ARGS[@]}"
fi
