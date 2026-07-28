#!/usr/bin/env bash
# Install NavDP training Python deps.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REQ_INF="${PKG_ROOT}/requirements.txt"
REQ_TRAIN="${PKG_ROOT}/requirements-train.txt"
PYTHON="$(bash "${SCRIPT_DIR}/_train_python.sh")"

if ! command -v "${PYTHON}" &>/dev/null; then
  echo "error: Python not found: ${PYTHON}" >&2
  exit 1
fi

echo "Using training Python: ${PYTHON}"

if "${PYTHON}" -c "import torch" >/dev/null 2>&1; then
  echo "torch already available; skipping requirements.txt (avoid redundant torch download)"
else
  echo "Installing inference deps ..."
  "${PYTHON}" -m pip install -r "${REQ_INF}"
fi

echo "Installing training deps ..."
"${PYTHON}" -m pip install -r "${REQ_TRAIN}"

echo "Verifying imports ..."
"${PYTHON}" -c "
import torch
import transformers
import tyro
import tensorboard
from autonomy_internnav.train.navdp_model import NavDPNet
print('torch', torch.__version__)
print('transformers', transformers.__version__)
print('tensorboard', tensorboard.__version__)
print('NavDPNet', NavDPNet)
"

echo "Done."
