#!/usr/bin/env bash
# Resolve Python for NavDP training (Docker /opt/venv or explicit override).
set -euo pipefail

if [[ -n "${AUTONOMY_TRAIN_PYTHON:-}" ]] && [[ -x "${AUTONOMY_TRAIN_PYTHON}" ]]; then
  echo "${AUTONOMY_TRAIN_PYTHON}"
  exit 0
fi

if [[ -n "${PYTHON:-}" ]] && [[ -x "${PYTHON}" ]]; then
  echo "${PYTHON}"
  exit 0
fi

if [[ -x /opt/venv/bin/python ]] \
    && /opt/venv/bin/python -c "import torch" >/dev/null 2>&1; then
  echo /opt/venv/bin/python
  exit 0
fi

echo python3
