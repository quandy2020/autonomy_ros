#!/usr/bin/env bash
# Install NavDP Python deps for the ROS 2 internnav_node (/usr/bin/python3).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REQ="${PKG_ROOT}/requirements.txt"
# ROS console_scripts use #!/usr/bin/python3 — install into that interpreter.
PYTHON="${PYTHON:-/usr/bin/python3}"

if [[ ! -x "${PYTHON}" ]]; then
  echo "error: Python not found: ${PYTHON}" >&2
  exit 1
fi

if [[ ! -f "${REQ}" ]]; then
  echo "error: requirements not found: ${REQ}" >&2
  exit 1
fi

export PYTHONPATH=

echo "Installing NavDP deps with ${PYTHON} ..."
"${PYTHON}" -m pip install --upgrade 'pip<26' 'setuptools>=30.3.0,<80'
# Ubuntu distutils sympy blocks upgrade unless replaced alone.
"${PYTHON}" -m pip install --ignore-installed --no-deps 'sympy>=1.13'
"${PYTHON}" -m pip install -r "${REQ}"

echo "Checking dependency conflicts ..."
if ! "${PYTHON}" -m pip check; then
  echo "warning: pip check reported conflicts (see above)" >&2
fi

echo "Verifying torch ..."
"${PYTHON}" -c "import torch; print('torch', torch.__version__, 'cuda', torch.cuda.is_available())"
