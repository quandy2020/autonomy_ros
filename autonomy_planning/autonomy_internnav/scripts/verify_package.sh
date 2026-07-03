#!/usr/bin/env bash
# Syntax-check autonomy_internnav; always remove __pycache__ on exit.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MODULE_ROOT="${PKG_ROOT}/autonomy_internnav"
CLEAN_SCRIPT="${SCRIPT_DIR}/clean_pycache.sh"
PYTHON="${PYTHON:-python3}"

cleanup() {
  if [[ -x "${CLEAN_SCRIPT}" ]]; then
    "${CLEAN_SCRIPT}" || true
  fi
}
trap cleanup EXIT

export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="${PKG_ROOT}:${PYTHONPATH:-}"

if [[ ! -d "${MODULE_ROOT}" ]]; then
  echo "error: module root not found: ${MODULE_ROOT}" >&2
  exit 1
fi

if [[ -x "${CLEAN_SCRIPT}" ]]; then
  "${CLEAN_SCRIPT}" || true
fi

echo "verify_package: syntax check (${PYTHON}, no bytecode write) ..."
count="$(
  MODULE_ROOT="${MODULE_ROOT}" "${PYTHON}" - <<'PY'
import ast
import os
import sys
from pathlib import Path

module_root = Path(os.environ["MODULE_ROOT"])
skip = {"InternNav"}
errors = 0
checked = 0
for path in sorted(module_root.rglob("*.py")):
    if skip.intersection(path.parts):
        continue
    checked += 1
    try:
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    except SyntaxError as exc:
        errors += 1
        print(f"syntax error: {path}: {exc}", file=sys.stderr)
if errors:
    sys.exit(1)
print(checked)
PY
)"

echo "verify_package: ok (${count} files)"
