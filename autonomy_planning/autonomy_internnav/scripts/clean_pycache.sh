#!/usr/bin/env bash
# Remove bytecode caches under autonomy_internnav (source tree only).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MODULE_ROOT="${PKG_ROOT}/autonomy_internnav"

if [[ ! -d "${MODULE_ROOT}" ]]; then
  echo "error: module root not found: ${MODULE_ROOT}" >&2
  exit 1
fi

removed=0
skipped=0
while IFS= read -r -d '' cache_dir; do
  if rm -rf "${cache_dir}" 2>/dev/null; then
    removed=$((removed + 1))
  else
    skipped=$((skipped + 1))
    echo "clean_pycache: skip ${cache_dir} (permission denied; run: sudo rm -rf '${cache_dir}')" >&2
  fi
done < <(find "${MODULE_ROOT}" -type d -name '__pycache__' -print0 2>/dev/null)

find "${MODULE_ROOT}" -type f \( -name '*.pyc' -o -name '*.pyo' \) -delete 2>/dev/null || true

echo "clean_pycache: removed ${removed} __pycache__ dir(s), skipped ${skipped} under ${MODULE_ROOT}"
