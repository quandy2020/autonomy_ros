#!/usr/bin/env bash
# Remove stale pedsim_msgs build/install trees before --symlink-install.
# Fixes: "failed to create symbolic link ... because existing path cannot be removed: Is a directory"
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
rm -rf "${ROOT}/build/pedsim_msgs" "${ROOT}/install/pedsim_msgs"
echo "Removed build/pedsim_msgs and install/pedsim_msgs under ${ROOT}"
