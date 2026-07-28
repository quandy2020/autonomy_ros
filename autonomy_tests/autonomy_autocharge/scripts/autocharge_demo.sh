#!/usr/bin/env bash
# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0
#
# Run autocharge demo without manually sourcing the workspace overlay.
# Usage (from anywhere inside the workspace):
#   bash src/autonomy_ros/autonomy_tests/autonomy_autocharge/scripts/autocharge_demo.sh
# Or after build:
#   bash install/autonomy_autocharge/share/autonomy_autocharge/scripts/autocharge_demo.sh

set -eo pipefail

_script="${BASH_SOURCE[0]}"
while [ -L "$_script" ]; do
  _dir="$(cd "$(dirname "$_script")" && pwd)"
  _script="$(readlink "$_script")"
  [[ "$_script" != /* ]] && _script="$_dir/$_script"
done
_script_dir="$(cd "$(dirname "$_script")" && pwd)"

# Resolve workspace root: .../autonomy (parent of install/ or src/)
if [ -f "$_script_dir/../../../../../install/setup.bash" ]; then
  # src/autonomy_ros/autonomy_tests/autonomy_autocharge/scripts -> workspace
  _ws="$(cd "$_script_dir/../../../../../" && pwd)"
elif [ -f "$_script_dir/../../../../setup.bash" ]; then
  # install/autonomy_autocharge/share/autonomy_autocharge/scripts -> workspace
  _ws="$(cd "$_script_dir/../../../../.." && pwd)"
else
  echo "error: cannot find workspace install/setup.bash" >&2
  exit 1
fi

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash
fi

# shellcheck source=/dev/null
source "$_ws/install/setup.bash"

exec ros2 launch autonomy_autocharge autocharge_demo.launch.py "$@"
