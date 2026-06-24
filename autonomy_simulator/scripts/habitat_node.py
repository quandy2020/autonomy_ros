#!/opt/venv/bin/python3
#
# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

"""ROS 2 entry point for the Habitat bridge node."""

from __future__ import annotations

import os
import sys


def _prepend(path: str) -> None:
    if path and os.path.isdir(path) and path not in sys.path:
        sys.path.insert(0, path)


def _workspace_install_prefix() -> str | None:
    path = os.path.dirname(os.path.abspath(__file__))
    for _ in range(10):
        install = os.path.abspath(os.path.join(path, 'install'))
        if os.path.isdir(os.path.join(install, 'autonomy_msgs')):
            return install
        parent = os.path.dirname(path)
        if parent == path:
            break
        path = parent
    return None


def _bootstrap_pythonpath() -> None:
    """venv Python does not inherit colcon overlays; load ROS msg packages."""
    for entry in os.environ.get('PYTHONPATH', '').split(os.pathsep):
        _prepend(entry)

    py_ver = f'python{sys.version_info.major}.{sys.version_info.minor}'
    prefixes: list[str] = []
    for key in ('COLCON_PREFIX_PATH', 'AMENT_PREFIX_PATH'):
        prefixes.extend(p for p in os.environ.get(key, '').split(os.pathsep) if p)

    install = _workspace_install_prefix()
    if install:
        prefixes.append(install)

    for prefix in prefixes:
        if not os.path.isdir(prefix):
            continue
        for pkg in os.listdir(prefix):
            for rel in (f'local/lib/{py_ver}/dist-packages', f'lib/{py_ver}/site-packages'):
                _prepend(os.path.join(prefix, pkg, rel))


_bootstrap_pythonpath()

import rclpy

from habitat.node import BridgeNode


def main(argv: list[str] | None = None) -> None:
    rclpy.init(args=argv)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
