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

"""Lightweight map→odom→base_footprint TF publisher for Habitat Session load."""

from __future__ import annotations

import os
import sys


def _prepend(path: str) -> None:
    if path and os.path.isdir(path) and path not in sys.path:
        sys.path.insert(0, path)


def _bootstrap_pythonpath() -> None:
    for entry in os.environ.get('PYTHONPATH', '').split(os.pathsep):
        _prepend(entry)
    py_ver = f'python{sys.version_info.major}.{sys.version_info.minor}'
    for key in ('COLCON_PREFIX_PATH', 'AMENT_PREFIX_PATH'):
        for prefix in os.environ.get(key, '').split(os.pathsep):
            if prefix:
                for pkg in os.listdir(prefix):
                    _prepend(os.path.join(prefix, pkg, f'local/lib/{py_ver}/dist-packages'))
                    _prepend(os.path.join(prefix, pkg, f'lib/{py_ver}/site-packages'))


_bootstrap_pythonpath()

import rclpy
from rclpy.node import Node

from habitat.config import load
from habitat.odom import OdomPublisher


class OdomTfNode(Node):
    """Publish odom TF while habitat_node blocks on GPU Session load."""

    def __init__(self) -> None:
        super().__init__('habitat_odom_tf')
        cfg = load(self)
        self._odom = OdomPublisher(self, cfg)
        self.create_timer(0.1, self._on_timer)

    def _on_timer(self) -> None:
        self._odom.hold_tf_alive(self.get_clock().now())


def main(argv: list[str] | None = None) -> None:
    rclpy.init(args=argv)
    node = OdomTfNode()
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
