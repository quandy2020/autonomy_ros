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
            if not prefix or not os.path.isdir(prefix):
                continue
            for pkg in os.listdir(prefix):
                _prepend(os.path.join(prefix, pkg, f'local/lib/{py_ver}/dist-packages'))
                _prepend(os.path.join(prefix, pkg, f'lib/{py_ver}/site-packages'))


_bootstrap_pythonpath()

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from habitat.config import load
from habitat.odom import OdomPublisher


class OdomTfNode(Node):
    """Publish identity odom TF while habitat_node blocks on GPU Session load.

    Exits once habitat_node starts publishing /odom so the two TFs no longer fight
    (identity vs real pose) and stamped lookups stop seeing stale authorities.
    """

    def __init__(self) -> None:
        super().__init__('habitat_odom_tf')
        cfg = load(self)
        self._odom = OdomPublisher(self, cfg)
        self._timer = self.create_timer(0.1, self._on_timer)
        self._odom_sub = self.create_subscription(
            Odometry, cfg.odom_topic, self._on_habitat_odom, qos_profile_system_default
        )
        self._shutting_down = False
        self._exit_timer = None

    def _on_timer(self) -> None:
        if self._shutting_down:
            return
        self._odom.hold_tf_alive(self.get_clock().now())

    def _on_habitat_odom(self, _msg: Odometry) -> None:
        if self._shutting_down:
            return
        self._shutting_down = True
        self.get_logger().info(
            'habitat_node /odom detected — releasing hold TF to habitat_node'
        )
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        # Defer destroy so the subscription callback can return cleanly.
        self._exit_timer = self.create_timer(0.01, self._finish)

    def _finish(self) -> None:
        if self._exit_timer is not None:
            self._exit_timer.cancel()
            self._exit_timer = None
        try:
            self.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: list[str] | None = None) -> None:
    rclpy.init(args=argv)
    node = OdomTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            try:
                node.destroy_node()
            except Exception:
                pass
            rclpy.shutdown()


if __name__ == '__main__':
    main()
