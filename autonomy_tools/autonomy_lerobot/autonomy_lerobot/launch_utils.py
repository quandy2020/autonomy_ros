# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Shared launch helpers for autonomy_lerobot."""

from launch.substitutions import FindExecutable
from launch_ros.actions import Node


def lerobot_bridge_node(**kwargs) -> Node:
    """Start LeRobot bridge via python3 -m (reliable in Docker)."""
    return Node(
        executable=FindExecutable(name='python3'),
        arguments=['-m', 'autonomy_lerobot.node'],
        respawn=True,
        respawn_delay=3.0,
        **kwargs,
    )
