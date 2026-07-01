# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Shared launch helpers for autonomy_task."""

from launch.substitutions import FindExecutable
from launch_ros.actions import Node


def collection_coordinator_node(**kwargs) -> Node:
    """Start the collection coordinator via python3 -m (robust in Docker/minimal PATH)."""
    return Node(
        executable=FindExecutable(name='python3'),
        arguments=['-m', 'autonomy_task.node'],
        **kwargs,
    )
