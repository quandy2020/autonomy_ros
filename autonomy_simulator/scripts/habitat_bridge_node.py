#!/usr/bin/env python3
# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0

"""ROS 2 entry point for the Habitat bridge node."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import List, Optional

_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

import rclpy

from habitat_bridge.node import HabitatBridgeNode


def main(args: Optional[List[str]] = None) -> None:
    """Initialize ROS, spin the bridge node, and shut down cleanly."""
    rclpy.init(args=args)
    node = HabitatBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
