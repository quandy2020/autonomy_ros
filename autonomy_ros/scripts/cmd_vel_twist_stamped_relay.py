#!/usr/bin/env python3
# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Relay geometry_msgs/Twist on cmd_vel to TwistStamped for habitat_bridge."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default


class CmdVelTwistStampedRelay(Node):
  """Forward Nav2 cmd_vel (Twist) as TwistStamped for Habitat."""

  def __init__(self) -> None:
    super().__init__('cmd_vel_twist_stamped_relay')
    self.declare_parameter('input_topic', 'cmd_vel')
    self.declare_parameter('output_topic', 'cmd_vel_stamped')
    input_topic = self.get_parameter('input_topic').value
    output_topic = self.get_parameter('output_topic').value
    self._publisher = self.create_publisher(
        TwistStamped, output_topic, qos_profile_system_default)
    self.create_subscription(
        Twist, input_topic, self._on_cmd_vel, qos_profile_system_default)

  def _on_cmd_vel(self, message: Twist) -> None:
    stamped = TwistStamped()
    stamped.header.stamp = self.get_clock().now().to_msg()
    stamped.header.frame_id = 'base_footprint'
    stamped.twist = message
    self._publisher.publish(stamped)


def main() -> None:
  rclpy.init()
  node = CmdVelTwistStampedRelay()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
