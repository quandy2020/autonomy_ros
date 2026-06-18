#!/usr/bin/env python3
# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0

"""Publish a static robot pose for standalone pedestrian simulator demos."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.node import Node


def yaw_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class PedestrianDemo(Node):
    def __init__(self) -> None:
        super().__init__('pedestrian_demo')
        self.declare_parameter('robot_x', -8.0)
        self.declare_parameter('robot_y', 0.0)
        self.declare_parameter('robot_yaw', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_hz', 10.0)

        self._x = self.get_parameter('robot_x').value
        self._y = self.get_parameter('robot_y').value
        self._yaw = self.get_parameter('robot_yaw').value
        self._frame = self.get_parameter('frame_id').value
        hz = float(self.get_parameter('publish_hz').value)

        self._pub = self.create_publisher(PoseStamped, '/robot_state', 10)
        self.create_timer(1.0 / hz, self._on_timer)
        self.get_logger().info(
            f'[pedestrian_demo] publishing /robot_state at ({self._x}, {self._y})'
        )

    def _on_timer(self) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame
        msg.pose.position.x = float(self._x)
        msg.pose.position.y = float(self._y)
        msg.pose.orientation = yaw_quaternion(float(self._yaw))
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = PedestrianDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
