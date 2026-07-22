#!/usr/bin/env python3
# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Convert pedestrian tracks (bbox pose + velocity) to TEB ObstacleArrayMsg."""

from __future__ import annotations

import math

import rclpy
from costmap_converter_msgs.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import Point32
from pedsim_msgs.msg import TrackedPersons
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class PedestrianObstacleBridge(Node):
    def __init__(self) -> None:
        super().__init__('pedestrian_obstacle_bridge')
        self.declare_parameter('input_topic', 'pedestrian_visualizer/tracked_persons')
        self.declare_parameter('output_topic', 'teb_obstacles')
        self.declare_parameter('pedestrian_radius', 0.35)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._radius = float(self.get_parameter('pedestrian_radius').value)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(ObstacleArrayMsg, output_topic, qos)
        self.create_subscription(TrackedPersons, input_topic, self._on_tracks, qos)
        self.get_logger().info(
            f'pedestrian tracks {input_topic} -> TEB obstacles {output_topic} '
            f'(radius={self._radius:.2f}m)')

    def _on_tracks(self, msg: TrackedPersons) -> None:
        out = ObstacleArrayMsg()
        out.header = msg.header
        for person in msg.tracks:
            if person.is_occluded:
                continue
            x = person.pose.pose.position.x
            y = person.pose.pose.position.y
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            obstacle = ObstacleMsg()
            obstacle.id = int(person.track_id)
            obstacle.radius = self._radius
            obstacle.orientation = person.pose.pose.orientation
            obstacle.velocities = person.twist
            pt = Point32()
            pt.x = x
            pt.y = y
            pt.z = 0.0
            obstacle.polygon.points = [pt]
            out.obstacles.append(obstacle)
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = PedestrianObstacleBridge()
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
