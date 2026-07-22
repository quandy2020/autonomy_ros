#!/usr/bin/env python3
# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Convert pedsim TrackedPersons to pedsim_msgs/People for Nav2 social layer."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point
from pedsim_msgs.msg import TrackedPersons
from pedsim_msgs.msg import People, Person
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data


class TrackedPersonsToPeopleBridge(Node):
    def __init__(self) -> None:
        super().__init__('tracked_persons_to_people_bridge')
        self.declare_parameter('input_topic', 'pedestrian_visualizer/tracked_persons')
        self.declare_parameter('output_topic', 'people')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        input_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(People, output_topic, qos_profile_sensor_data)
        self.create_subscription(TrackedPersons, input_topic, self._on_tracks, input_qos)
        self.get_logger().info(
            f'tracked persons {input_topic} -> social layer {output_topic}')

    def _on_tracks(self, msg: TrackedPersons) -> None:
        out = People()
        out.header = msg.header
        for track in msg.tracks:
            if track.is_occluded:
                continue
            x = track.pose.pose.position.x
            y = track.pose.pose.position.y
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            person = Person()
            person.name = str(track.track_id)
            person.position = Point(x=x, y=y, z=0.0)
            person.velocity = Point(
                x=track.twist.twist.linear.x,
                y=track.twist.twist.linear.y,
                z=0.0,
            )
            person.reliability = 1.0
            out.people.append(person)
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = TrackedPersonsToPeopleBridge()
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
