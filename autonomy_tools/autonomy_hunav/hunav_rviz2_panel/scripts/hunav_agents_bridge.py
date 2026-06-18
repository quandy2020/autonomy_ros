#!/usr/bin/env python3
# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0

"""Bridge hunav_msgs/Agents to derived_object_msgs/ObjectArray for pedsim RViz."""

from __future__ import annotations

import math

import rclpy
from derived_object_msgs.msg import Object, ObjectArray
from hunav_msgs.msg import Agent, Agents
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class HunavAgentsBridge(Node):
    def __init__(self) -> None:
        super().__init__('hunav_agents_bridge')
        self.declare_parameter('input_topic', '/human_states')
        self.declare_parameter('output_topic', '/hunav_agents_bridge/pedestrians')
        self.declare_parameter('frame_id', 'map')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._frame_id = self.get_parameter('frame_id').value

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(ObjectArray, output_topic, qos)
        self.create_subscription(Agents, input_topic, self._on_agents, qos)
        self.get_logger().info(f'Bridging {input_topic} -> {output_topic}')

    def _on_agents(self, msg: Agents) -> None:
        out = ObjectArray()
        out.header = msg.header
        out.header.frame_id = self._frame_id or msg.header.frame_id or 'map'

        for agent in msg.agents:
            if agent.type != Agent.PERSON:
                continue
            obj = Object()
            obj.id = int(agent.id)
            obj.pose.position.x = float(agent.position.position.x)
            obj.pose.position.y = float(agent.position.position.y)
            obj.pose.position.z = float(agent.position.position.z)
            obj.pose.orientation = agent.position.orientation

            speed = float(agent.linear_vel)
            yaw = float(agent.yaw)
            if speed <= 1e-6 and (
                abs(float(agent.velocity.linear.x)) > 1e-6
                or abs(float(agent.velocity.linear.y)) > 1e-6
            ):
                obj.twist.linear.x = float(agent.velocity.linear.x)
                obj.twist.linear.y = float(agent.velocity.linear.y)
            else:
                obj.twist.linear.x = speed * math.cos(yaw)
                obj.twist.linear.y = speed * math.sin(yaw)
            obj.twist.angular.z = float(agent.angular_vel)
            out.objects.append(obj)

        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = HunavAgentsBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
