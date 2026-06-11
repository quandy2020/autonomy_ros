# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0

"""ROS 2 node bridging Habitat-Sim MP3D scenes to camera and pose topics."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from habitat_bridge.bridge_config import BridgeConfig
from habitat_bridge.camera_publishers import CameraPublishers
from habitat_bridge.habitat_session import HabitatSession
from habitat_bridge.pointcloud_publishers import SemanticPointcloudPublisher
from habitat_bridge.tf_broadcasters import RobotTfOdomPublisher


class HabitatBridgeNode(Node):
    """Bridge Habitat agent pose and camera sensors to ROS 2 topics."""

    def __init__(self) -> None:
        super().__init__('habitat_bridge_node')
        self._config = BridgeConfig.from_node(self)
        self._cameras = CameraPublishers(self, self._config)
        self._semantic_cloud = SemanticPointcloudPublisher(
            self, self._config, self.get_logger()
        )
        self._session = HabitatSession(self._config, self.get_logger())
        self._tf_odom = RobotTfOdomPublisher(self, self._config)

        self._last_cmd_time = self.get_clock().now()

        self._agent_pose_pub = self.create_publisher(
            PoseStamped,
            self._config.agent_pose_topic,
            qos_profile_system_default,
        )
        self.create_subscription(
            PoseStamped,
            self._config.set_agent_pose_topic,
            self._on_set_agent_pose,
            qos_profile_system_default,
        )
        self.create_subscription(
            TwistStamped,
            self._config.cmd_vel_topic,
            self._on_cmd_vel,
            qos_profile_system_default,
        )

        period_s = 1.0 / self._config.update_rate_hz
        self.create_timer(period_s, self._on_timer)

        cloud_rate_hz = float(self._config.semantic_pointcloud_rate_hz)
        if cloud_rate_hz > 0.0:
            self.create_timer(1.0 / cloud_rate_hz, self._on_pointcloud_timer)

        self.get_logger().info(
            f'[habitat_bridge] scene={self._config.scene_id} '
            f'cmd_vel={self._config.cmd_vel_topic} '
            f'odom={self._config.odom_topic}'
        )

    def _on_cmd_vel(self, message: TwistStamped) -> None:
        self._last_cmd_time = self.get_clock().now()
        self._session.set_velocity_command(
            float(message.twist.linear.x),
            float(message.twist.angular.z),
        )

    def _on_set_agent_pose(self, message: PoseStamped) -> None:
        self._session.set_agent_pose(message)
        self._session.set_velocity_command(0.0, 0.0)

    def _cmd_timed_out(self) -> bool:
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        return elapsed > self._config.cmd_vel_timeout

    def _on_timer(self) -> None:
        stamp = self.get_clock().now()
        dt = 1.0 / self._config.update_rate_hz
        cmd_timed_out = self._cmd_timed_out()
        observations = self._session.step(dt, cmd_timed_out)

        self._agent_pose_pub.publish(
            self._session.agent_pose_stamped(self._config.agent_pose_frame, stamp)
        )
        self._tf_odom.publish(stamp, self._session, cmd_timed_out)
        self._cameras.publish(stamp, observations)

    def _on_pointcloud_timer(self) -> None:
        self._semantic_cloud.publish(self.get_clock().now())

    def destroy_node(self) -> bool:
        self._session.close()
        return super().destroy_node()
