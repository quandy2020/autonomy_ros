# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0

"""ROS 2 publishers for RGB, depth, and semantic camera topics."""

from __future__ import annotations

from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

from habitat_bridge.bridge_config import BridgeConfig
from habitat_bridge.image_conversion import (
    colorize_semantic,
    make_camera_info,
    numpy_depth_to_image,
    numpy_rgb_to_image,
)


def _observation_key(observations: Dict[str, Any], suffix: str) -> Optional[str]:
    for key in observations:
        if key == suffix or key.endswith(suffix):
            return key
    return None


class CameraPublishers:
    """Camera-related ROS publishers for the Habitat bridge."""

    def __init__(self, node: Node, config: BridgeConfig) -> None:
        self._config = config
        self._rgb_key: Optional[str] = None
        self._depth_key: Optional[str] = None
        self._semantic_key: Optional[str] = None

        qos = qos_profile_system_default
        self._rgb_pub: Optional[Publisher] = None
        self._depth_pub: Optional[Publisher] = None
        self._semantic_pub: Optional[Publisher] = None
        self._semantic_colored_pub: Optional[Publisher] = None
        self._rgb_info_pub: Optional[Publisher] = None
        self._depth_info_pub: Optional[Publisher] = None
        self._semantic_info_pub: Optional[Publisher] = None

        if config.enable_rgb:
            self._rgb_pub = node.create_publisher(Image, config.rgb_topic, qos)
            self._rgb_info_pub = node.create_publisher(
                CameraInfo, config.rgb_camera_info_topic, qos
            )
        if config.enable_depth:
            self._depth_pub = node.create_publisher(Image, config.depth_topic, qos)
            self._depth_info_pub = node.create_publisher(
                CameraInfo, config.depth_camera_info_topic, qos
            )
        if config.enable_semantic:
            self._semantic_pub = node.create_publisher(
                Image, config.semantic_topic, qos
            )
            self._semantic_info_pub = node.create_publisher(
                CameraInfo, config.semantic_camera_info_topic, qos
            )
        if config.enable_semantic_colored:
            self._semantic_colored_pub = node.create_publisher(
                Image, config.semantic_colored_topic, qos
            )

    def publish(self, stamp: rclpy.time.Time, observations: Dict[str, Any]) -> None:
        """Publish RGB, depth, and semantic camera images."""
        config = self._config
        rgb_header = Header()
        rgb_header.stamp = stamp.to_msg()
        rgb_header.frame_id = config.rgb_camera_frame
        rgb_camera_info = make_camera_info(
            rgb_header, config.image_width, config.image_height
        )

        if self._rgb_pub is not None:
            if self._rgb_key is None:
                self._rgb_key = _observation_key(observations, 'rgb')
            if self._rgb_key is not None:
                self._rgb_pub.publish(
                    numpy_rgb_to_image(rgb_header, observations[self._rgb_key])
                )
                if self._rgb_info_pub is not None:
                    self._rgb_info_pub.publish(rgb_camera_info)

        if self._depth_pub is not None:
            if self._depth_key is None:
                self._depth_key = _observation_key(observations, 'depth')
            if self._depth_key is not None:
                depth_header = Header()
                depth_header.stamp = stamp.to_msg()
                depth_header.frame_id = config.depth_camera_frame
                depth_camera_info = make_camera_info(
                    depth_header, config.image_width, config.image_height
                )
                self._depth_pub.publish(
                    numpy_depth_to_image(depth_header, observations[self._depth_key])
                )
                if self._depth_info_pub is not None:
                    self._depth_info_pub.publish(depth_camera_info)

        if self._semantic_pub is None and self._semantic_colored_pub is None:
            return

        if self._semantic_key is None:
            self._semantic_key = _observation_key(observations, 'semantic')
        if self._semantic_key is None:
            return

        semantic = observations[self._semantic_key]
        colored = colorize_semantic(semantic)
        semantic_header = Header()
        semantic_header.stamp = stamp.to_msg()
        semantic_header.frame_id = config.semantic_camera_frame
        semantic_camera_info = make_camera_info(
            semantic_header, config.image_width, config.image_height
        )
        if self._semantic_pub is not None:
            self._semantic_pub.publish(
                numpy_rgb_to_image(semantic_header, colored)
            )
            if self._semantic_info_pub is not None:
                self._semantic_info_pub.publish(semantic_camera_info)
        if self._semantic_colored_pub is not None:
            self._semantic_colored_pub.publish(
                numpy_rgb_to_image(semantic_header, colored)
            )
