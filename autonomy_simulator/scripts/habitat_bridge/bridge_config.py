# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0

"""ROS parameter configuration for the Habitat bridge."""

from __future__ import annotations

import os
from dataclasses import dataclass
from rclpy.node import Node


@dataclass(frozen=True)
class BridgeConfig:
    """Immutable bridge configuration loaded from ROS parameters."""

    scene_data_path: str = '/workspace/autonomy/src/17DRP5sb8fy'
    scene_id: str = '17DRP5sb8fy'
    mp3d_root: str = '/workspace/autonomy/src'
    scene_dataset_config: str = ''
    package_scene_dataset_config: str = ''
    image_width: int = 640
    image_height: int = 480
    sensor_height: float = 0.6

    agent_pose_topic: str = 'habitat/agent_pose'
    set_agent_pose_topic: str = 'habitat/set_agent_pose'
    agent_pose_frame: str = 'map'
    odom_frame: str = 'odom'
    base_footprint_frame: str = 'base_footprint'
    base_frame: str = 'base_link'
    base_link_height: float = 0.01
    camera_link_frame: str = 'camera_link'
    rgb_camera_frame: str = 'camera_rgb_optical_frame'
    depth_camera_frame: str = 'camera_depth_optical_frame'
    semantic_camera_frame: str = 'camera_semantic_optical_frame'
    publish_camera_tf: bool = True
    odom_topic: str = 'odom'
    cmd_vel_topic: str = 'cmd_vel'
    cmd_vel_timeout: float = 1.0
    wheel_separation: float = 0.287
    wheel_radius: float = 0.033

    rgb_topic: str = 'camera/rgb/image_raw'
    depth_topic: str = 'camera/depth/image_raw'
    semantic_topic: str = 'camera/semantic/image_raw'
    semantic_colored_topic: str = 'camera/semantic_colored/image_raw'
    rgb_camera_info_topic: str = 'camera/rgb/camera_info'
    depth_camera_info_topic: str = 'camera/depth/camera_info'
    semantic_camera_info_topic: str = 'camera/semantic/camera_info'

    semantic_ply_path: str = ''
    semantic_pointcloud_topic: str = 'semantic_pointcloud'
    semantic_pointcloud_frame: str = 'map'
    semantic_pointcloud_downsample: int = 1
    semantic_pointcloud_rate_hz: float = 1.0

    enable_rgb: bool = True
    enable_depth: bool = True
    enable_semantic: bool = True
    enable_semantic_colored: bool = True
    enable_semantic_pointcloud: bool = True
    update_rate_hz: float = 30.0

    @classmethod
    def from_node(cls, node: Node) -> 'BridgeConfig':
        """Load configuration from declared ROS parameters."""
        defaults = cls()
        spec = [
            ('scene_data_path', defaults.scene_data_path),
            ('scene_id', defaults.scene_id),
            ('mp3d_root', defaults.mp3d_root),
            ('scene_dataset_config', defaults.scene_dataset_config),
            ('package_scene_dataset_config', defaults.package_scene_dataset_config),
            ('image_width', defaults.image_width),
            ('image_height', defaults.image_height),
            ('sensor_height', defaults.sensor_height),
            ('agent_pose_topic', defaults.agent_pose_topic),
            ('set_agent_pose_topic', defaults.set_agent_pose_topic),
            ('agent_pose_frame', defaults.agent_pose_frame),
            ('odom_frame', defaults.odom_frame),
            ('base_footprint_frame', defaults.base_footprint_frame),
            ('base_frame', defaults.base_frame),
            ('base_link_height', defaults.base_link_height),
            ('camera_link_frame', defaults.camera_link_frame),
            ('rgb_camera_frame', defaults.rgb_camera_frame),
            ('depth_camera_frame', defaults.depth_camera_frame),
            ('semantic_camera_frame', defaults.semantic_camera_frame),
            ('publish_camera_tf', defaults.publish_camera_tf),
            ('odom_topic', defaults.odom_topic),
            ('cmd_vel_topic', defaults.cmd_vel_topic),
            ('cmd_vel_timeout', defaults.cmd_vel_timeout),
            ('wheel_separation', defaults.wheel_separation),
            ('wheel_radius', defaults.wheel_radius),
            ('rgb_topic', defaults.rgb_topic),
            ('depth_topic', defaults.depth_topic),
            ('semantic_topic', defaults.semantic_topic),
            ('semantic_colored_topic', defaults.semantic_colored_topic),
            ('rgb_camera_info_topic', defaults.rgb_camera_info_topic),
            ('depth_camera_info_topic', defaults.depth_camera_info_topic),
            ('semantic_camera_info_topic', defaults.semantic_camera_info_topic),
            ('enable_rgb', defaults.enable_rgb),
            ('enable_depth', defaults.enable_depth),
            ('enable_semantic', defaults.enable_semantic),
            ('enable_semantic_colored', defaults.enable_semantic_colored),
            ('semantic_ply_path', defaults.semantic_ply_path),
            ('semantic_pointcloud_topic', defaults.semantic_pointcloud_topic),
            ('semantic_pointcloud_frame', defaults.semantic_pointcloud_frame),
            ('semantic_pointcloud_downsample', defaults.semantic_pointcloud_downsample),
            ('semantic_pointcloud_rate_hz', defaults.semantic_pointcloud_rate_hz),
            ('enable_semantic_pointcloud', defaults.enable_semantic_pointcloud),
            ('update_rate_hz', defaults.update_rate_hz),
        ]
        to_declare = [
            (name, value) for name, value in spec if not node.has_parameter(name)
        ]
        if to_declare:
            node.declare_parameters('', to_declare)

        scene_data_path = node.get_parameter('scene_data_path').value
        scene_id = node.get_parameter('scene_id').value
        mp3d_root = node.get_parameter('mp3d_root').value
        if scene_data_path:
            scene_data_path = scene_data_path.rstrip('/')
            scene_id = os.path.basename(scene_data_path)
            mp3d_root = os.path.dirname(scene_data_path)

        return cls(**{
            name: node.get_parameter(name).value
            for name, _ in spec
            if name not in ('scene_id', 'mp3d_root')
        } | {'scene_id': scene_id, 'mp3d_root': mp3d_root})

    def resolved_scene_dataset_config(self) -> str:
        """Return scene dataset config path, using mp3d_root default when empty."""
        if self.scene_dataset_config:
            return self.scene_dataset_config
        return os.path.join(self.mp3d_root, 'mp3d.scene_dataset_config.json')

    def resolved_scene_asset(self) -> str:
        """Return Habitat scene asset id relative to the scene dataset root."""
        return f'{self.scene_id}/{self.scene_id}.glb'

    def scene_directory(self) -> str:
        """Absolute path to the scene asset directory."""
        if self.scene_data_path:
            return self.scene_data_path.rstrip('/')
        return os.path.join(self.mp3d_root, self.scene_id)
