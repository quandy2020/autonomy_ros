# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Inline ROS parameters for jdrobot / kujiale-compatible collection.

Namespaced nodes (e.g. /robot1/lerobot_bridge_node) do not receive parameters from
YAML keyed as ``lerobot_bridge_node:`` — use this dict in launch files.
"""

from __future__ import annotations

from typing import Any

# Episode timing shared with autonomy_task/config/collection_task.yaml
JDROBOT_EPISODE_TIMING: dict[str, float] = {
    'record_before_sec': 0.5,
    'record_after_sec': 1.0,
    'max_nav_sec': 40.0,
    'stall_move_m': 0.3,
}


def jdrobot_max_episode_seconds() -> float:
    """Upper bound for one online/offline episode (pre-record + nav + post-record)."""
    timing = JDROBOT_EPISODE_TIMING
    return timing['record_before_sec'] + timing['max_nav_sec'] + timing['record_after_sec']


# kujiale_0003 / LeRobot v3 jdrobot schema
JDROBOT_COLLECTION_ROS_PARAMS: dict[str, Any] = {
    'dataset_format': 'jdrobot',
    'robot_type': 'jdrobot',
    'rgb_topic': 'camera/rgb/image_raw',
    'depth_topic': 'camera/depth/image_raw',
    'camera_info_topic': 'camera/rgb/camera_info',
    'odom_topic': 'odom',
    'use_depth': True,
    'record_depth': True,
    'record_semantic': False,
    'record_map': False,
    'record_pointcloud': True,
    'pointcloud_from_depth': True,
    'pointcloud_stride': 4,
    'max_pointcloud_points': 4096,
    'record_camera_info': True,
    'record_nav2': False,
    'image_width': 640,
    'image_height': 480,
    'depth_min_m': 0.0,
    'depth_max_m': 10.0,
    'dataset_repo_id': 'local/habitat_collection',
    'record_fps': 20.0,
    'task': 'navigate to goal',
    'video_vcodec': 'av1',
    'streaming_encoding': True,
    'parallel_video_encoding': True,
}


def jdrobot_collection_parameters(**overrides: Any) -> dict[str, Any]:
    """Return jdrobot collection params with optional per-robot overrides."""
    return {**JDROBOT_COLLECTION_ROS_PARAMS, **overrides}
