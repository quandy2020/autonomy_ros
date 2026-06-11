# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0

"""TF and odometry publishers for map->odom->base_footprint and cameras."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Dict, List

import numpy as np
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from habitat_bridge.bridge_config import BridgeConfig

if TYPE_CHECKING:
    from habitat_bridge.habitat_session import HabitatSession

_HAB_CAMERA_TO_ROS_OPTICAL = np.array(
    [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]],
    dtype=np.float64,
)


def _yaw_rotation_matrix(yaw: float) -> np.ndarray:
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return np.array(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def _matrix_to_quaternion(rotation: np.ndarray) -> Quaternion:
    matrix = rotation
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return Quaternion(
            x=(matrix[2, 1] - matrix[1, 2]) / s,
            y=(matrix[0, 2] - matrix[2, 0]) / s,
            z=(matrix[1, 0] - matrix[0, 1]) / s,
            w=0.25 * s,
        )

    if matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        s = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        return Quaternion(
            x=0.25 * s,
            y=(matrix[0, 1] + matrix[1, 0]) / s,
            z=(matrix[0, 2] + matrix[2, 0]) / s,
            w=(matrix[2, 1] - matrix[1, 2]) / s,
        )

    if matrix[1, 1] > matrix[2, 2]:
        s = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        return Quaternion(
            x=(matrix[0, 1] + matrix[1, 0]) / s,
            y=0.25 * s,
            z=(matrix[1, 2] + matrix[2, 1]) / s,
            w=(matrix[0, 2] - matrix[2, 0]) / s,
        )

    s = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
    return Quaternion(
        x=(matrix[0, 2] + matrix[2, 0]) / s,
        y=(matrix[1, 2] + matrix[2, 1]) / s,
        z=0.25 * s,
        w=(matrix[1, 0] - matrix[0, 1]) / s,
    )


def _pose_to_transform(
    stamp: object,
    parent_frame: str,
    child_frame: str,
    x: float,
    y: float,
    z: float,
    yaw: float,
) -> TransformStamped:
    translation = np.array([x, y, z], dtype=np.float64)
    return _make_transform(
        stamp,
        parent_frame,
        child_frame,
        translation,
        _yaw_rotation_matrix(yaw),
    )


def _make_transform(
    stamp: object,
    parent_frame: str,
    child_frame: str,
    translation: np.ndarray,
    rotation: np.ndarray,
) -> TransformStamped:
    transform = TransformStamped()
    transform.header.stamp = stamp.to_msg() if hasattr(stamp, 'to_msg') else stamp
    transform.header.frame_id = parent_frame
    transform.child_frame_id = child_frame
    transform.transform.translation.x = float(translation[0])
    transform.transform.translation.y = float(translation[1])
    transform.transform.translation.z = float(translation[2])
    transform.transform.rotation = _matrix_to_quaternion(rotation)
    return transform


def _enabled_sensor_frames(config: BridgeConfig) -> Dict[str, str]:
    frames: Dict[str, str] = {}
    if config.enable_rgb or config.enable_semantic_colored:
        frames['rgb'] = config.rgb_camera_frame
    if config.enable_depth:
        frames['depth'] = config.depth_camera_frame
    if config.enable_semantic or config.enable_semantic_colored:
        frames['semantic'] = config.semantic_camera_frame
    return frames


def build_odometry(
    session: HabitatSession,
    config: BridgeConfig,
    stamp: object,
    cmd_timed_out: bool = False,
) -> Odometry:
    x, y, z, yaw = session.odom_to_base_footprint_pose()
    if cmd_timed_out:
        linear, angular = 0.0, 0.0
    else:
        linear, angular = session.active_velocity()
    rotation = _yaw_rotation_matrix(yaw)

    message = Odometry()
    message.header.stamp = stamp.to_msg() if hasattr(stamp, 'to_msg') else stamp
    message.header.frame_id = config.odom_frame
    message.child_frame_id = config.base_footprint_frame
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.position.z = z
    message.pose.pose.orientation = _matrix_to_quaternion(rotation)
    message.twist.twist.linear.x = linear
    message.twist.twist.angular.z = angular
    return message


def _fixed_camera_mount_on_footprint(sensor_height: float) -> tuple[np.ndarray, np.ndarray]:
    """Fixed camera height above base_footprint (Habitat agent floor origin)."""
    translation = np.array([0.0, 0.0, sensor_height], dtype=np.float64)
    rotation = np.eye(3, dtype=np.float64)
    return translation, rotation


def build_transforms(
    session: HabitatSession,
    config: BridgeConfig,
    stamp: object,
) -> List[TransformStamped]:
    """Build map->odom, odom->base_footprint, base_footprint->camera_link->optical."""
    optical_frames = _enabled_sensor_frames(config)

    map_x, map_y, map_z, map_yaw = session.map_to_odom_pose()
    odom_x, odom_y, odom_z, odom_yaw = session.odom_to_base_footprint_pose()

    transforms = [
        _pose_to_transform(
            stamp,
            config.agent_pose_frame,
            config.odom_frame,
            map_x,
            map_y,
            map_z,
            map_yaw,
        ),
        _pose_to_transform(
            stamp,
            config.odom_frame,
            config.base_footprint_frame,
            odom_x,
            odom_y,
            odom_z,
            odom_yaw,
        ),
    ]

    if not optical_frames:
        return transforms

    mount_translation, mount_rotation = _fixed_camera_mount_on_footprint(
        config.sensor_height,
    )
    transforms.append(
        _make_transform(
            stamp,
            config.base_footprint_frame,
            config.camera_link_frame,
            mount_translation,
            mount_rotation,
        )
    )

    optical_origin = np.zeros(3, dtype=np.float64)
    for child_frame in optical_frames.values():
        transforms.append(
            _make_transform(
                stamp,
                config.camera_link_frame,
                child_frame,
                optical_origin,
                _HAB_CAMERA_TO_ROS_OPTICAL,
            )
        )

    return transforms


class RobotTfOdomPublisher:
    """Publish odometry and TF for the Habitat-driven robot."""

    def __init__(self, node, config: BridgeConfig) -> None:
        self._config = config
        self._enabled = config.publish_camera_tf
        self._tf_broadcaster = TransformBroadcaster(node) if self._enabled else None
        self._odom_pub = None
        if self._enabled:
            from rclpy.qos import qos_profile_system_default

            self._odom_pub = node.create_publisher(
                Odometry,
                config.odom_topic,
                qos_profile_system_default,
            )

    def publish(
        self,
        stamp: object,
        session: HabitatSession,
        cmd_timed_out: bool = False,
    ) -> None:
        if not self._enabled:
            return
        if self._odom_pub is not None:
            self._odom_pub.publish(
                build_odometry(session, self._config, stamp, cmd_timed_out)
            )
        if self._tf_broadcaster is not None:
            for transform in build_transforms(session, self._config, stamp):
                self._tf_broadcaster.sendTransform(transform)
