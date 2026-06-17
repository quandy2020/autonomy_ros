#
# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

"""Habitat-Sim ↔ ROS coordinate helpers."""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped


def yaw(q: np.quaternion) -> float:
    # Agent rotates about Habitat +Y; 2*atan2(y, w) avoids pi flip on identity.
    return 2.0 * math.atan2(float(q.y), float(q.w))


# Habitat agent forward is -Z; ROS REP-103 base_link forward is +X.
_HABITAT_ROS_YAW_OFFSET = -math.pi / 2.0


def habitat_yaw(ros_yaw_val: float) -> float:
    return ros_yaw_val + _HABITAT_ROS_YAW_OFFSET


def ros_yaw_from_habitat(habitat_yaw_val: float) -> float:
    return habitat_yaw_val - _HABITAT_ROS_YAW_OFFSET


def ros_yaw_from_quat(q: np.quaternion) -> float:
    return ros_yaw_from_habitat(yaw(q))


def quat(ros_yaw_val: float) -> np.quaternion:
    """ROS map yaw (Z rotation) -> Habitat agent quaternion (Y rotation)."""
    half = habitat_yaw(ros_yaw_val) * 0.5
    return np.quaternion(math.cos(half), 0.0, math.sin(half), 0.0)


def xy(pos: np.ndarray) -> tuple[float, float]:
    # Habitat (X, Y-up, Z) -> ROS map (x, y) with y = -Z; matches MP3D semantic PLY.
    return float(pos[0]), -float(pos[2])


def from_pose(pose: PoseStamped, floor: float) -> tuple[np.ndarray, np.quaternion]:
    # ROS pose z=0 means keep current floor height, not Habitat y=0.
    p = pose.pose.position
    height = float(p.z) if p.z != 0.0 else floor
    position = np.array([p.x, height, -p.y], dtype=np.float32)

    o = pose.pose.orientation
    ros_yaw = math.atan2(
        2.0 * (o.w * o.z + o.x * o.y),
        1.0 - 2.0 * (o.y * o.y + o.z * o.z),
    )
    return position, quat(ros_yaw)


def to_pose(
    pos: np.ndarray,
    rot: np.quaternion,
    frame_id: str,
    stamp: rclpy.time.Time,
) -> PoseStamped:
    map_x, map_y = xy(pos)
    half = ros_yaw_from_quat(rot) * 0.5

    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp.to_msg()
    msg.pose.position.x = map_x
    msg.pose.position.y = map_y
    msg.pose.position.z = 0.0
    msg.pose.orientation.z = math.sin(half)
    msg.pose.orientation.w = math.cos(half)
    return msg


def to_odom_pose(
    x: float,
    y: float,
    z: float,
    yaw_val: float,
    frame_id: str,
    stamp: rclpy.time.Time,
) -> PoseStamped:
    half = yaw_val * 0.5
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp.to_msg()
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.position.z = float(z)
    msg.pose.orientation.z = math.sin(half)
    msg.pose.orientation.w = math.cos(half)
    return msg


def ros_yaw(quaternion) -> float:
    o = quaternion
    return math.atan2(
        2.0 * (o.w * o.z + o.x * o.y),
        1.0 - 2.0 * (o.y * o.y + o.z * o.z),
    )
