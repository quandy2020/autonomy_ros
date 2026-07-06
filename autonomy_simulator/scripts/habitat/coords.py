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


def quat_look_down() -> np.quaternion:
    """World-frame quaternion: pinhole camera looking straight down (Habitat Y-up)."""
    half = -math.pi * 0.25  # -90° about +X
    return np.quaternion(math.cos(half), math.sin(half), 0.0, 0.0)


def _normalize_quat(q: np.quaternion) -> np.quaternion:
    n = math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z)
    if n < 1e-12:
        return np.quaternion(1.0, 0.0, 0.0, 0.0)
    return np.quaternion(q.w / n, q.x / n, q.y / n, q.z / n)


def quat_look_at(eye: np.ndarray, target: np.ndarray) -> np.quaternion:
    """Habitat camera pose at eye, looking toward target (matches debug_visualizer)."""
    import magnum as mn

    eye_v = mn.Vector3(float(eye[0]), float(eye[1]), float(eye[2]))
    target_v = mn.Vector3(float(target[0]), float(target[1]), float(target[2]))
    look_dir = target_v - eye_v
    if look_dir.length() < 1e-8:
        return quat_look_down()

    look_up = mn.Vector3(0.0, 1.0, 0.0)
    if abs(look_dir.x) < 1e-6 and abs(look_dir.z) < 1e-6:
        look_up = mn.Vector3(1.0, 0.0, 0.0)

    q = mn.Quaternion.from_matrix(
        mn.Matrix4.look_at(eye_v, target_v, look_up).rotation(),
    )
    return _normalize_quat(
        np.quaternion(float(q.scalar), float(q.vector.x), float(q.vector.y), float(q.vector.z)),
    )


def map_forward_xy(yaw_rad: float) -> tuple[float, float]:
    """ROS map-frame forward unit vector from yaw."""
    return math.cos(yaw_rad), math.sin(yaw_rad)


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
