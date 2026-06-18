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

"""ROS message to numpy conversions for dataset recording."""

from __future__ import annotations

import math

import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


def image_to_numpy(msg: Image) -> np.ndarray:
    """Decode rgb8 or 32FC1 sensor_msgs/Image."""
    if msg.encoding == 'rgb8':
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        return np.ascontiguousarray(arr)
    if msg.encoding in ('32FC1', 'passthrough'):
        arr = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        return np.ascontiguousarray(arr)
    raise ValueError(f'unsupported image encoding: {msg.encoding}')


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def odom_to_state(msg: Odometry) -> np.ndarray:
    """Return [x, y, yaw, linear_x, angular_z]."""
    pose = msg.pose.pose
    yaw = yaw_from_quaternion(
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    twist = msg.twist.twist
    return np.array(
        [pose.position.x, pose.position.y, yaw, twist.linear.x, twist.angular.z],
        dtype=np.float32,
    )


def pose_to_state(msg: PoseStamped) -> np.ndarray:
    """Return [x, y, yaw, 0, 0]."""
    pose = msg.pose
    yaw = yaw_from_quaternion(
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    return np.array([pose.position.x, pose.position.y, yaw, 0.0, 0.0], dtype=np.float32)


def cmd_vel_to_action(msg: Twist) -> np.ndarray:
    """Return diff-drive action [linear_x, angular_z]."""
    return np.array([msg.linear.x, msg.angular.z], dtype=np.float32)


def camera_info_to_array(msg: CameraInfo) -> np.ndarray:
    """Return [fx, fy, cx, cy, width, height, d0..d4]."""
    k = msg.k
    d = list(msg.d) + [0.0] * (5 - len(msg.d))
    return np.array(
        [k[0], k[4], k[2], k[5], float(msg.width), float(msg.height), *d[:5]],
        dtype=np.float32,
    )


def occupancy_grid_to_array(msg: OccupancyGrid) -> np.ndarray:
    """Return int8 grid (H, W, 1). Values: -1 unknown, 0 free, 100 occupied."""
    h, w = msg.info.height, msg.info.width
    grid = np.asarray(msg.data, dtype=np.int8).reshape(h, w)
    return np.ascontiguousarray(grid[..., np.newaxis])


def grid_info_to_array(msg: OccupancyGrid) -> np.ndarray:
    """Return [resolution, width, height, origin_x, origin_y, origin_yaw]."""
    o = msg.info.origin
    yaw = yaw_from_quaternion(o.orientation.x, o.orientation.y,
                              o.orientation.z, o.orientation.w)
    return np.array(
        [msg.info.resolution, float(msg.info.width), float(msg.info.height),
         o.position.x, o.position.y, yaw],
        dtype=np.float32,
    )


def path_to_array(msg: Path, max_len: int) -> tuple[np.ndarray, np.ndarray]:
    """Return padded (max_len, 3) [x,y,yaw] and length scalar (1,)."""
    n = min(len(msg.poses), max_len)
    arr = np.zeros((max_len, 3), dtype=np.float32)
    for i in range(n):
        p = msg.poses[i].pose
        arr[i, 0] = p.position.x
        arr[i, 1] = p.position.y
        arr[i, 2] = yaw_from_quaternion(
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
    return arr, np.array([float(n)], dtype=np.float32)


def pointcloud_to_array(msg: PointCloud2, max_points: int) -> np.ndarray:
    """Return (max_points, 6) [x,y,z,r,g,b] with zero-padding."""
    fields = ('x', 'y', 'z', 'rgb') if pc2.has_field(msg, 'rgb') else ('x', 'y', 'z')
    pts = list(pc2.read_points(msg, field_names=fields, skip_nans=True))
    arr = np.zeros((max_points, 6), dtype=np.float32)
    n = min(len(pts), max_points)
    for i in range(n):
        arr[i, 0] = pts[i][0]
        arr[i, 1] = pts[i][1]
        arr[i, 2] = pts[i][2]
        if len(fields) == 4:
            rgb = int(pts[i][3])
            arr[i, 3] = (rgb >> 16) & 0xFF
            arr[i, 4] = (rgb >> 8) & 0xFF
            arr[i, 5] = rgb & 0xFF
    return arr
