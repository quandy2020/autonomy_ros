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
    """Decode common ROS image encodings into contiguous numpy arrays.

    Depth images encoded as ``16UC1`` are interpreted as millimeters and
    converted to ``float32`` meters so downstream visualization and pointcloud
    code can use a single depth unit.
    """
    if msg.encoding == 'rgb8':
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        return np.ascontiguousarray(arr)
    if msg.encoding in ('32FC1', 'passthrough'):
        arr = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        return np.ascontiguousarray(arr)
    if msg.encoding == '16UC1':
        arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        depth_m = arr.astype(np.float32) / 1000.0
        return np.ascontiguousarray(depth_m)
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


def quaternion_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Return 3x3 rotation matrix from a unit quaternion."""
    return np.array(
        [
            [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
            [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
            [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
        ],
        dtype=np.float32,
    )


def pose_to_action_matrix(
    x: float, y: float, z: float,
    qx: float, qy: float, qz: float, qw: float,
) -> np.ndarray:
    """Return 4x4 pose as row-major flatten (jdrobot action schema)."""
    rot = quaternion_to_rotation_matrix(qx, qy, qz, qw)
    mat = np.eye(4, dtype=np.float32)
    mat[:3, :3] = rot
    mat[0, 3] = x
    mat[1, 3] = y
    mat[2, 3] = z
    return mat.reshape(-1)


def odom_to_action_matrix(msg: Odometry) -> np.ndarray:
    """Return jdrobot-style action from odometry pose."""
    pose = msg.pose.pose
    return pose_to_action_matrix(
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
    )


def identity_extrinsic() -> np.ndarray:
    """Return 4x4 identity extrinsic (row-major flatten), matching kujiale datasets."""
    return np.eye(4, dtype=np.float32).reshape(-1)


def camera_info_to_intrinsic(msg: CameraInfo) -> np.ndarray:
    """Return 3x3 camera matrix K flattened row-major (jdrobot schema)."""
    return np.array(msg.k[:9], dtype=np.float32)


def depth_to_video_rgb(depth: np.ndarray, depth_min: float, depth_max: float) -> np.ndarray:
    """Encode float depth (H, W) meters as uint8 RGB video frames (kujiale style)."""
    safe = np.nan_to_num(depth, nan=depth_min, posinf=depth_max, neginf=depth_min)
    denom = max(float(depth_max - depth_min), 1e-6)
    norm = np.clip((safe - depth_min) / denom, 0.0, 1.0)
    gray = (norm * 255.0).astype(np.uint8)
    return np.stack([gray, gray, gray], axis=-1)


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


def depth_rgb_to_pointcloud_array(
    depth: np.ndarray,
    camera_info: CameraInfo,
    *,
    rgb: np.ndarray | None = None,
    valid_mask: np.ndarray | None = None,
    max_points: int = 4096,
    depth_min: float = 0.05,
    depth_max: float = 10.0,
    stride: int = 4,
) -> np.ndarray:
    """Back-project depth to (max_points, 6) [x,y,z,r,g,b] in camera optical frame."""
    if depth.ndim != 2:
        raise ValueError('depth must be a 2-D array (H, W)')
    stride = max(1, int(stride))
    h, w = depth.shape
    fx = float(camera_info.k[0])
    fy = float(camera_info.k[4])
    cx = float(camera_info.k[2])
    cy = float(camera_info.k[5])
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError('camera_info K must have positive fx and fy')

    v_coords = np.arange(0, h, stride, dtype=np.int32)
    u_coords = np.arange(0, w, stride, dtype=np.int32)
    uu, vv = np.meshgrid(u_coords, v_coords)
    z = depth[vv, uu].astype(np.float32)
    valid = np.isfinite(z) & (z > depth_min) & (z < depth_max)
    if valid_mask is not None:
        if valid_mask.shape != depth.shape:
            raise ValueError('valid_mask must match depth shape')
        valid = valid & valid_mask[vv, uu].astype(bool)
    if not np.any(valid):
        return np.zeros((max_points, 6), dtype=np.float32)

    z = z[valid]
    u = uu[valid].astype(np.float32)
    v = vv[valid].astype(np.float32)
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    n_total = z.shape[0]
    if n_total > max_points:
        pick = np.linspace(0, n_total - 1, max_points, dtype=np.int64)
        x, y, z, u, v = x[pick], y[pick], z[pick], u[pick], v[pick]
    n = min(n_total, max_points)

    arr = np.zeros((max_points, 6), dtype=np.float32)
    arr[:n, 0] = x[:n]
    arr[:n, 1] = y[:n]
    arr[:n, 2] = z[:n]
    if rgb is not None:
        vi = np.clip(v[:n].astype(np.int32), 0, h - 1)
        ui = np.clip(u[:n].astype(np.int32), 0, w - 1)
        arr[:n, 3:6] = rgb[vi, ui].astype(np.float32)
    return arr
