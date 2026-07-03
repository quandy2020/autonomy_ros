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

"""ROS message conversions for NavDP."""

from __future__ import annotations

import math

import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

_BRIDGE = CvBridge()


def image_to_bgr(msg: Image) -> np.ndarray:
    """Convert sensor Image to BGR uint8 array (NavDP convention)."""
    encoding = msg.encoding.lower()
    if encoding in ('rgb8', 'bgr8'):
        cv_img = _BRIDGE.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    elif encoding in ('rgba8', 'bgra8'):
        cv_img = _BRIDGE.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    else:
        cv_img = _BRIDGE.imgmsg_to_cv2(msg)
        if len(cv_img.shape) == 2:
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
        elif cv_img.shape[2] == 4:
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGRA2BGR)
        elif cv_img.shape[2] == 3:
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
    return np.asarray(cv_img, dtype=np.uint8)


def depth_to_meters(msg: Image, depth_encoding: str = '32fc1') -> np.ndarray:
    """Convert depth Image to float32 meters with shape (H, W, 1)."""
    enc = depth_encoding.lower()
    if enc in ('16uc1', '16uc1_mm') or msg.encoding == '16UC1':
        raw = _BRIDGE.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        depth = raw.astype(np.float32) / 1000.0
    elif enc in ('16uc1_10k', '16uc1_navdp'):
        raw = _BRIDGE.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        depth = raw.astype(np.float32) / 10000.0
    else:
        depth = _BRIDGE.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        depth = np.asarray(depth, dtype=np.float32)
    depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
    if depth.ndim == 2:
        depth = depth[:, :, np.newaxis]
    return depth


def camera_info_to_intrinsic(msg: CameraInfo) -> np.ndarray:
    """Return 3x3 camera intrinsic matrix."""
    return np.array(msg.k, dtype=np.float64).reshape(3, 3)


def _quat_to_rot_matrix(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def goal_to_robot_xy(goal: PoseStamped, odom: Odometry) -> tuple[np.ndarray, np.ndarray]:
    """Transform goal into robot body frame (NavDP / Isaac oracle_imu_pose convention).

    Returns (goal_array shape (1,3), rel_xyz in body frame).
    """
    gp = goal.pose.position
    rp = odom.pose.pose.position
    rot = _quat_to_rot_matrix(odom.pose.pose.orientation)
    rel = rot.T @ (np.array([gp.x, gp.y, gp.z]) - np.array([rp.x, rp.y, rp.z]))
    goal_arr = np.array([[rel[0], rel[1], 0.0]], dtype=np.float32)
    return goal_arr, rel


def trajectory_body_to_map_xy(trajectory: np.ndarray, odom: Odometry) -> np.ndarray:
    """Map body-frame NavDP trajectory (x, y, yaw) to map-frame ground points."""
    x0 = odom.pose.pose.position.x
    y0 = odom.pose.pose.position.y
    yaw = _yaw_from_quaternion(odom.pose.pose.orientation)
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    mapped = []
    for point in trajectory:
        bx, by = float(point[0]), float(point[1])
        mapped.append([x0 + cos_y * bx - sin_y * by, y0 + sin_y * bx + cos_y * by, 0.0])
    return np.asarray(mapped, dtype=np.float64)


def trajectories_body_to_map_xy(
    all_trajectories: np.ndarray,
    odom: Odometry,
) -> np.ndarray:
    """Map a batch of body-frame diffusion samples to map-frame (N, T, 3)."""
    arr = np.asarray(all_trajectories)
    if arr.ndim == 4:
        arr = arr[0]
    return np.stack(
        [trajectory_body_to_map_xy(traj, odom) for traj in arr],
        axis=0,
    )


def bgr_array_to_image_msg(
    bgr: np.ndarray,
    stamp,
    frame_id: str,
) -> Image:
    """Publish NavDP ``trajectory_mask`` overlay as sensor_msgs/Image."""
    msg = _BRIDGE.cv2_to_imgmsg(np.asarray(bgr, dtype=np.uint8), encoding='bgr8')
    msg.header = Header(stamp=stamp, frame_id=frame_id)
    return msg


def trajectory_to_path(
    trajectory: np.ndarray,
    stamp,
    frame_id: str,
) -> Path:
    """Convert NavDP trajectory rows (x, y, yaw) to nav_msgs/Path."""
    path = Path()
    path.header = Header(stamp=stamp, frame_id=frame_id)
    for point in trajectory:
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = float(point[0])
        pose.pose.position.y = float(point[1])
        pose.pose.position.z = 0.0
        yaw = float(point[2]) if len(point) > 2 else 0.0
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        path.poses.append(pose)
    return path


def trajectory_to_twist(
    trajectory: np.ndarray,
    linear_speed: float,
    angular_speed: float,
    lookahead_index: int,
    min_dist: float = 0.05,
) -> Twist:
    """Pure-pursuit velocity from body-frame trajectory (x forward, y left)."""
    cmd = Twist()
    if trajectory.size == 0:
        return cmd

    n = len(trajectory)
    order = list(range(min(max(lookahead_index, 0), n - 1), n)) + list(range(0, min(lookahead_index, n)))
    for idx in order:
        target = trajectory[idx]
        x = float(target[0])
        y = float(target[1])
        dist = math.hypot(x, y)
        if dist < min_dist:
            continue
        heading = math.atan2(y, x)
        cmd.linear.x = min(linear_speed, dist)
        cmd.angular.z = max(-angular_speed, min(angular_speed, heading))
        return cmd
    return cmd
