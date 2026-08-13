"""Deploy-time proprioceptive state construction."""

from __future__ import annotations

import math

import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from autonomy_navrl.core.state import PROPRIO_STATE_DIM, build_proprioceptive_state
from autonomy_navrl.preprocessing.imu import normalize_imu
from autonomy_navrl.preprocessing.odom import build_odom_vector


def _yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def build_deploy_state_vector(
    odom: Odometry,
    goal: PoseStamped,
    *,
    imu: Imu | None = None,
    state_dim: int = PROPRIO_STATE_DIM,
) -> np.ndarray:
    """Build policy state vector from live ROS sensor messages."""
    position = np.array([
        odom.pose.pose.position.x,
        odom.pose.pose.position.y,
    ], dtype=np.float32)
    yaw = _yaw_from_quaternion(odom.pose.pose.orientation)
    linear = odom.twist.twist.linear
    angular = odom.twist.twist.angular
    goal_xy = np.array([
        goal.pose.position.x,
        goal.pose.position.y,
    ], dtype=np.float32)
    if imu is not None:
        imu_reading = normalize_imu(
            np.array([
                imu.angular_velocity.x,
                imu.angular_velocity.y,
                imu.angular_velocity.z,
            ], dtype=np.float32),
            np.array([
                imu.linear_acceleration.x,
                imu.linear_acceleration.y,
                imu.linear_acceleration.z,
            ], dtype=np.float32),
            np.zeros(3, dtype=np.float32),
        )
    else:
        imu_reading = None
    odom_vec = build_odom_vector(
        position,
        yaw,
        np.array([linear.x, linear.y], dtype=np.float32),
        angular.z,
    )
    return build_proprioceptive_state(
        position, yaw, goal_xy, imu=imu_reading, odom=odom_vec, state_dim=state_dim,
    )
