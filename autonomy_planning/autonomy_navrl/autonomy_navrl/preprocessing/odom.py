"""Odometry and goal-relative state helpers."""

from __future__ import annotations

import numpy as np

from autonomy_navrl.core.types import OdomReading


def build_odom_vector(
    position_xy: np.ndarray,
    yaw: float,
    linear_velocity_xy: np.ndarray,
    angular_velocity_z: float,
) -> OdomReading:
    """Build an OdomReading from raw state."""
    return OdomReading(
        position_xy=np.asarray(position_xy, dtype=np.float32),
        yaw=float(yaw),
        linear_velocity_xy=np.asarray(linear_velocity_xy, dtype=np.float32),
        angular_velocity_z=float(angular_velocity_z),
    )


def goal_relative_state(
    position_xy: np.ndarray,
    yaw: float,
    goal_xy: np.ndarray,
) -> np.ndarray:
    """Return goal-relative features in robot frame: [dx, dy, distance, bearing]."""
    delta = goal_xy - position_xy
    distance = float(np.linalg.norm(delta))
    bearing_world = float(np.arctan2(delta[1], delta[0]))
    bearing = float(np.arctan2(
        np.sin(bearing_world - yaw),
        np.cos(bearing_world - yaw),
    ))
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    dx_body = cos_yaw * delta[0] + sin_yaw * delta[1]
    dy_body = -sin_yaw * delta[0] + cos_yaw * delta[1]
    return np.array([dx_body, dy_body, distance, bearing], dtype=np.float32)
