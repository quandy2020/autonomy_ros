"""Proprioceptive state vector construction."""

from __future__ import annotations

import numpy as np

from autonomy_navrl.core.types import ImuReading, OdomReading
from autonomy_navrl.preprocessing.odom import goal_relative_state

# goal(4) + imu(9) + odom(6)
PROPRIO_STATE_DIM = 19


def build_proprioceptive_state(
    position_xy: np.ndarray,
    yaw: float,
    goal_xy: np.ndarray,
    imu: ImuReading | None = None,
    odom: OdomReading | None = None,
    state_dim: int = PROPRIO_STATE_DIM,
) -> np.ndarray:
    """Pack goal, IMU, and odom features into a fixed-length state vector."""
    goal_feat = goal_relative_state(position_xy, yaw, goal_xy)
    imu_feat = imu.as_vector() if imu is not None else np.zeros(9, dtype=np.float32)
    if odom is not None:
        odom_feat = odom.as_vector()
    else:
        odom_feat = np.array(
            [position_xy[0], position_xy[1], yaw, 0.0, 0.0, 0.0],
            dtype=np.float32,
        )
    packed = np.concatenate([goal_feat, imu_feat, odom_feat], axis=0).astype(np.float32)
    state = np.zeros(state_dim, dtype=np.float32)
    copy_len = min(state_dim, packed.shape[0])
    state[:copy_len] = packed[:copy_len]
    return state
