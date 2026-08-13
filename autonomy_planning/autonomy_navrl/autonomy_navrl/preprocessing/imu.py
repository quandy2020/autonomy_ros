"""IMU normalization helpers."""

from __future__ import annotations

import numpy as np

from autonomy_navrl.core.types import ImuReading


def normalize_imu(
    angular_velocity: np.ndarray,
    linear_acceleration: np.ndarray,
    orientation_rpy: np.ndarray,
    gravity: float = 9.81,
) -> ImuReading:
    """Normalize raw IMU arrays into an ImuReading."""
    return ImuReading(
        angular_velocity=np.asarray(angular_velocity, dtype=np.float32),
        linear_acceleration=np.asarray(linear_acceleration, dtype=np.float32) / gravity,
        orientation_rpy=np.asarray(orientation_rpy, dtype=np.float32),
    )
