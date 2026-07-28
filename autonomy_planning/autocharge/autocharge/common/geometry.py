"""Shared 2D geometry helpers."""

from __future__ import annotations

import math


def wrap_pi(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw from quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    """Return quaternion (x, y, z, w) from planar yaw."""
    half = 0.5 * float(yaw)
    return (0.0, 0.0, math.sin(half), math.cos(half))
