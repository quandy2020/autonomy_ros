"""Shared data types for training and deployment."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np


@dataclass(frozen=True)
class VelocityCommand:
    """High-level velocity command (vx, vy, w)."""

    vx: float
    vy: float
    w: float

    def as_array(self) -> np.ndarray:
        """Return command as float32 array."""
        return np.array([self.vx, self.vy, self.w], dtype=np.float32)

    @classmethod
    def from_array(cls, values: np.ndarray) -> VelocityCommand:
        """Build command from a length-3 array."""
        if values.shape[-1] != 3:
            raise ValueError(f'Expected action dim 3, got {values.shape}')
        return cls(vx=float(values[0]), vy=float(values[1]), w=float(values[2]))


@dataclass(frozen=True)
class ImuReading:
    """IMU observation in body frame."""

    angular_velocity: np.ndarray
    linear_acceleration: np.ndarray
    orientation_rpy: np.ndarray

    def as_vector(self) -> np.ndarray:
        """Flatten IMU reading to a 9-d vector."""
        return np.concatenate(
            [self.angular_velocity, self.linear_acceleration, self.orientation_rpy]
        ).astype(np.float32)


@dataclass(frozen=True)
class OdomReading:
    """Odometry observation."""

    position_xy: np.ndarray
    yaw: float
    linear_velocity_xy: np.ndarray
    angular_velocity_z: float

    def as_vector(self) -> np.ndarray:
        """Flatten odom reading to a 6-d vector."""
        return np.array(
            [
                self.position_xy[0],
                self.position_xy[1],
                self.yaw,
                self.linear_velocity_xy[0],
                self.linear_velocity_xy[1],
                self.angular_velocity_z,
            ],
            dtype=np.float32,
        )


@dataclass(frozen=True)
class RgbdFrame:
    """Single RGB-D frame."""

    rgb: np.ndarray
    depth_m: np.ndarray


@dataclass
class TrainConfig:
    """Training configuration wrapper."""

    raw: dict[str, Any] = field(default_factory=dict)

    @property
    def run_name(self) -> str:
        return str(self.raw.get('run_name', 'navrl'))

    @property
    def robot_urdf_path(self) -> str:
        return str(self.raw.get('robot', {}).get('urdf_path', ''))

    @property
    def num_envs(self) -> int:
        return int(self.raw.get('num_envs', 1))


@dataclass
class DeployConfig:
    """ROS deployment configuration."""

    checkpoint: str = 'navrl_latest.pt'
    device: str = 'cuda:0'
    rgb_topic: str = 'camera/rgb/image_raw'
    depth_topic: str = 'camera/depth/image_raw'
    imu_topic: str = 'imu'
    odom_topic: str = 'odom'
    goal_topic: str = 'goal_pose'
    cmd_vel_topic: str = 'cmd_vel'
    markers_topic: str = 'navrl/markers'
    path_topic: str = 'navrl/path'
    cmd_msg_type: str = 'twist'
    base_frame: str = 'base_link'
    map_frame: str = 'map'
    inference_rate_hz: float = 20.0
    image_width: int = 128
    image_height: int = 96
    max_vx: float = 1.0
    max_vy: float = 0.5
    max_w: float = 1.5
    goal_tolerance_m: float = 0.5

    @classmethod
    def from_dict(cls, params: dict[str, Any]) -> DeployConfig:
        """Build config from a flat ROS parameter dictionary."""
        known = {field.name for field in cls.__dataclass_fields__.values()}  # type: ignore[attr-defined]
        filtered = {key: value for key, value in params.items() if key in known}
        return cls(**filtered)
