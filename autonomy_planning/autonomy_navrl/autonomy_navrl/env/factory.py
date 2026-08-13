"""Environment factory."""

from __future__ import annotations

from typing import Any

from autonomy_navrl.env.base_env import BaseNavrlEnv
from autonomy_navrl.env.mock_env import MockNavrlEnv


def create_training_env(config: dict[str, Any], backend: str = 'auto') -> BaseNavrlEnv:
    """Create a training environment.

    Visual navigation training requires ``backend=isaac`` (Isaac Lab TiledCamera + Imu).
    Mock is for CI/PPO pipeline smoke tests only (RGBD zeros, no Isaac sensors).
    """
    if backend == 'mock':
        return MockNavrlEnv(config)

    from autonomy_navrl.env.isaac_env import IsaacLabNavrlEnv

    if backend in ('isaac', 'auto'):
        return IsaacLabNavrlEnv(config)

    raise ValueError(f'Unknown training backend: {backend}')
