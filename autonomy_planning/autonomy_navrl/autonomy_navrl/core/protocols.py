"""Minimal protocols — decouple plugins from concrete Isaac env classes."""

from __future__ import annotations

from typing import Any, Protocol, runtime_checkable

import torch


@runtime_checkable
class NavEnvSurface(Protocol):
    """Surface area tasks / rewards / locomotion plugins may use."""

    device: torch.device
    num_envs: int
    cfg: Any

    @property
    def common_step_counter(self) -> int: ...

    def _numpy_rng(self): ...

    @property
    def _goals(self) -> torch.Tensor: ...

    @property
    def _goal_yaws(self) -> torch.Tensor: ...

    @property
    def _goal_curriculum(self) -> Any: ...

    @property
    def _reward_profile(self) -> str: ...

    @property
    def _robot(self) -> Any: ...

    @property
    def _task(self) -> Any: ...

    def _robot_pos_xy(self) -> torch.Tensor: ...

    def _robot_yaw(self) -> torch.Tensor: ...

    def _yaw_error(self) -> torch.Tensor: ...
