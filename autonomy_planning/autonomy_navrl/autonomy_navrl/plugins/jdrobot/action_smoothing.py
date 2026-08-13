"""Velocity command smoothing adapted from jdrobot PreTrainedPolicyAction."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import torch


@dataclass
class ActionSmoothingConfig:
    enabled: bool = False
    max_lin_acc: float = 0.8
    max_ang_acc: float = 1.5

    @classmethod
    def from_dict(cls, control_cfg: dict[str, Any]) -> ActionSmoothingConfig:
        smooth = control_cfg.get('action_smoothing', {})
        return cls(
            enabled=bool(smooth.get('enabled', False)),
            max_lin_acc=float(smooth.get('max_lin_acc', 0.8)),
            max_ang_acc=float(smooth.get('max_ang_acc', 1.5)),
        )


class VelocityActionSmoother:
    """Clamp per-step velocity changes for sim2real-friendly commands."""

    def __init__(
        self,
        num_envs: int,
        device: torch.device,
        cfg: ActionSmoothingConfig,
        control_dt: float,
    ) -> None:
        self.cfg = cfg
        self._dt = control_dt
        self._prev = torch.zeros(num_envs, 3, device=device)

    def reset(self, env_ids: torch.Tensor) -> None:
        self._prev[env_ids] = 0.0

    def apply(
        self,
        vx: torch.Tensor,
        vy: torch.Tensor,
        w: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        if not self.cfg.enabled:
            return vx, vy, w

        target = torch.stack([vx, vy, w], dim=-1)
        max_delta = target.new_tensor([
            self.cfg.max_lin_acc * self._dt,
            self.cfg.max_lin_acc * self._dt,
            self.cfg.max_ang_acc * self._dt,
        ])
        delta = torch.clamp(target - self._prev, -max_delta, max_delta)
        smoothed = self._prev + delta
        self._prev = smoothed.detach()
        return smoothed[:, 0], smoothed[:, 1], smoothed[:, 2]
