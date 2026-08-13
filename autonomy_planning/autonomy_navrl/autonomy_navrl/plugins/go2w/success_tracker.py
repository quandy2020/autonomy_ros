"""Consecutive IoU success tracking for Go2W precision parking."""

from __future__ import annotations

from dataclasses import dataclass

import torch

from autonomy_navrl.plugins.go2w.iou import (
    GO2W_ROBOT_LENGTH,
    GO2W_ROBOT_WIDTH,
    GO2W_SPOT_LENGTH,
    GO2W_SPOT_WIDTH,
    bbox_iou_from_body_pose,
)


@dataclass(frozen=True)
class ConsecutiveIoUConfig:
    enabled: bool = True
    iou_threshold: float = 0.95
    num_success: int = 10
    theta_threshold: float | None = None
    robot_length: float = GO2W_ROBOT_LENGTH
    robot_width: float = GO2W_ROBOT_WIDTH
    spot_length: float = GO2W_SPOT_LENGTH
    spot_width: float = GO2W_SPOT_WIDTH

    @classmethod
    def from_dict(cls, raw: dict | None) -> ConsecutiveIoUConfig:
        if not raw:
            return cls(enabled=False)
        return cls(
            enabled=bool(raw.get('enabled', True)),
            iou_threshold=float(raw.get('iou_threshold', 0.95)),
            num_success=int(raw.get('num_success', 10)),
            theta_threshold=(
                float(raw['theta_threshold']) if raw.get('theta_threshold') is not None else None
            ),
            robot_length=float(raw.get('robot_length', GO2W_ROBOT_LENGTH)),
            robot_width=float(raw.get('robot_width', GO2W_ROBOT_WIDTH)),
            spot_length=float(raw.get('spot_length', GO2W_SPOT_LENGTH)),
            spot_width=float(raw.get('spot_width', GO2W_SPOT_WIDTH)),
        )


class ConsecutiveIoUTracker:
    """Matches Isaac Lab ConsecutiveIoUSuccess semantics for DirectRLEnv."""

    def __init__(self, cfg: ConsecutiveIoUConfig, num_envs: int, device: torch.device) -> None:
        self.cfg = cfg
        self._counter = torch.zeros(num_envs, device=device)
        self._max_counter = torch.zeros(num_envs, device=device)
        self.current_iou = torch.zeros(num_envs, device=device)
        self.navigation_success = torch.zeros(num_envs, device=device)

    def reset(self, env_ids: torch.Tensor) -> None:
        if not self.cfg.enabled:
            return
        succeeded = self._counter[env_ids] >= float(self.cfg.num_success)
        self.navigation_success[env_ids] = succeeded.float()
        self._counter[env_ids] = 0.0
        self._max_counter[env_ids] = 0.0
        self.current_iou[env_ids] = 0.0

    def update(
        self,
        dx_body: torch.Tensor,
        dy_body: torch.Tensor,
        yaw_error: torch.Tensor,
    ) -> torch.Tensor:
        if not self.cfg.enabled:
            return torch.zeros_like(dx_body, dtype=torch.bool)

        iou = bbox_iou_from_body_pose(
            dx_body,
            dy_body,
            yaw_error,
            robot_length=self.cfg.robot_length,
            robot_width=self.cfg.robot_width,
            spot_length=self.cfg.spot_length,
            spot_width=self.cfg.spot_width,
        )
        self.current_iou = iou
        success_mask = iou >= self.cfg.iou_threshold
        if self.cfg.theta_threshold is not None:
            success_mask = success_mask & (yaw_error.abs() <= self.cfg.theta_threshold)

        self._counter = torch.where(
            success_mask,
            self._counter + 1.0,
            torch.zeros_like(self._counter),
        )
        self._max_counter = torch.maximum(self._max_counter, self._counter)
        return self._counter >= float(self.cfg.num_success)

    @property
    def consecutive_success(self) -> torch.Tensor:
        return self._counter
