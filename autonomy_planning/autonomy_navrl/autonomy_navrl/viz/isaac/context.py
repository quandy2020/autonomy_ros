"""Shared tensors for Isaac pose visualization modules."""

from __future__ import annotations

from dataclasses import dataclass

import torch


@dataclass
class PoseVizContext:
    """World-frame pose data passed to visualization modules each frame."""

    robot_pos_w: torch.Tensor
    robot_yaw: torch.Tensor
    target_pos_w: torch.Tensor
    target_yaw: torch.Tensor
    goal_tolerance_m: float
    commands: torch.Tensor
    max_vx: float
    max_vy: float
    max_w: float
    spot_length: float = 0.9
    spot_width: float = 0.3
    robot_length: float = 0.9
    robot_width: float = 0.3
