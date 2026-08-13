"""Body-frame velocity command scaling, clipping, and Isaac application."""

from __future__ import annotations

import numpy as np
import torch

from autonomy_navrl.core.types import VelocityCommand


def clip_velocity_command(
    command: VelocityCommand,
    max_vx: float,
    max_vy: float,
    max_w: float,
) -> VelocityCommand:
    """Clip velocity command to configured limits."""
    return VelocityCommand(
        vx=float(np.clip(command.vx, -max_vx, max_vx)),
        vy=float(np.clip(command.vy, -max_vy, max_vy)),
        w=float(np.clip(command.w, -max_w, max_w)),
    )


def scale_action_to_command(
    action: np.ndarray,
    max_vx: float,
    max_vy: float,
    max_w: float,
) -> VelocityCommand:
    """Map normalized action in [-1, 1] to physical velocity command."""
    action = np.asarray(action, dtype=np.float32).reshape(-1)
    if action.shape[0] != 3:
        raise ValueError(f'Expected action dim 3, got {action.shape}')
    scaled = np.clip(action, -1.0, 1.0) * np.array([max_vx, max_vy, max_w], dtype=np.float32)
    return VelocityCommand(vx=float(scaled[0]), vy=float(scaled[1]), w=float(scaled[2]))


def body_velocity_to_world(
    vx: torch.Tensor,
    vy: torch.Tensor,
    w: torch.Tensor,
    yaw: torch.Tensor,
) -> torch.Tensor:
    """Convert body-frame (vx, vy, w) to world-frame root velocity [vx, vy, vz, wx, wy, wz]."""
    cos_yaw = torch.cos(yaw)
    sin_yaw = torch.sin(yaw)
    world_vx = cos_yaw * vx - sin_yaw * vy
    world_vy = sin_yaw * vx + cos_yaw * vy
    zeros = torch.zeros_like(vx)
    return torch.stack([world_vx, world_vy, zeros, zeros, zeros, w], dim=-1)


def scale_actions_to_velocity(
    actions: torch.Tensor,
    max_vx: float,
    max_vy: float,
    max_w: float,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Map normalized actions in [-1, 1] to body-frame velocities."""
    scaled = torch.tanh(actions) * actions.new_tensor([max_vx, max_vy, max_w])
    return scaled[:, 0], scaled[:, 1], scaled[:, 2]
