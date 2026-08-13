"""Body-frame pose and goal-relative feature math (torch)."""

from __future__ import annotations

import torch


def body_goal_features(
    goals_xy: torch.Tensor,
    goal_yaws: torch.Tensor,
    pos_xy: torch.Tensor,
    yaw: torch.Tensor,
    yaw_error_fn,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """Return ``dx_body, dy_body, distance, bearing, yaw_err``."""
    goal_delta = goals_xy - pos_xy
    distance = torch.linalg.norm(goal_delta, dim=-1)
    bearing = torch.atan2(goal_delta[:, 1], goal_delta[:, 0]) - yaw
    bearing = torch.atan2(torch.sin(bearing), torch.cos(bearing))
    cos_yaw = torch.cos(yaw)
    sin_yaw = torch.sin(yaw)
    dx_body = cos_yaw * goal_delta[:, 0] + sin_yaw * goal_delta[:, 1]
    dy_body = -sin_yaw * goal_delta[:, 0] + cos_yaw * goal_delta[:, 1]
    yaw_err = yaw_error_fn()
    return dx_body, dy_body, distance, bearing, yaw_err
