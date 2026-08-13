"""Go2W 2D bounding-box IoU (aligned with Isaac Lab unitree_go2w)."""

from __future__ import annotations

import torch

# Official Go2W precision navigation dimensions (m).
GO2W_ROBOT_LENGTH = 0.9
GO2W_ROBOT_WIDTH = 0.3
GO2W_SPOT_LENGTH = 0.9
GO2W_SPOT_WIDTH = 0.3


def compute_iou_2d(
    box1_min: torch.Tensor,
    box1_max: torch.Tensor,
    box2_min: torch.Tensor,
    box2_max: torch.Tensor,
) -> torch.Tensor:
    inter_min = torch.max(box1_min, box2_min)
    inter_max = torch.min(box1_max, box2_max)
    inter_dims = torch.clamp(inter_max - inter_min, min=0.0)
    inter_area = inter_dims[:, 0] * inter_dims[:, 1]
    box1_area = (box1_max[:, 0] - box1_min[:, 0]) * (box1_max[:, 1] - box1_min[:, 1])
    box2_area = (box2_max[:, 0] - box2_min[:, 0]) * (box2_max[:, 1] - box2_min[:, 1])
    union_area = box1_area + box2_area - inter_area
    return inter_area / (union_area + 1e-8)


def robot_bbox_base_frame(
    num_envs: int,
    device: torch.device,
    dtype: torch.dtype,
    *,
    robot_length: float = GO2W_ROBOT_LENGTH,
    robot_width: float = GO2W_ROBOT_WIDTH,
) -> tuple[torch.Tensor, torch.Tensor]:
    half_l = robot_length * 0.5
    half_w = robot_width * 0.5
    box_min = torch.tensor([[-half_l, -half_w]], device=device, dtype=dtype).expand(num_envs, -1)
    box_max = torch.tensor([[half_l, half_w]], device=device, dtype=dtype).expand(num_envs, -1)
    return box_min, box_max


def spot_bbox_base_frame(
    command: torch.Tensor,
    *,
    spot_length: float = GO2W_SPOT_LENGTH,
    spot_width: float = GO2W_SPOT_WIDTH,
) -> tuple[torch.Tensor, torch.Tensor]:
    """command: (N, 4) = [rel_x, rel_y, rel_z, rel_heading] in body frame."""
    num_envs = command.shape[0]
    device = command.device
    dtype = command.dtype
    target_pos = command[:, :2]
    target_heading = command[:, 3]

    half_l = spot_length * 0.5
    half_w = spot_width * 0.5
    corners_spot = torch.tensor(
        [[-half_l, -half_w], [half_l, -half_w], [half_l, half_w], [-half_l, half_w]],
        device=device,
        dtype=dtype,
    )
    cos_h = torch.cos(target_heading).unsqueeze(1).unsqueeze(2)
    sin_h = torch.sin(target_heading).unsqueeze(1).unsqueeze(2)
    corners = corners_spot.unsqueeze(0).expand(num_envs, -1, -1)
    x = corners[:, :, 0:1]
    y = corners[:, :, 1:2]
    x_b = x * cos_h - y * sin_h
    y_b = x * sin_h + y * cos_h
    corners_base = torch.cat([x_b, y_b], dim=2) + target_pos.unsqueeze(1)
    return torch.min(corners_base, dim=1)[0], torch.max(corners_base, dim=1)[0]


def bbox_iou_from_body_pose(
    dx_body: torch.Tensor,
    dy_body: torch.Tensor,
    yaw_error: torch.Tensor,
    *,
    robot_length: float = GO2W_ROBOT_LENGTH,
    robot_width: float = GO2W_ROBOT_WIDTH,
    spot_length: float = GO2W_SPOT_LENGTH,
    spot_width: float = GO2W_SPOT_WIDTH,
) -> torch.Tensor:
    """Compute 2D bbox IoU between robot (at origin) and target spot in body frame."""
    num_envs = dx_body.shape[0]
    device = dx_body.device
    dtype = dx_body.dtype
    command = torch.stack([dx_body, dy_body, torch.zeros_like(dx_body), yaw_error], dim=-1)
    robot_min, robot_max = robot_bbox_base_frame(
        num_envs, device, dtype, robot_length=robot_length, robot_width=robot_width,
    )
    spot_min, spot_max = spot_bbox_base_frame(
        command, spot_length=spot_length, spot_width=spot_width,
    )
    return compute_iou_2d(robot_min, robot_max, spot_min, spot_max)
