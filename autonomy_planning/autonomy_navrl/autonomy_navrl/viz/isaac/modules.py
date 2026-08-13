"""Pluggable Isaac Sim visualization modules for pose / heading."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

import torch
from isaaclab.markers import VisualizationMarkers
from isaaclab.utils.math import quat_from_euler_xyz

from autonomy_navrl.viz.isaac.context import PoseVizContext
from autonomy_navrl.viz.isaac.markers import (
    arrow_marker_cfg,
    footprint_rect_cfg,
    frame_marker_cfg,
    sphere_marker_cfg,
    tolerance_disk_cfg,
)

if TYPE_CHECKING:
    from autonomy_navrl.viz.isaac.config import PoseVizConfig


class PoseVizModule(ABC):
    """One visual layer (target frame, robot arrow, etc.)."""

    name: str

    def __init__(self, cfg: PoseVizConfig, device: torch.device | str) -> None:
        self._cfg = cfg
        self._device = device
        self._marker: VisualizationMarkers | None = None

    @abstractmethod
    def _ensure_marker(self) -> VisualizationMarkers:
        raise NotImplementedError

    @abstractmethod
    def update(self, ctx: PoseVizContext) -> None:
        raise NotImplementedError

    def set_visible(self, visible: bool) -> None:
        if self._marker is not None:
            self._marker.set_visibility(visible)


def _yaw_quat_wxyz(yaw: torch.Tensor) -> torch.Tensor:
    zeros = torch.zeros_like(yaw)
    return quat_from_euler_xyz(zeros, zeros, yaw)


class TargetPoseFrameModule(PoseVizModule):
    name = 'target_pose_frame'

    def _ensure_marker(self) -> VisualizationMarkers:
        if self._marker is None:
            self._marker = VisualizationMarkers(
                frame_marker_cfg('/Visuals/Navrl/target_pose_frame', self._cfg.frame_scale)
            )
        return self._marker

    def update(self, ctx: PoseVizContext) -> None:
        marker = self._ensure_marker()
        marker.visualize(
            translations=ctx.target_pos_w,
            orientations=_yaw_quat_wxyz(ctx.target_yaw),
        )


class RobotPoseFrameModule(PoseVizModule):
    name = 'robot_pose_frame'

    def _ensure_marker(self) -> VisualizationMarkers:
        if self._marker is None:
            self._marker = VisualizationMarkers(
                frame_marker_cfg('/Visuals/Navrl/robot_pose_frame', self._cfg.frame_scale * 0.85)
            )
        return self._marker

    def update(self, ctx: PoseVizContext) -> None:
        marker = self._ensure_marker()
        marker.visualize(
            translations=ctx.robot_pos_w,
            orientations=_yaw_quat_wxyz(ctx.robot_yaw),
        )


class TargetFootprintModule(PoseVizModule):
    """Target parking spot as a flat rectangle (0.9 x 0.3 m for Go2W)."""

    name = 'target_footprint'

    def _ensure_marker(self) -> VisualizationMarkers:
        if self._marker is None:
            self._marker = VisualizationMarkers(
                footprint_rect_cfg('/Visuals/Navrl/target_footprint', (0.15, 0.92, 0.25))
            )
        return self._marker

    def update(self, ctx: PoseVizContext) -> None:
        marker = self._ensure_marker()
        height = self._cfg.footprint_height
        scales = torch.zeros(ctx.target_pos_w.shape[0], 3, device=self._device)
        scales[:, 0] = ctx.spot_length
        scales[:, 1] = ctx.spot_width
        scales[:, 2] = height
        pos = ctx.target_pos_w.clone()
        pos[:, 2] += height * 0.5
        marker.visualize(
            translations=pos,
            orientations=_yaw_quat_wxyz(ctx.target_yaw),
            scales=scales,
        )


class RobotFootprintModule(PoseVizModule):
    """Robot body footprint as a flat rectangle."""

    name = 'robot_footprint'

    def _ensure_marker(self) -> VisualizationMarkers:
        if self._marker is None:
            self._marker = VisualizationMarkers(
                footprint_rect_cfg('/Visuals/Navrl/robot_footprint', (0.2, 0.55, 1.0))
            )
        return self._marker

    def update(self, ctx: PoseVizContext) -> None:
        marker = self._ensure_marker()
        height = self._cfg.footprint_height
        scales = torch.zeros(ctx.robot_pos_w.shape[0], 3, device=self._device)
        scales[:, 0] = ctx.robot_length
        scales[:, 1] = ctx.robot_width
        scales[:, 2] = height
        pos = ctx.robot_pos_w.clone()
        pos[:, 2] += height * 0.5
        marker.visualize(
            translations=pos,
            orientations=_yaw_quat_wxyz(ctx.robot_yaw),
            scales=scales,
        )


class TargetSphereModule(PoseVizModule):
    name = 'target_sphere'

    def _ensure_marker(self) -> VisualizationMarkers:
        if self._marker is None:
            self._marker = VisualizationMarkers(
                sphere_marker_cfg('/Visuals/Navrl/target_sphere', 0.06, (0.1, 0.95, 0.2))
            )
        return self._marker

    def update(self, ctx: PoseVizContext) -> None:
        marker = self._ensure_marker()
        marker.visualize(translations=ctx.target_pos_w)


class TargetHeadingArrowModule(PoseVizModule):
    name = 'target_heading_arrow'

    def _ensure_marker(self) -> VisualizationMarkers:
        if self._marker is None:
            self._marker = VisualizationMarkers(
                arrow_marker_cfg('/Visuals/Navrl/target_heading_arrow', (0.1, 0.95, 0.2))
            )
        return self._marker

    def update(self, ctx: PoseVizContext) -> None:
        marker = self._ensure_marker()
        length = self._cfg.arrow_length
        scales = torch.zeros(ctx.target_pos_w.shape[0], 3, device=self._device)
        scales[:, 0] = length
        scales[:, 1] = 1.0
        scales[:, 2] = 1.0
        marker.visualize(
            translations=ctx.target_pos_w,
            orientations=_yaw_quat_wxyz(ctx.target_yaw),
            scales=scales,
        )


class RobotHeadingArrowModule(PoseVizModule):
    name = 'robot_heading_arrow'

    def _ensure_marker(self) -> VisualizationMarkers:
        if self._marker is None:
            self._marker = VisualizationMarkers(
                arrow_marker_cfg('/Visuals/Navrl/robot_heading_arrow', (0.2, 0.55, 1.0))
            )
        return self._marker

    def update(self, ctx: PoseVizContext) -> None:
        marker = self._ensure_marker()
        length = self._cfg.arrow_length * 0.85
        scales = torch.zeros(ctx.robot_pos_w.shape[0], 3, device=self._device)
        scales[:, 0] = length
        scales[:, 1] = 1.0
        scales[:, 2] = 1.0
        marker.visualize(
            translations=ctx.robot_pos_w,
            orientations=_yaw_quat_wxyz(ctx.robot_yaw),
            scales=scales,
        )


class PositionToleranceModule(PoseVizModule):
    name = 'position_tolerance'

    def _ensure_marker(self) -> VisualizationMarkers:
        if self._marker is None:
            self._marker = VisualizationMarkers(tolerance_disk_cfg('/Visuals/Navrl/position_tolerance'))
        return self._marker

    def update(self, ctx: PoseVizContext) -> None:
        marker = self._ensure_marker()
        radius = max(float(ctx.goal_tolerance_m), 1e-3)
        scales = torch.zeros(ctx.target_pos_w.shape[0], 3, device=self._device)
        scales[:, 0] = radius
        scales[:, 1] = radius
        scales[:, 2] = 1.0
        pos = ctx.target_pos_w.clone()
        pos[:, 2] -= 0.01
        marker.visualize(translations=pos, scales=scales)


class VelocityCommandModule(PoseVizModule):
    name = 'velocity_command'

    def _ensure_marker(self) -> VisualizationMarkers:
        if self._marker is None:
            self._marker = VisualizationMarkers(
                arrow_marker_cfg('/Visuals/Navrl/velocity_command', (1.0, 0.45, 0.1))
            )
        return self._marker

    def update(self, ctx: PoseVizContext) -> None:
        marker = self._ensure_marker()
        vx = ctx.commands[:, 0] * ctx.max_vx
        vy = ctx.commands[:, 1] * ctx.max_vy
        speed = torch.hypot(vx, vy)
        yaw_cmd = torch.atan2(vy, vx)
        yaw_cmd = torch.where(speed > 1e-4, yaw_cmd, ctx.robot_yaw)
        length = torch.clamp(speed, min=0.08, max=self._cfg.arrow_length)
        scales = torch.stack([length, torch.ones_like(length), torch.ones_like(length)], dim=-1)
        marker.visualize(
            translations=ctx.robot_pos_w,
            orientations=_yaw_quat_wxyz(yaw_cmd),
            scales=scales,
        )


MODULE_CLASSES: dict[str, type[PoseVizModule]] = {
    TargetPoseFrameModule.name: TargetPoseFrameModule,
    RobotPoseFrameModule.name: RobotPoseFrameModule,
    TargetSphereModule.name: TargetSphereModule,
    TargetFootprintModule.name: TargetFootprintModule,
    RobotFootprintModule.name: RobotFootprintModule,
    TargetHeadingArrowModule.name: TargetHeadingArrowModule,
    RobotHeadingArrowModule.name: RobotHeadingArrowModule,
    PositionToleranceModule.name: PositionToleranceModule,
    VelocityCommandModule.name: VelocityCommandModule,
}


def build_modules(cfg: PoseVizConfig, device: torch.device | str) -> list[PoseVizModule]:
    modules: list[PoseVizModule] = []
    for name in cfg.modules:
        module_cls = MODULE_CLASSES[name]
        modules.append(module_cls(cfg, device))
    return modules
