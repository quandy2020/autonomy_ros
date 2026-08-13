"""Orchestrates modular Isaac pose / heading visualization."""

from __future__ import annotations

import torch

from autonomy_navrl.viz.isaac.config import PoseVizConfig
from autonomy_navrl.viz.isaac.context import PoseVizContext
from autonomy_navrl.viz.isaac.modules import PoseVizModule, build_modules


class PoseVizManager:
    """Update registered visualization modules from env pose tensors."""

    def __init__(self, cfg: PoseVizConfig, device: torch.device | str) -> None:
        self.cfg = cfg
        self.device = device
        self._modules: list[PoseVizModule] = build_modules(cfg, device)

    @property
    def module_names(self) -> list[str]:
        return [module.name for module in self._modules]

    def set_visible(self, visible: bool) -> None:
        for module in self._modules:
            module.set_visible(visible)

    def update(self, ctx: PoseVizContext) -> None:
        for module in self._modules:
            module.update(ctx)


def build_pose_viz_manager(cfg: PoseVizConfig | None, device: torch.device | str) -> PoseVizManager | None:
    if cfg is None or not cfg.enabled:
        return None
    return PoseVizManager(cfg, device)
