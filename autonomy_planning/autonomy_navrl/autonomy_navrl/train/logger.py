"""TensorBoard logging helpers."""

from __future__ import annotations

from pathlib import Path
from typing import Any


class TensorboardLogger:
    """Thin wrapper around torch.utils.tensorboard.SummaryWriter."""

    def __init__(self, log_dir: str | Path) -> None:
        from torch.utils.tensorboard import SummaryWriter

        self._writer = SummaryWriter(str(log_dir))

    def log_scalars(self, step: int, metrics: dict[str, float]) -> None:
        for key, value in metrics.items():
            self._writer.add_scalar(key, value, step)

    def log_dict(self, step: int, prefix: str, metrics: dict[str, Any]) -> None:
        flattened = {
            f'{prefix}/{key}': float(value)
            for key, value in metrics.items()
            if isinstance(value, (int, float))
        }
        self.log_scalars(step, flattened)

    def log_rgbd_preview(self, step: int, rgbd: Any, tag: str = 'train/rgbd_preview') -> None:
        """Log first env RGB + depth as a 2-row image grid."""
        import torch

        tensor = torch.as_tensor(rgbd)
        if tensor.ndim != 4:
            return
        rgb = tensor[0, :3].clamp(0.0, 1.0)
        depth = tensor[0, 3:4].clamp(0.0, 1.0).repeat(3, 1, 1)
        grid = torch.cat([rgb, depth], dim=1)
        self._writer.add_image(tag, grid, global_step=step)

    def close(self) -> None:
        self._writer.close()
