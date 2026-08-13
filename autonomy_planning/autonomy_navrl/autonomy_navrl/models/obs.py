"""Observation dict ↔ torch batch helpers."""

from __future__ import annotations

from typing import Any

import numpy as np
import torch


def obs_to_torch(obs: dict[str, Any], device: torch.device) -> dict[str, torch.Tensor]:
    """Convert env observation dict to torch tensors on ``device``."""
    out: dict[str, torch.Tensor] = {}
    for key, value in obs.items():
        if key == 'sensor_backend' or value is None:
            continue
        if isinstance(value, np.ndarray):
            out[key] = torch.as_tensor(value, device=device)
        elif isinstance(value, torch.Tensor):
            out[key] = value.to(device)
    return out
