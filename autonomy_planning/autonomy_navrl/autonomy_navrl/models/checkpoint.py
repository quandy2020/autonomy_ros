"""Checkpoint save/load helpers."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import torch
from torch import nn


def save_checkpoint(
    path: Path,
    *,
    policy: nn.Module,
    optimizer: torch.optim.Optimizer,
    config: dict[str, Any],
    global_step: int,
    algorithm: str = 'ppo',
) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    model_raw = config.get('model', {})
    policy_type = model_raw.get('policy_type', config.get('policy', {}).get('type', 'gaussian_actor_critic'))
    torch.save(
        {
            'policy_state_dict': policy.state_dict(),
            'optimizer_state_dict': optimizer.state_dict(),
            'config': config,
            'global_step': global_step,
            'algorithm': algorithm,
            'policy_type': policy_type,
        },
        path,
    )
    return path


def load_checkpoint_payload(path: Path, device: torch.device) -> dict[str, Any]:
    try:
        return torch.load(path, map_location=device, weights_only=False)
    except TypeError:
        return torch.load(path, map_location=device)
