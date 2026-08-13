"""Proprioceptive / task state encoder."""

from __future__ import annotations

import torch
from torch import nn


class StateEncoder(nn.Module):
    """Encode low-dimensional state vectors."""

    def __init__(self, state_dim: int, hidden_dim: int) -> None:
        super().__init__()
        self._mlp = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(inplace=True),
        )
        self.output_dim = hidden_dim

    def forward(self, state: torch.Tensor) -> torch.Tensor:
        return self._mlp(state)
