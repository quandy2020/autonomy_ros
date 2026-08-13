"""On-policy rollout storage."""

from __future__ import annotations

from dataclasses import dataclass

import torch


@dataclass
class RolloutBuffer:
    """Stores vectorized on-policy rollout data."""

    rgbd: torch.Tensor
    state: torch.Tensor
    actions: torch.Tensor
    log_probs: torch.Tensor
    values: torch.Tensor
    rewards: torch.Tensor
    dones: torch.Tensor

    def as_obs_dict(self) -> dict[str, torch.Tensor]:
        """Flatten time × env into batch observation dict."""
        timesteps, num_envs = self.rewards.shape
        flat_size = timesteps * num_envs
        return {
            'rgbd': self.rgbd.reshape(flat_size, *self.rgbd.shape[2:]),
            'state': self.state.reshape(flat_size, self.state.shape[-1]),
        }
