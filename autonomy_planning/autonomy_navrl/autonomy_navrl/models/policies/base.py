"""Policy base types."""

from __future__ import annotations

from abc import ABC, abstractmethod

import torch
from torch import nn


class NavPolicy(nn.Module, ABC):
    """Algorithm-agnostic navigation policy interface."""

    @abstractmethod
    def act(
        self,
        obs: dict[str, torch.Tensor],
        deterministic: bool = False,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """Return action, log_prob, value, action_mean."""

    @abstractmethod
    def evaluate_actions(
        self,
        obs: dict[str, torch.Tensor],
        actions: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Return log_prob, value, entropy."""

    def act_tensors(
        self,
        rgbd: torch.Tensor,
        state: torch.Tensor,
        deterministic: bool = False,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """Legacy tensor API for deploy and older call sites."""
        return self.act({'rgbd': rgbd, 'state': state}, deterministic=deterministic)

    def evaluate_action_tensors(
        self,
        rgbd: torch.Tensor,
        state: torch.Tensor,
        actions: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Legacy tensor API for rollout updates."""
        return self.evaluate_actions({'rgbd': rgbd, 'state': state}, actions)
