"""Gaussian actor-critic policy."""

from __future__ import annotations

from typing import Sequence

import torch
from torch import nn
from torch.distributions import Normal

from autonomy_navrl.core.registry import Registry
from autonomy_navrl.models.encoders.fusion import create_fusion_encoder
from autonomy_navrl.models.policies.base import NavPolicy
from autonomy_navrl.models.spec import ModelSpec

policy_registry: Registry[type['GaussianActorCriticPolicy']] = Registry('policy')


@policy_registry.register('gaussian_actor_critic')
class GaussianActorCriticPolicy(NavPolicy):
    """Gaussian actor and scalar critic over fused multimodal observations."""

    name = 'gaussian_actor_critic'

    def __init__(
        self,
        spec: ModelSpec | None = None,
        *,
        input_channels: int = 4,
        image_height: int = 96,
        image_width: int = 128,
        cnn_channels: Sequence[int] = (32, 64, 64),
        cnn_kernel_sizes: Sequence[int] = (8, 4, 3),
        cnn_strides: Sequence[int] = (4, 2, 1),
        state_dim: int = 20,
        hidden_dim: int = 256,
        action_dim: int = 3,
    ) -> None:
        super().__init__()
        if spec is None:
            spec = ModelSpec(
                action_dim=action_dim,
                hidden_dim=hidden_dim,
                state_dim=state_dim,
                image_height=image_height,
                image_width=image_width,
                rgbd_channels=input_channels,
                cnn_channels=tuple(cnn_channels),
                cnn_kernel_sizes=tuple(cnn_kernel_sizes),
                cnn_strides=tuple(cnn_strides),
            )
        self._spec = spec
        self._encoder = create_fusion_encoder(spec)
        self._actor_mean = nn.Linear(spec.hidden_dim, spec.action_dim)
        self._actor_log_std = nn.Parameter(torch.zeros(spec.action_dim))
        self._critic = nn.Linear(spec.hidden_dim, 1)

    def encode(self, obs: dict[str, torch.Tensor]) -> torch.Tensor:
        return self._encoder(obs)

    def forward(
        self,
        rgbd: torch.Tensor,
        state: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Legacy forward for backward-compatible call sites."""
        features = self.encode({'rgbd': rgbd, 'state': state})
        value = self._critic(features).squeeze(-1)
        mean = self._actor_mean(features)
        return mean, value

    def act(
        self,
        obs: dict[str, torch.Tensor],
        deterministic: bool = False,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        features = self.encode(obs)
        mean = self._actor_mean(features)
        value = self._critic(features).squeeze(-1)
        std = self._actor_log_std.exp().expand_as(mean)
        dist = Normal(mean, std)
        action = mean if deterministic else dist.sample()
        log_prob = dist.log_prob(action).sum(dim=-1)
        return action, log_prob, value, mean

    def evaluate_actions(
        self,
        obs: dict[str, torch.Tensor],
        actions: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        features = self.encode(obs)
        mean = self._actor_mean(features)
        value = self._critic(features).squeeze(-1)
        std = self._actor_log_std.exp().expand_as(mean)
        dist = Normal(mean, std)
        log_prob = dist.log_prob(actions).sum(dim=-1)
        entropy = dist.entropy().sum(dim=-1)
        return log_prob, value, entropy
