"""Build navigation policies from YAML config."""

from __future__ import annotations

from typing import Any

import torch

from autonomy_navrl.models.policies.gaussian import GaussianActorCriticPolicy, policy_registry
from autonomy_navrl.models.policies.base import NavPolicy
from autonomy_navrl.models.spec import ModelSpec


def create_nav_policy(config: dict[str, Any], device: torch.device | str | None = None) -> NavPolicy:
    """Instantiate a policy from training/deploy config."""
    spec = ModelSpec.from_config(config)
    policy = policy_registry.create(spec.policy_type, spec=spec)
    if device is not None:
        policy = policy.to(device)
    return policy


# Backward-compatible alias
ActorCriticPolicy = GaussianActorCriticPolicy

__all__ = ['ActorCriticPolicy', 'create_nav_policy', 'policy_registry']
