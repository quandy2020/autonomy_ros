"""Algorithm factory — build policy + trainer from config."""

from __future__ import annotations

from typing import Any

import torch

from autonomy_navrl.algorithms.base import BaseAlgorithm
from autonomy_navrl.algorithms.ppo import PpoAlgorithm, algorithm_registry
from autonomy_navrl.env.base_env import BaseNavrlEnv
from autonomy_navrl.models.factory import create_nav_policy


def resolve_algorithm_name(config: dict[str, Any]) -> str:
    """Read algorithm name from config (``algorithm.name`` or legacy default)."""
    algo_raw = config.get('algorithm', {})
    return str(algo_raw.get('name', 'ppo')).lower()


def create_algorithm(
    env: BaseNavrlEnv,
    config: dict[str, Any],
    device: str = 'cuda:0',
) -> BaseAlgorithm:
    """Build policy + registered training algorithm from config."""
    import autonomy_navrl.algorithms.diffusion  # noqa: F401
    import autonomy_navrl.algorithms.grpo  # noqa: F401

    name = resolve_algorithm_name(config)
    torch_device = torch.device(device if torch.cuda.is_available() else 'cpu')
    policy = create_nav_policy(config, device=torch_device)
    return algorithm_registry.create(name, env=env, policy=policy, config=config, device=device)


__all__ = [
    'BaseAlgorithm',
    'PpoAlgorithm',
    'algorithm_registry',
    'create_algorithm',
    'resolve_algorithm_name',
]
