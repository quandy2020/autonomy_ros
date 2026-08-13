"""Diffusion policy training (placeholder)."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from autonomy_navrl.algorithms.base import BaseAlgorithm
from autonomy_navrl.algorithms.ppo import algorithm_registry
from autonomy_navrl.env.base_env import BaseNavrlEnv
from autonomy_navrl.models.policies.base import NavPolicy


@algorithm_registry.register('diffusion')
class DiffusionAlgorithm(BaseAlgorithm):
    """Diffusion policy trainer stub."""

    name = 'diffusion'

    def __init__(
        self,
        env: BaseNavrlEnv,
        policy: NavPolicy,
        config: dict[str, Any],
        device: str = 'cuda:0',
    ) -> None:
        super().__init__(env, policy, config, device)
        raise NotImplementedError(
            'Diffusion training is not implemented yet. Set algorithm.name to "ppo".'
        )

    def train(self) -> Path:
        raise NotImplementedError

    def save_checkpoint(self, path: Path) -> Path:
        raise NotImplementedError

    def load_checkpoint(self, path: Path) -> int:
        raise NotImplementedError
