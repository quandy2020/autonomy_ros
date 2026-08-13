"""Training algorithm base types."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from autonomy_navrl.env.base_env import BaseNavrlEnv
from autonomy_navrl.models.policies.base import NavPolicy


@dataclass(frozen=True)
class TrainResult:
    """Outcome of a training run."""

    checkpoint: Path
    global_step: int


class BaseAlgorithm(ABC):
    """Algorithm-agnostic trainer (PPO, GRPO, diffusion, …)."""

    name: str

    def __init__(
        self,
        env: BaseNavrlEnv,
        policy: NavPolicy,
        config: dict[str, Any],
        device: str = 'cuda:0',
    ) -> None:
        self._env = env
        self._policy = policy
        self._config = config
        self._device_str = device

    @abstractmethod
    def train(self) -> Path:
        """Run training and return final checkpoint path."""

    @abstractmethod
    def save_checkpoint(self, path: Path) -> Path:
        ...

    @abstractmethod
    def load_checkpoint(self, path: Path) -> int:
        ...
