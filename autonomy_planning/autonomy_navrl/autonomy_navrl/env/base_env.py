"""Abstract environment interface for Isaac Lab RL training."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass
class EnvStepResult:
    """Single environment step output."""

    observation: dict[str, np.ndarray]
    reward: np.ndarray
    terminated: np.ndarray
    truncated: np.ndarray
    info: dict[str, Any]


class BaseNavrlEnv(ABC):
    """Vectorized navigation environment interface."""

    @property
    @abstractmethod
    def num_envs(self) -> int:
        """Number of parallel environments."""

    @property
    @abstractmethod
    def action_dim(self) -> int:
        """Action dimension."""

    @abstractmethod
    def reset(self) -> dict[str, np.ndarray]:
        """Reset all environments and return initial observations."""

    @abstractmethod
    def step(self, actions: np.ndarray) -> EnvStepResult:
        """Apply actions and return step results."""

    @abstractmethod
    def close(self) -> None:
        """Release simulator resources."""
