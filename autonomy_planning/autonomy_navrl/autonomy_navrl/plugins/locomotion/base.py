"""Low-level locomotion controllers (root kinematics or JIT policies)."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

import torch

from autonomy_navrl.core.locomotion_spec import LocomotionSpec
from autonomy_navrl.core.registry import Registry

if TYPE_CHECKING:
    from autonomy_navrl.env.isaac.direct_env import NavrlDirectEnv

locomotion_registry: Registry[type['BaseLocomotionController']] = Registry('locomotion')


class BaseLocomotionController(ABC):
    """Apply high-level velocity commands to the simulated robot."""

    name: str

    def __init__(self, spec: LocomotionSpec, env: NavrlDirectEnv) -> None:
        self.spec = spec
        self.env = env

    @abstractmethod
    def reset(self, env_ids: torch.Tensor) -> None:
        raise NotImplementedError

    @abstractmethod
    def apply(self) -> None:
        """Called every physics sub-step from DirectRLEnv._apply_action."""

    def velocity_command(self) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        adapter = self.env._action_adapter
        return getattr(adapter, '_vx'), getattr(adapter, '_vy'), getattr(adapter, '_w')


def create_locomotion_controller(spec: LocomotionSpec, env: NavrlDirectEnv) -> BaseLocomotionController:
    from autonomy_navrl.plugins.bootstrap import ensure_plugins

    ensure_plugins()
    return locomotion_registry.create(spec.backend, spec=spec, env=env)
