"""Action adapters: normalized actions -> sim root velocity."""

from __future__ import annotations

from abc import ABC, abstractmethod

import torch

from autonomy_navrl.control.commands import body_velocity_to_world
from autonomy_navrl.core.registry import Registry
from autonomy_navrl.core.spec import ActionSpec

action_registry: Registry['BaseActionAdapter'] = Registry('action_model')


class BaseActionAdapter(ABC):
    """Maps policy actions to body-frame velocity commands."""

    def __init__(self, spec: ActionSpec, num_envs: int, device: torch.device | str) -> None:
        self.spec = spec
        self.num_envs = num_envs
        self.device = device
        self._commands = torch.zeros(num_envs, spec.dim, device=device)

    @property
    def dim(self) -> int:
        return self.spec.dim

    @property
    def commands(self) -> torch.Tensor:
        return self._commands

    @abstractmethod
    def pre_physics_step(self, actions: torch.Tensor) -> None:
        """Store and scale actions."""

    @abstractmethod
    def root_velocity_world(self, yaw: torch.Tensor) -> torch.Tensor:
        """Return (N, 6) root velocity for write_root_com_velocity_to_sim."""


@action_registry.register('holonomic_3d')
class Holonomic3DAction(BaseActionAdapter):
    """Body-frame (vx, vy, w) for quadruped / wheeled-legged / humanoid."""

    def pre_physics_step(self, actions: torch.Tensor) -> None:
        self._commands = actions.clone()
        limits = actions.new_tensor([self.spec.max_vx, self.spec.max_vy, self.spec.max_w])
        # Isaac Lab convention: clamp normalized action then scale to m/s (no extra tanh).
        scaled = torch.clamp(actions, -1.0, 1.0) * limits
        self._vx = scaled[:, 0]
        self._vy = scaled[:, 1]
        self._w = scaled[:, 2]

    def apply_smoothing(self, vx: torch.Tensor, vy: torch.Tensor, w: torch.Tensor):
        self._vx, self._vy, self._w = vx, vy, w

    def root_velocity_world(self, yaw: torch.Tensor) -> torch.Tensor:
        return body_velocity_to_world(self._vx, self._vy, self._w, yaw)


@action_registry.register('diff_drive_2d')
class DiffDrive2DAction(BaseActionAdapter):
    """Unicycle (v, w) for differential-drive mobile bases."""

    def pre_physics_step(self, actions: torch.Tensor) -> None:
        self._commands = actions.clone()
        scaled = torch.clamp(actions, -1.0, 1.0)
        self._vx = scaled[:, 0] * self.spec.max_vx
        self._vy = torch.zeros_like(self._vx)
        self._w = scaled[:, 1] * self.spec.max_w

    def apply_smoothing(self, vx: torch.Tensor, vy: torch.Tensor, w: torch.Tensor):
        self._vx, self._vy, self._w = vx, vy, w

    def root_velocity_world(self, yaw: torch.Tensor) -> torch.Tensor:
        return body_velocity_to_world(self._vx, self._vy, self._w, yaw)


def create_action_adapter(
    spec: ActionSpec,
    num_envs: int,
    device: torch.device | str,
) -> BaseActionAdapter:
    return action_registry.create(spec.model, spec=spec, num_envs=num_envs, device=device)
