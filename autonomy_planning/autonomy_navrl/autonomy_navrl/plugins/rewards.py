"""Reward computation plugins for Isaac navigation MDP."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING

import torch

from autonomy_navrl.core.registry import Registry

if TYPE_CHECKING:
    from autonomy_navrl.env.isaac.direct_env import NavrlDirectEnv

reward_registry: Registry[type['BaseRewardComputer']] = Registry('reward_profile')


@dataclass
class RewardStepContext:
    """Tensors shared by all reward plugins each control step."""

    distance: torch.Tensor
    progress: torch.Tensor
    proximity: torch.Tensor
    bearing: torch.Tensor
    yaw_error: torch.Tensor
    reached: torch.Tensor
    commands: torch.Tensor
    prev_commands: torch.Tensor
    lin_speed_b: torch.Tensor
    collision_terminated: torch.Tensor
    timeout: torch.Tensor


class BaseRewardComputer(ABC):
    """Compute scalar reward vector (N,) and metric dict."""

    name: str

    def __init__(self, env: NavrlDirectEnv) -> None:
        self._env = env

    @abstractmethod
    def reset(self, env_ids: torch.Tensor) -> None:
        raise NotImplementedError

    @abstractmethod
    def compute(self, ctx: RewardStepContext) -> tuple[torch.Tensor, dict[str, float]]:
        raise NotImplementedError


@reward_registry.register('default')
class DefaultRewardComputer(BaseRewardComputer):
    name = 'default'

    def reset(self, env_ids: torch.Tensor) -> None:
        del env_ids

    def compute(self, ctx: RewardStepContext) -> tuple[torch.Tensor, dict[str, float]]:
        cfg = self._env.cfg
        reward = ctx.progress * cfg.reward_progress_scale
        reward += ctx.proximity * cfg.reward_proximity_scale
        action_delta = ctx.commands - ctx.prev_commands
        reward += (action_delta ** 2).sum(dim=-1) * cfg.reward_smoothness_scale
        reward = torch.where(ctx.reached, reward + cfg.reward_goal_reached, reward)
        reward = torch.where(ctx.proximity > 0.25, cfg.reward_collision_penalty, reward)
        reward = torch.where(ctx.timeout, reward + cfg.reward_timeout_penalty, reward)
        metrics = {
            'reward_mean': float(reward.mean().detach().cpu()),
            'goal_rate': float(ctx.reached.float().mean().detach().cpu()),
            'collision_rate': float(ctx.collision_terminated.float().mean().detach().cpu()),
            'progress_mean': float(ctx.progress.mean().detach().cpu()),
        }
        return reward, metrics


@reward_registry.register('jdrobot')
class JdrobotRewardComputerPlugin(BaseRewardComputer):
    name = 'jdrobot'

    def __init__(self, env: NavrlDirectEnv) -> None:
        super().__init__(env)
        from autonomy_navrl.plugins.jdrobot.rewards import JdrobotRewardComputer, JdrobotRewardConfig

        jd_cfg = getattr(env.cfg, '_jdrobot_reward_cfg', JdrobotRewardConfig.from_dict({}))
        self._computer = JdrobotRewardComputer(jd_cfg, env.device)

    def reset(self, env_ids: torch.Tensor) -> None:
        self._computer.reset(env_ids)

    def compute(self, ctx: RewardStepContext) -> tuple[torch.Tensor, dict[str, float]]:
        reward, metrics = self._computer.compute(
            distance=ctx.distance,
            bearing=ctx.bearing,
            progress=ctx.progress,
            proximity=ctx.proximity,
            commands=ctx.commands,
            prev_commands=ctx.prev_commands,
            lin_speed_b=ctx.lin_speed_b,
            reached=ctx.reached,
            collision_terminated=ctx.collision_terminated,
            timeout=ctx.timeout,
        )
        reward = self._env._task.modify_reward(
            reward, distance=ctx.distance, yaw_error=ctx.yaw_error
        )
        return reward, metrics


def create_reward_computer(profile: str, env: NavrlDirectEnv) -> BaseRewardComputer:
    return reward_registry.create(profile, env=env)
