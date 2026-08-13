"""Example / advanced reward plugins (register on import)."""

from __future__ import annotations

import torch

from autonomy_navrl.plugins.rewards import BaseRewardComputer, RewardStepContext, reward_registry


@reward_registry.register('sparse')
class SparseRewardComputer(BaseRewardComputer):
    """Sparse success-only reward for debugging / imitation-style training."""

    name = 'sparse'

    def reset(self, env_ids: torch.Tensor) -> None:
        del env_ids

    def compute(self, ctx: RewardStepContext) -> tuple[torch.Tensor, dict[str, float]]:
        reward = torch.zeros_like(ctx.distance)
        reward = torch.where(ctx.reached, reward + 1.0, reward)
        reward = torch.where(ctx.collision_terminated, reward - 1.0, reward)
        return reward, {
            'reward_mean': float(reward.mean().detach().cpu()),
            'goal_rate': float(ctx.reached.float().mean().detach().cpu()),
            'collision_rate': float(ctx.collision_terminated.float().mean().detach().cpu()),
        }
