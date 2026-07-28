"""GRPO 算法模块.

参考 RLinf 的注册表设计, 与模型/环境完全解耦. 当前实现:
- advantages: compute_grpo_advantages (组内归一化)
- losses: compute_grpo_actor_loss (PPO-clip + 可选 KL)
- registry: ADV_REGISTRY / LOSS_REGISTRY, 通过 cfg.adv_type / cfg.loss_type 派发
- rewards: 抽象 RewardFn 接口 (留空壳, 后续扩展)
"""

from .advantages import compute_grpo_advantages
from .losses import compute_grpo_actor_loss
from .registry import (
    ADV_REGISTRY,
    LOSS_REGISTRY,
    calculate_adv_and_returns,
    policy_loss,
    register_advantage,
    register_policy_loss,
)
from .rewards import DummyRewardFn, RewardFn, build_reward_fn

__all__ = [
    "compute_grpo_advantages",
    "compute_grpo_actor_loss",
    "ADV_REGISTRY",
    "LOSS_REGISTRY",
    "calculate_adv_and_returns",
    "policy_loss",
    "register_advantage",
    "register_policy_loss",
    "RewardFn",
    "DummyRewardFn",
    "build_reward_fn",
]