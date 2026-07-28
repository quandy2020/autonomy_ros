"""注册表: 解耦优势函数与策略损失的实现.

参考 RLinf rlinf/algorithms/registry.py 的设计模式. 通过 cfg.algorithm.adv_type
和 cfg.algorithm.loss_type 派发到具体函数, 便于后续支持多种算法 (GRPO/PPO/...).
"""

from typing import Any, Callable, Dict

import torch

ADV_REGISTRY: Dict[str, Callable] = {}
LOSS_REGISTRY: Dict[str, Callable] = {}


def register_advantage(name: str):
    """装饰器: 注册一个优势函数到 ADV_REGISTRY."""

    def _decorator(fn: Callable) -> Callable:
        if name in ADV_REGISTRY:
            raise ValueError(f"Advantage `{name}` already registered.")
        ADV_REGISTRY[name] = fn
        return fn

    return _decorator


def register_policy_loss(name: str):
    """装饰器: 注册一个策略损失函数到 LOSS_REGISTRY."""

    def _decorator(fn: Callable) -> Callable:
        if name in LOSS_REGISTRY:
            raise ValueError(f"Policy loss `{name}` already registered.")
        LOSS_REGISTRY[name] = fn
        return fn

    return _decorator


def calculate_adv_and_returns(
    adv_type: str,
    rewards: torch.Tensor,
    **kwargs: Any,
) -> Dict[str, torch.Tensor]:
    """根据 adv_type 派发到具体的优势函数.

    Args:
        adv_type: 优势函数类型, 例如 "grpo".
        rewards: 形状 [batch_size] 或 [num_groups, group_size] 的标量奖励.
        **kwargs: 透传给具体优势函数, 例如 GRPO 需要 ``group_size``.

    Returns:
        dict: {"advantages": Tensor, ...}; 其他键由具体实现填充 (如 returns).
    """
    if adv_type not in ADV_REGISTRY:
        raise KeyError(
            f"adv_type=`{adv_type}` 未注册, 可选: {list(ADV_REGISTRY.keys())}"
        )
    return ADV_REGISTRY[adv_type](rewards=rewards, **kwargs)


def policy_loss(
    loss_type: str,
    logprobs: torch.Tensor,
    prev_logprobs: torch.Tensor,
    advantages: torch.Tensor,
    **kwargs: Any,
) -> Dict[str, torch.Tensor]:
    """根据 loss_type 派发到具体的策略损失.

    Args:
        loss_type: 损失类型, 例如 "actor" (PPO-clip 风格).
        logprobs: 训练时重计算的 log prob, 形状 [batch_size, ...].
        prev_logprobs: rollout 时保存的 log prob, 形状与 logprobs 完全一致.
        advantages: 形状 [batch_size] 的优势, 自动广播到 logprobs 形状.
        **kwargs: 透传给具体损失函数 (clip_ratio_low/high, kl_beta, ref_logprobs ...).

    Returns:
        dict: {"loss": Tensor, "pg_loss": Tensor, "kl_loss": Tensor, "ratio": Tensor, ...}.
    """
    if loss_type not in LOSS_REGISTRY:
        raise KeyError(
            f"loss_type=`{loss_type}` 未注册, 可选: {list(LOSS_REGISTRY.keys())}"
        )
    return LOSS_REGISTRY[loss_type](
        logprobs=logprobs,
        prev_logprobs=prev_logprobs,
        advantages=advantages,
        **kwargs,
    )


# 触发注册 (importing 时副作用)
from . import advantages as _advantages  # noqa: E402, F401
from . import losses as _losses  # noqa: E402, F401