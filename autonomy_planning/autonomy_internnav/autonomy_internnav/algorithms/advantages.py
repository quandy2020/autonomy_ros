"""优势函数实现 (GRPO 等).

参考 RLinf rlinf/algorithms/advantages.py. GRPO 通过组内 (group-wise) 归一化
得到 advantage, 无需 critic / value function.
"""

from typing import Dict, Optional

import torch

from .registry import register_advantage


@register_advantage("grpo")
def compute_grpo_advantages(
    rewards: torch.Tensor,
    group_size: int,
    normalize: bool = True,
    eps: float = 1e-5,
    mask: Optional[torch.Tensor] = None,
) -> Dict[str, torch.Tensor]:
    """计算 GRPO 优势 (组内归一化).

    将 ``rewards`` 按 ``group_size`` 重组为 ``[num_groups, group_size]``, 然后在
    组内做 ``(r - mean) / (std + eps)``. 同一 prompt 复制 group_size 次得到的
    group_size 条轨迹组成一组, 彼此相对比较.

    Args:
        rewards: 标量奖励, 形状 [batch_size]; 要求 ``batch_size % group_size == 0``.
        group_size: 组大小, 即每个 prompt 并行采样的轨迹数.
        normalize: 是否除以组内 std (False 时只减均值, 用于消融).
        eps: std 防零项.
        mask: 可选 [batch_size] 掩码, True/1 为有效样本; 当前实现仅记录, 由上层使用.

    Returns:
        dict: 包含
            - advantages: [batch_size], 已展平回原顺序.
            - returns: 同 advantages (GRPO 中 returns 即 advantages, 无 critic).
            - reward_mean / reward_std: [num_groups] 组内统计量, 便于日志.
    """
    if rewards.dim() != 1:
        raise ValueError(f"rewards 应为一维, 实际 shape={tuple(rewards.shape)}")
    batch_size = rewards.shape[0]
    if batch_size % group_size != 0:
        raise ValueError(
            f"batch_size ({batch_size}) 必须能被 group_size ({group_size}) 整除"
        )

    num_groups = batch_size // group_size
    grouped = rewards.view(num_groups, group_size)
    reward_mean = grouped.mean(dim=-1, keepdim=True)
    reward_std = grouped.std(dim=-1, keepdim=True)

    adv = grouped - reward_mean
    if normalize:
        c = 1.0
        # adv = adv / c   
        adv = adv / (reward_std + eps)

    advantages = adv.reshape(batch_size).to(rewards.dtype)

    return {
        "advantages": advantages,
        "returns": advantages.clone(),
        "reward_mean": reward_mean.squeeze(-1),
        "reward_std": reward_std.squeeze(-1),
    }


def filter_rewards_by_group(
    rewards: torch.Tensor,
    loss_mask: torch.Tensor,
    group_size: int,
    lower_bound: float = -float("inf"),
    upper_bound: float = float("inf"),
) -> torch.Tensor:
    """对每个 group 的均值 reward 做范围过滤, 不在 [lower, upper] 的组其
    loss_mask 置零, 不参与训练. 返回更新后的 loss_mask.

    与 RLinf 的 filter_rewards 行为一致, 当前 trainer 默认关闭, 留作可选项.
    """
    if rewards.shape[0] % group_size != 0:
        raise ValueError("batch_size 与 group_size 不匹配")
    num_groups = rewards.shape[0] // group_size
    grouped_mean = rewards.view(num_groups, group_size).mean(dim=-1)  # [num_groups]
    keep_group = (grouped_mean >= lower_bound) & (grouped_mean <= upper_bound)
    keep_mask = keep_group.repeat_interleave(group_size).to(loss_mask.dtype)
    return loss_mask * keep_mask