"""策略损失实现 (PPO-clip 风格, 供 GRPO 复用).

参考 RLinf rlinf/algorithms/losses.py 的 compute_grpo_actor_loss_fn. GRPO 的
actor 损失本质上是 PPO 的 clip ratio 损失, 仅 advantage 来源不同, 因此与 PPO
共享同一份实现.
"""

from typing import Dict, Optional

import torch

from .registry import register_policy_loss


def _broadcast_advantages(advantages: torch.Tensor, target_shape: torch.Size) -> torch.Tensor:
    """将 advantages 广播到 logprobs 的形状.

    支持两种情况:
    - advantages [batch_size], logprobs [batch_size]: 直接使用
    - advantages [batch_size], logprobs [T, batch_size]: 广播到 [1, batch_size] -> [T, batch_size]
      (joint_logprob=False 时, 同一条轨迹的每个去噪步共享同一个 advantage)
    """
    if advantages.shape == target_shape:
        return advantages
    # [batch_size] -> [batch_size, ...] 或 [batch_size] -> [T, batch_size]
    if advantages.dim() == 1:
        # 找到 advantages 对应的维度位置
        for dim_idx in range(len(target_shape)):
            if target_shape[dim_idx] == advantages.shape[0]:
                # 在该维度前插入 1, 其余维度也插入 1, 然后 expand
                view_shape = [1] * len(target_shape)
                view_shape[dim_idx] = advantages.shape[0]
                return advantages.view(view_shape).expand(target_shape)
    raise ValueError(
        f"advantages shape={tuple(advantages.shape)} 无法广播到 logprobs shape={tuple(target_shape)}"
    )


@register_policy_loss("actor")
def compute_grpo_actor_loss(
    logprobs: torch.Tensor,
    prev_logprobs: torch.Tensor,
    advantages: torch.Tensor,
    clip_ratio_low: float = 0.2,
    clip_ratio_high: float = 0.28,
    kl_beta: float = 0.0,
    ref_logprobs: Optional[torch.Tensor] = None,
    loss_mask: Optional[torch.Tensor] = None,
    entropy: Optional[torch.Tensor] = None,
    entropy_bonus: float = 0.0,
) -> Dict[str, torch.Tensor]:
    """PPO-clip 风格的 actor 损失, 用于 GRPO.

    Args:
        logprobs: 训练时重计算的 log prob, 形状 [batch_size, ...].
        prev_logprobs: rollout 时保存的 log prob, 与 logprobs 形状一致.
        advantages: [batch_size] 标量优势, 自动广播.
        clip_ratio_low: 比率裁剪下界 1 - low.
        clip_ratio_high: 比率裁剪上界 1 + high (非对称裁剪, 与 DAPO 一致).
        kl_beta: 与 ref_logprobs 的 KL 系数 (0 时跳过).
        ref_logprobs: 参考策略 (通常是 SFT 初始模型) 的 log prob.
        loss_mask: 形状与 logprobs 一致的 0/1 掩码, 用于过滤无效样本.
        entropy: 形状与 logprobs 一致的策略熵; 仅在 entropy_bonus>0 时使用.
        entropy_bonus: 熵奖励系数.

    Returns:
        dict: {"loss", "pg_loss", "kl_loss", "entropy_loss", "ratio", "clip_frac", "approx_kl"}.
    """
    if logprobs.shape != prev_logprobs.shape:
        raise ValueError(
            f"logprobs {tuple(logprobs.shape)} 与 prev_logprobs {tuple(prev_logprobs.shape)} 不一致"
        )

    adv = _broadcast_advantages(advantages.to(logprobs.dtype), logprobs.shape)

    log_ratio = logprobs - prev_logprobs.detach()
    ratio = torch.exp(log_ratio)

    pg_loss1 = -adv * ratio
    pg_loss2 = -adv * torch.clamp(ratio, 1.0 - clip_ratio_low, 1.0 + clip_ratio_high)
    pg_loss_elem = torch.maximum(pg_loss1, pg_loss2)

    if loss_mask is None:
        mask = torch.ones_like(pg_loss_elem)
    else:
        mask = loss_mask.to(pg_loss_elem.dtype)
        if mask.shape != pg_loss_elem.shape:
            mask = mask.view(mask.shape[0], *([1] * (pg_loss_elem.dim() - 1))).expand_as(pg_loss_elem)

    denom = mask.sum().clamp(min=1.0)
    pg_loss = (pg_loss_elem * mask).sum() / denom

    # 近似 KL (PPO 监控用): 0.5 * E[(log_ratio)^2]
    approx_kl = (0.5 * (log_ratio ** 2) * mask).sum() / denom
    clip_frac = (((ratio - 1.0).abs() > torch.tensor(
        max(clip_ratio_low, clip_ratio_high), device=ratio.device, dtype=ratio.dtype
    )) * mask).sum() / denom

    # 与参考策略的 KL (k3 估计: exp(ref-logp) - (ref-logp) - 1 >= 0)
    # 始终计算 kl_loss (用于监控), kl_beta 仅控制是否加入总 loss
    kl_loss = torch.zeros((), device=logprobs.device, dtype=logprobs.dtype)
    if ref_logprobs is not None:
        if ref_logprobs.shape != logprobs.shape:
            raise ValueError("ref_logprobs 形状与 logprobs 不一致")
        kl_delta = ref_logprobs.detach() - logprobs
        kl_elem = torch.exp(kl_delta) - kl_delta - 1.0
        kl_loss = (kl_elem * mask).sum() / denom

    entropy_loss = torch.zeros((), device=logprobs.device, dtype=logprobs.dtype)
    if entropy_bonus != 0.0 and entropy is not None:
        if entropy.shape != logprobs.shape:
            entropy = entropy.expand_as(logprobs)
        entropy_loss = -(entropy * mask).sum() / denom  # 最大化熵 -> 损失加负号

    loss = pg_loss + kl_beta * kl_loss + entropy_bonus * entropy_loss

    return {
        "loss": loss,
        "pg_loss": pg_loss.detach(),
        "kl_loss": kl_loss.detach(),
        "entropy_loss": entropy_loss.detach(),
        "ratio": ratio.detach().mean(),
        "approx_kl": approx_kl.detach(),
        "clip_frac": clip_frac.detach(),
    }