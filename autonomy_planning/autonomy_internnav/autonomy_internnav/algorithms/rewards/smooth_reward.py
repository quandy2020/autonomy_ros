"""平滑奖励 (SmoothRewardFn).

衡量轨迹相邻 waypoint 之间步长的均值，用于控制加速度。
由于时间间隔固定，限制前后两点距离的变化量等价于限制加速度。

公式:
    步长 d_i = ||p_i - p_{i-1}||_2,  i = 1..T-1   (注意 p_0 = 起点 (0,0))
    单条轨迹 smooth_metric = mean(d_i)        # 默认 reduce='mean'
    或     smooth_metric = sum(d_i)           # reduce='sum'

输出 "平均步长" (>= 0)：CompositeRewardFn 中配合**负权重**使用。

特别说明:
    - 若 actions 为 [T, *], 则得到 T 个 waypoint, T 个步长 (含起点到第一个点).
      公式中第一个步长即 ||p_0||，由于 p_0 是 cumsum 的第一个元素，
      其本身就等于第一个 delta_xy. 与 NavDP 行为一致.
    - 若 reduce='mean', smooth_metric 表示平均步长，适合作为平滑度指标.
    - 若 reduce='sum', 总步长随 T 增长, 与 target_speed/progress 量级风格一致.
"""

from typing import Any, Dict, List, Optional

import numpy as np
import torch

from .base_reward import RewardFn, register_reward_fn
from .trajectory_utils import actions_to_delta_xy


def compute_smooth_reward(
    actions: np.ndarray,
    action_scale: float = 4.0,
    reduce: str = "mean",
) -> float:
    """计算单条轨迹的步长均值 (用于控制加速度, >=0, 越小越好).

    Args:
        actions: shape=[T, >=2], 已 ×action_scale 缩放.
        action_scale: 反归一化因子.
        reduce: 'mean' 或 'sum'.

    Returns:
        float: 步长均值或总和 (mean/sum), >= 0.
    """
    if reduce not in ("mean", "sum"):
        raise ValueError(f"reduce 必须为 'mean' 或 'sum', 当前={reduce}")

    delta_xy = actions_to_delta_xy(actions, action_scale=action_scale)
    step_lens = np.linalg.norm(delta_xy, axis=-1)  # [T]

    if step_lens.size == 0:
        return 0.0

    if reduce == "mean":
        return float(step_lens.mean())
    return float(step_lens.sum())


@register_reward_fn("smooth")
class SmoothRewardFn(RewardFn):
    """平滑奖励函数 (步长均值, 用于控制加速度, >=0).

    config 字段:
        action_scale: NavDP 反归一化因子, 默认 4.0.
        reduce: 'mean' (默认) 或 'sum'.

    输出 (>= 0):
        Composite 中应配合**负权重** (smooth_weight < 0) 使用.
    """

    name: str = "smooth"

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)
        self.action_scale = float(self.config.get("action_scale", 4.0))
        self.reduce = str(self.config.get("reduce", "mean"))
        if self.reduce not in ("mean", "sum"):
            raise ValueError(
                f"reduce 必须为 'mean' 或 'sum', 当前={self.reduce}"
            )

    def compute(
        self,
        trajectories: List[Dict[str, Any]],
        **kwargs: Any,
    ) -> torch.Tensor:
        device = kwargs.get("device", torch.device("cpu"))
        dtype = kwargs.get("dtype", torch.float32)

        rewards: List[float] = []
        for traj_idx, traj in enumerate(trajectories):
            actions = traj.get("actions")
            if actions is None:
                raise ValueError(
                    f"轨迹 {traj_idx} 缺少 actions, 无法计算平滑奖励"
                )
            if isinstance(actions, torch.Tensor):
                actions_np = actions.detach().cpu().numpy()
            else:
                actions_np = np.asarray(actions)

            reward = compute_smooth_reward(
                actions_np,
                action_scale=self.action_scale,
                reduce=self.reduce,
            )
            rewards.append(reward)

        return torch.tensor(rewards, device=device, dtype=dtype)