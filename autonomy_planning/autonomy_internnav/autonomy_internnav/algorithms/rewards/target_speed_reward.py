"""目标速度奖励 (TargetSpeedRewardFn).

只对超速进行惩罚，低速不再限制。

公式：
    单点超速量 overspeed_i = max(0, v_i - v_target)
    总惩罚     total       = sum_i slope * overspeed_i

其中:
    - v_i 由 actions[:, :2] / action_scale 还原物理位移, 再除以
      dt = frame_dt × sample_interval 得到真实速度 (m/s).
    - slope 为单位超速量斜率, 默认 1.0; 调高/调低可控制惩罚力度.

设计要点:
    1. 只惩罚"超速" (v > target_speed), 低速 (v <= target_speed) 不惩罚。
    2. 输出"超速量"为正值, 越大越差. 由 CompositeRewardFn 用**负权重**进行惩罚.
       推荐组合: target_speed_weight = -k_outer (例如 -1.0).
    3. 不做 1/predict_size 归一化. 累加值随轨迹长度增长, 由权重统一控制量级.

⚠️ frame_dt 与 sample_interval 必须与训练/评测数据集保持一致, 否则真实速度
   单位错位 → 超速量错算.
"""

from typing import Any, Dict, List, Optional

import numpy as np
import torch

from .base_reward import RewardFn, register_reward_fn
from .trajectory_utils import compute_speeds, derive_dt


def compute_target_speed_reward(
    actions: np.ndarray,
    target_speed: float = 0.8,
    slope: float = 1.0,
    action_scale: float = 4.0,
    dt: float = 1.0 / 30.0 * 4,
) -> float:
    """计算单条轨迹的超速惩罚量（只惩罚超速，不惩罚低速）。

    Args:
        actions: shape=[T, >=2], 已 ×action_scale 缩放的动作.
        target_speed: 目标真实速度上限 (m/s), 默认 0.8.
        slope: 超速惩罚斜率 k. 单点贡献 = slope * max(0, v_i - target_speed).
        action_scale: 反归一化因子, 默认 4.0.
        dt: 单步真实时间间隔 (秒).

    Returns:
        float: sum_i slope * max(0, v_i - target_speed) (>= 0). 越小越好.
    """
    if slope < 0:
        raise ValueError(f"slope 必须 >= 0, 当前={slope}")
    speeds = compute_speeds(actions, action_scale=action_scale, dt=dt)
    # 只惩罚超速部分，低速不惩罚
    overspeed = np.maximum(speeds - float(target_speed), 0.0)
    deviation = float(slope) * overspeed
    return float(deviation.sum())


@register_reward_fn("target_speed")
class TargetSpeedRewardFn(RewardFn):
    """目标速度奖励函数 (惩罚项, 只惩罚超速，不惩罚低速).

    config 字段:
        target_speed: 目标真实速度上限 (m/s), 默认 0.8.
        slope: 超速惩罚斜率 k, 默认 1.0; 单点 = k * max(0, v - v_target).
        action_scale: NavDP 反归一化因子, 默认 4.0.
        frame_dt: 数据集**原始采样周期** (秒), 默认 1/30.
        sample_interval: 数据集采样间隔, 默认 4.

    自动推导 dt = frame_dt × sample_interval.

    输出:
        torch.Tensor [batch], 单条轨迹的超速惩罚量 (>= 0).
        Composite 中应配合**负权重** (target_speed_weight < 0) 使用.
    """

    name: str = "target_speed"

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)
        self.target_speed = float(self.config.get("target_speed", 0.8))
        if self.target_speed < 0:
            raise ValueError(
                f"target_speed 必须 >= 0, 当前={self.target_speed}"
            )
        self.slope = float(self.config.get("slope", 1.0))
        self.action_scale = float(self.config.get("action_scale", 4.0))
        self.frame_dt = float(self.config.get("frame_dt", 1.0 / 30.0))
        self.sample_interval = int(self.config.get("sample_interval", 4))
        self.dt = derive_dt(self.frame_dt, self.sample_interval)

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
                    f"轨迹 {traj_idx} 缺少 actions, 无法计算目标速度奖励"
                )
            if isinstance(actions, torch.Tensor):
                actions_np = actions.detach().cpu().numpy()
            else:
                actions_np = np.asarray(actions)

            reward = compute_target_speed_reward(
                actions_np,
                target_speed=self.target_speed,
                slope=self.slope,
                action_scale=self.action_scale,
                dt=self.dt,
            )
            rewards.append(reward)

        return torch.tensor(rewards, device=device, dtype=dtype)