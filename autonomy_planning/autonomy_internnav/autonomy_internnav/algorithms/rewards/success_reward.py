"""成功奖励 (SuccessRewardFn).

根据轨迹终点到目标的距离给出软成功奖励，并要求目标先落在成功奖励生效阈值内。

输入: trajectories[i] 必须包含
    - ``actions``: [predict_size, 3] (dx, dy, dθ) 增量, 用于计算轨迹终点;
    - ``goal_point``: [2] 局部目标点 (x, y);
    - ``goal_pixel``: [2] 目标像素坐标 (可选，用于兼容性);
    - ``gt_actions``: [H, 3] GT 动作序列 (可选).

输出: [batch_size] float tensor, 在成功半径内按距离线性给分，范围 [0, 1].

配置:
    - ``threshold``: 成功判定阈值（米），默认 0.3
    - ``success_effective_threshold``: 成功奖励生效阈值（米），可显式指定
    - ``target_speed`` / ``frame_dt`` / ``sample_interval`` / ``success_effective_horizon_points``:
      当未显式指定 ``success_effective_threshold`` 时，用它们推导生效阈值
"""

from typing import Any, Dict, List

import numpy as np
import torch

from .base_reward import RewardFn, register_reward_fn
from .trajectory_utils import (
    actions_to_waypoints_with_start,
    derive_success_effective_threshold,
)


def compute_success_reward(
    trajectory: np.ndarray,
    goal: np.ndarray,
    threshold: float = 0.3,
    effective_threshold: float | None = None,
) -> float:
    """计算单条轨迹的软成功奖励。

    Args:
        trajectory: shape=[H, 2], 轨迹点序列 (x, y).
        goal: shape=[2], 目标点 (x, y).
        threshold: 成功判定阈值（米）.

    Returns:
        float: 终点在 threshold 边界上为 0，终点与目标重合时为 1，
        中间按距离线性插值；若起点不在生效阈值内则返回 1（满分）以便监控曲线。
    """
    # 边界情况检查
    if trajectory.size == 0:
        raise ValueError("trajectory 不能为空数组")

    if np.any(np.isnan(goal)):
        raise ValueError("goal 包含 NaN 值")

    if threshold <= 0:
        raise ValueError(f"threshold 必须为正数, 当前值: {threshold}")
    if effective_threshold is not None and effective_threshold <= 0:
        raise ValueError(
            f"effective_threshold 必须为正数, 当前值: {effective_threshold}"
        )

    start_point = trajectory[0]
    start_distance = np.linalg.norm(start_point - goal)
    if effective_threshold is not None and start_distance > effective_threshold:
        return 1.0  # 起点不在生效阈值内，给满分以便监控成功奖励曲线

    # 提取轨迹终点
    endpoint = trajectory[-1]  # [2]

    # 计算欧氏距离
    distance = float(np.linalg.norm(endpoint - goal))
    if distance >= threshold:
        return 0.0
    return float((threshold - distance) / threshold)


def compute_success_reward_batch(
    trajectories: np.ndarray,
    goals: np.ndarray,
    threshold: float = 0.3,
    effective_threshold: float | None = None,
) -> np.ndarray:
    """批量计算软成功奖励。

    Args:
        trajectories: shape=[N, H, 2], N 条轨迹点序列.
        goals: shape=[N, 2], N 个目标点.
        threshold: 成功判定阈值（米）.

    Returns:
        np.ndarray: shape=[N], 每条轨迹的软成功奖励.
    """
    if trajectories.size == 0:
        raise ValueError("trajectories 不能为空数组")

    if np.any(np.isnan(goals)):
        raise ValueError("goals 包含 NaN 值")

    if threshold <= 0:
        raise ValueError(f"threshold 必须为正数, 当前值: {threshold}")
    if effective_threshold is not None and effective_threshold <= 0:
        raise ValueError(
            f"effective_threshold 必须为正数, 当前值: {effective_threshold}"
        )

    startpoints = trajectories[:, 0, :]
    endpoints = trajectories[:, -1, :]  # [N, 2]

    start_distances = np.linalg.norm(startpoints - goals, axis=1)
    distances = np.linalg.norm(endpoints - goals, axis=1)  # [N]

    success = np.maximum(0.0, (threshold - distances) / threshold)
    if effective_threshold is not None:
        success[start_distances > effective_threshold] = 1.0  # 起点不在生效阈值内，给满分以便监控成功奖励曲线
    return success


@register_reward_fn("success")
class SuccessRewardFn(RewardFn):
    """成功奖励函数。按终点距目标的远近输出软成功分数。"""
    
    name: str = "success"

    def __init__(self, config: Dict[str, Any] | None = None):
        super().__init__(config)
        self.threshold = float(self.config.get("threshold", 0.3))
        if self.threshold <= 0:
            raise ValueError(f"threshold 必须为正数, 当前值: {self.threshold}")

        explicit_effective = self.config.get("success_effective_threshold")
        if explicit_effective is not None:
            self.effective_threshold = float(explicit_effective)
            if self.effective_threshold <= 0:
                raise ValueError(
                    "success_effective_threshold 必须为正数, "
                    f"当前值: {self.effective_threshold}"
                )
        else:
            self.effective_threshold = derive_success_effective_threshold(
                target_speed=float(self.config.get("target_speed", 0.8)),
                frame_dt=float(self.config.get("frame_dt", 1.0 / 30.0)),
                sample_interval=int(self.config.get("sample_interval", 4)),
                horizon_points=int(
                    self.config.get("success_effective_horizon_points", 24)
                ),
            )
    
    def compute(self, trajectories: List[Dict[str, Any]], **kwargs: Any) -> torch.Tensor:
        """计算 batch reward。

        Args:
            trajectories: rollout 轨迹列表, 每个元素包含 actions 和 goal 信息.
            **kwargs: device, dtype 等.

        Returns:
            torch.Tensor: 形状 [batch_size] 的 float reward.
        """
        device = kwargs.get("device", torch.device("cpu"))
        dtype = kwargs.get("dtype", torch.float32)
        
        rewards = []
        for traj in trajectories:
            if traj.get("waypoints_with_start") is not None:
                wp = traj["waypoints_with_start"]
                waypoints_np = (
                    wp.detach().cpu().numpy() if isinstance(wp, torch.Tensor) else np.asarray(wp)
                )
            elif traj.get("waypoints") is not None:
                wp = traj["waypoints"]
                wp_np = wp.detach().cpu().numpy() if isinstance(wp, torch.Tensor) else np.asarray(wp)
                waypoints_np = np.vstack([np.zeros((1, 2), dtype=np.float64), wp_np[:, :2]])
            else:
                actions = traj["actions"]
                waypoints_np = actions_to_waypoints_with_start(
                    actions,
                    action_scale=float(self.config.get("action_scale", 4.0)),
                )
            
            # 获取目标点 (仅取 xy 二维, 兼容 xyt 三维输入)
            goal_point = traj.get("goal_point")
            if goal_point is None:
                raise ValueError(
                    f"轨迹 {len(rewards)} 缺少 goal_point！"
                    f"目标点是计算成功奖励的必要条件，不能使用默认值！"
                )
            
            if isinstance(goal_point, torch.Tensor):
                goal_np = goal_point.cpu().numpy()
            else:
                goal_np = np.array(goal_point)
            
            # 确保形状正确并只取 xy 二维
            if goal_np.ndim > 1:
                goal_np = goal_np.squeeze()
            goal_np = goal_np[:2]  # 只取 xy, 兼容 xyt 三维输入
            
            # 计算成功奖励
            reward = compute_success_reward(
                waypoints_np,
                goal_np,
                threshold=self.threshold,
                effective_threshold=self.effective_threshold,
            )
            rewards.append(reward)
        
        return torch.tensor(rewards, device=device, dtype=dtype)
