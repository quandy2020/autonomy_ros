"""进度奖励 (ProgressRewardFn).

衡量轨迹是否使机器人更接近目标。

输入: trajectories[i] 必须包含
    - ``actions``: [predict_size, 3] (dx, dy, dθ) 增量, 用于计算轨迹;
    - ``goal_point``: [2] 全局目标点 (x, y).
    - ``waypoints``: [H, 2] 可选，若上游已预计算则直接复用;

输出: [batch_size] float tensor.

说明:
    - 轨迹前进（更接近目标）返回正奖励
    - 轨迹倒退（远离目标）返回 0.0
    - 轨迹无移动返回 0.0

适配 GRPO 训练数据格式:
    - 优先复用上游预计算的 waypoints
    - 否则从 actions 累积计算
"""

from typing import Any, Dict, List, Optional

import numpy as np
import torch

from .base_reward import RewardFn, register_reward_fn
from .trajectory_utils import (
    actions_to_waypoints_with_start,
    derive_success_effective_threshold,
)


def compute_progress_reward(
    trajectory: np.ndarray,
    goal: np.ndarray,
    reach_threshold: float = 0.3,
    start_no_progress_threshold: float | None = None,
) -> float:
    """计算单条轨迹的进度奖励。"""
    if trajectory.size == 0:
        raise ValueError("trajectory 不能为空数组")

    if np.any(np.isnan(goal)):
        raise ValueError("goal 包含 NaN 值")

    if reach_threshold <= 0:
        raise ValueError(f"reach_threshold 必须为正数, 当前值: {reach_threshold}")
    if start_no_progress_threshold is not None and start_no_progress_threshold <= 0:
        raise ValueError(
            "start_no_progress_threshold 必须为正数, "
            f"当前值: {start_no_progress_threshold}"
        )

    start_point = trajectory[0]
    dist_start = float(np.linalg.norm(start_point - goal))
    dists_to_goal = np.linalg.norm(trajectory - goal, axis=1)

    # 规则1：起点已在"无需 progress"阈值内 -> progress 直接为 0
    # 若未显式传入，则退化为旧行为：使用成功阈值。
    effective_start_threshold = (
        float(start_no_progress_threshold)
        if start_no_progress_threshold is not None
        else float(reach_threshold)
    )
    if dist_start <= effective_start_threshold:
        return 0.0

    inside_mask = dists_to_goal <= reach_threshold
    if inside_mask.any():
        first_inside_idx = int(np.argmax(inside_mask))
        effective_end_dist = float(dists_to_goal[first_inside_idx])
    else:
        effective_end_dist = float(dists_to_goal[-1])

    return max(0.0, dist_start - effective_end_dist)


def compute_progress_reward_batch(
    trajectories: np.ndarray,
    goals: np.ndarray,
    reach_threshold: float = 0.3,
    start_no_progress_threshold: float | None = None,
) -> np.ndarray:
    """批量计算进度奖励。"""
    if trajectories.size == 0:
        raise ValueError("trajectories 不能为空数组")

    if np.any(np.isnan(goals)):
        raise ValueError("goals 包含 NaN 值")

    if reach_threshold <= 0:
        raise ValueError(f"reach_threshold 必须为正数, 当前值: {reach_threshold}")
    if start_no_progress_threshold is not None and start_no_progress_threshold <= 0:
        raise ValueError(
            "start_no_progress_threshold 必须为正数, "
            f"当前值: {start_no_progress_threshold}"
        )

    start_points = trajectories[:, 0, :]
    dist_start = np.linalg.norm(start_points - goals, axis=1)  # [N]
    dists_to_goal = np.linalg.norm(trajectories - goals[:, None, :], axis=2)  # [N, H]
    inside_mask = dists_to_goal <= reach_threshold
    first_inside_idx = np.argmax(inside_mask, axis=1)
    has_inside = inside_mask.any(axis=1)
    last_dist = dists_to_goal[:, -1]
    inside_dist = dists_to_goal[np.arange(len(trajectories)), first_inside_idx]
    effective_end_dist = np.where(has_inside, inside_dist, last_dist)
    progress = np.maximum(0.0, dist_start - effective_end_dist)

    # 规则1：起点已在"无需 progress"阈值内 -> progress 置 0
    effective_start_threshold = (
        float(start_no_progress_threshold)
        if start_no_progress_threshold is not None
        else float(reach_threshold)
    )
    start_in_threshold = dist_start <= effective_start_threshold
    progress[start_in_threshold] = 0.0
    return progress


@register_reward_fn("progress")
class ProgressRewardFn(RewardFn):
    """进度奖励函数。衡量轨迹是否使机器人更接近目标。"""
    
    name: str = "progress"

    def __init__(self, config: Dict[str, Any] | None = None):
        super().__init__(config)
        self.reach_threshold = float(
            self.config.get(
                "success_threshold",
                self.config.get(
                    "success_judge_threshold",
                    self.config.get("threshold", 0.3),
                ),
            )
        )
        if self.reach_threshold <= 0:
            raise ValueError(
                f"success_threshold 必须为正数, 当前值: {self.reach_threshold}"
            )

        explicit_effective = self.config.get("success_effective_threshold")
        if explicit_effective is not None:
            self.start_no_progress_threshold = float(explicit_effective)
            if self.start_no_progress_threshold <= 0:
                raise ValueError(
                    "success_effective_threshold 必须为正数, "
                    f"当前值: {self.start_no_progress_threshold}"
                )
        else:
            self.start_no_progress_threshold = derive_success_effective_threshold(
                target_speed=float(self.config.get("target_speed", 0.8)),
                frame_dt=float(self.config.get("frame_dt", 1.0 / 30.0)),
                sample_interval=int(self.config.get("sample_interval", 4)),
                horizon_points=int(
                    self.config.get("success_effective_horizon_points", 24)
                ),
            )
        self.action_scale = float(self.config.get("action_scale", 4.0))
    
    def compute(self, trajectories: List[Dict[str, Any]], **kwargs: Any) -> torch.Tensor:
        """计算 batch reward。

        支持的数据格式:
        1. 优先复用 trajectories[i]["waypoints_with_start"]
        2. 或将 trajectories[i]["waypoints"] 视为局部坐标累计轨迹并补局部原点
        3. 否则从 trajectories[i]["actions"] 累积计算

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
            if "waypoints_with_start" in traj and traj["waypoints_with_start"] is not None:
                waypoints = traj["waypoints_with_start"]
                if isinstance(waypoints, torch.Tensor):
                    waypoints_np = waypoints.cpu().numpy()
                else:
                    waypoints_np = np.asarray(waypoints, dtype=np.float64)
            elif "waypoints" in traj and traj["waypoints"] is not None:
                waypoints = traj["waypoints"]
                if isinstance(waypoints, torch.Tensor):
                    waypoints_np = waypoints.cpu().numpy()
                else:
                    waypoints_np = np.asarray(waypoints, dtype=np.float64)
                waypoints_np = np.vstack([
                    np.zeros((1, 2), dtype=np.float64),
                    waypoints_np[:, :2],
                ])
            else:
                actions = traj["actions"]
                waypoints_np = actions_to_waypoints_with_start(
                    actions,
                    action_scale=self.action_scale,
                )
            
            # 获取目标点 (仅取 xy 二维, 兼容 xyt 三维输入)
            goal_point = traj.get("goal_point")
            if goal_point is None:
                raise ValueError(
                    f"轨迹 {len(rewards)} 缺少 goal_point！"
                    f"目标点是计算进度奖励的必要条件，不能使用默认值！"
                )
            
            if isinstance(goal_point, torch.Tensor):
                goal_np = goal_point.cpu().numpy()
            else:
                goal_np = np.array(goal_point)
            
            # 确保形状正确并只取 xy 二维
            if goal_np.ndim > 1:
                goal_np = goal_np.squeeze()
            goal_np = goal_np[:2]  # 只取 xy, 兼容 xyt 三维输入
            
            # 计算进度奖励：
            # - 起点是否还需要 progress 奖励，使用 success_effective_threshold
            # - 轨迹中途是否算“到达成功区”，仍使用 success_threshold
            reward = compute_progress_reward(
                waypoints_np,
                goal_np,
                reach_threshold=self.reach_threshold,
                start_no_progress_threshold=self.start_no_progress_threshold,
            )
            rewards.append(reward)
        
        return torch.tensor(rewards, device=device, dtype=dtype)
