"""目标方向投影奖励 (GoalDirectionProjectionRewardFn).

把轨迹终点投影到「起点→目标」方向上, 只奖励正投影 (朝目标方向的位移).
天然同时解决方向和速度两个问题:

    投影 = dot(终点 - 起点, 方向单位向量)
    reward = min(max(0, 投影), 起点到目标距离)

场景分析:
    往目标方向快速走 → 投影大 → 高奖励 ✓
    往目标方向慢速走 → 投影小 → 低奖励 (自然鼓励提速) ✓
    偏离目标方向     → 投影减小 → 低奖励 ✓
    背离目标方向     → 投影 ≤ 0 → 零奖励 ✓
    超出目标点       → 投影截断到目标距离 → 防止恶意得分 ✓

输入: trajectories[i] 必须包含
    - ``actions``: [predict_size, 3] (dx, dy, dθ) 增量;
    - ``goal_point``: [2] 全局目标点 (x, y);
    - 可选 ``waypoints_with_start``: [T+1, 2] 含起点的轨迹点.

输出: [batch_size] float tensor, 值域 [0, 起点到目标距离].

注意:
    - 输出为正向量投影 (>=0), 投影越大越好; 在 CompositeRewardFn 中
      应配合**正权重**使用 (goal_direction_projection_weight > 0).
    - 当起点与目标重合 (direction 长度 ≈ 0) 时, 返回 0.0 以避免除零.
    - 投影上限为起点到目标的距离, 防止超出目标后继续得分.
"""

from typing import Any, Dict, List, Optional

import numpy as np
import torch

from .base_reward import RewardFn, register_reward_fn
from .trajectory_utils import actions_to_waypoints_with_start


def compute_goal_direction_projection(
    trajectory: np.ndarray,
    goal: np.ndarray,
    eps: float = 1e-8,
) -> float:
    """计算单条轨迹的目标方向投影奖励.

    Args:
        trajectory: [T+1, 2] 含起点的轨迹点序列 (起点为 trajectory[0]).
        goal: [2] 目标点 (x, y).
        eps: 防止除零的小量.

    Returns:
        float: 正投影长度 (>=0), 上限为起点到目标距离.
    """
    if trajectory.size == 0:
        raise ValueError("trajectory 不能为空数组")
    if np.any(np.isnan(goal)):
        raise ValueError("goal 包含 NaN 值")

    start_point = trajectory[0, :2]
    end_point = trajectory[-1, :2]

    direction = goal[:2] - start_point
    direction_norm = np.linalg.norm(direction)

    if direction_norm < eps:
        return 0.0

    direction_unit = direction / direction_norm
    displacement = end_point - start_point
    projection = float(np.dot(displacement, direction_unit))

    # 截断投影: 上限为起点到目标的距离, 防止超出目标后恶意得分
    return min(direction_norm, max(0.0, projection))


def compute_goal_direction_projection_batch(
    trajectories: np.ndarray,
    goals: np.ndarray,
    eps: float = 1e-8,
) -> np.ndarray:
    """批量计算目标方向投影奖励.

    Args:
        trajectories: [N, T+1, 2] 含起点的轨迹点序列.
        goals: [N, 2] 目标点.
        eps: 防止除零的小量.

    Returns:
        np.ndarray: [N] 正投影长度 (>=0), 上限为起点到目标距离.
    """
    if trajectories.size == 0:
        raise ValueError("trajectories 不能为空数组")
    if np.any(np.isnan(goals)):
        raise ValueError("goals 包含 NaN 值")

    start_points = trajectories[:, 0, :2]
    end_points = trajectories[:, -1, :2]

    directions = goals[:, :2] - start_points
    direction_norms = np.linalg.norm(directions, axis=1)

    displacements = end_points - start_points
    projections = np.sum(displacements * directions, axis=1)

    safe_mask = direction_norms >= eps
    projections = np.where(safe_mask, projections / np.maximum(direction_norms, eps), 0.0)

    # 截断投影: 上限为起点到目标的距离, 防止超出目标后恶意得分
    return np.minimum(direction_norms, np.maximum(0.0, projections))


@register_reward_fn("goal_direction_projection")
class GoalDirectionProjectionRewardFn(RewardFn):
    """目标方向投影奖励函数.

    把轨迹终点投影到「起点→目标」方向, 只奖励正投影.
    投影越大代表朝目标方向的有效位移越大, 奖励越高.
    投影上限为起点到目标距离, 防止超出目标后恶意得分.

    config 字段:
        action_scale: NavDP 反归一化因子, 默认 4.0.
        eps: 防止除零的小量, 默认 1e-8.

    输出:
        torch.Tensor [batch], 正投影长度 (>=0), 上限为起点到目标距离.
        Composite 中应配合**正权重** (goal_direction_projection_weight > 0) 使用.
    """

    name: str = "goal_direction_projection"

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)
        self.action_scale = float(self.config.get("action_scale", 4.0))
        self.eps = float(self.config.get("eps", 1e-8))

    def compute(
        self,
        trajectories: List[Dict[str, Any]],
        **kwargs: Any,
    ) -> torch.Tensor:
        """计算 batch reward.

        支持的数据格式:
        1. 优先复用 trajectories[i]["waypoints_with_start"]
        2. 或将 trajectories[i]["waypoints"] 视为局部坐标累计轨迹并补局部原点
        3. 否则从 trajectories[i]["actions"] 累积计算

        Args:
            trajectories: rollout 轨迹列表, 每个元素包含 actions 和 goal 信息.
            **kwargs: device, dtype 等.

        Returns:
            torch.Tensor: 形状 [batch_size] 的 float reward (>=0).
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

            goal_point = traj.get("goal_point")
            if goal_point is None:
                raise ValueError(
                    "轨迹缺少 goal_point！"
                    "目标点是计算目标方向投影奖励的必要条件！"
                )

            if isinstance(goal_point, torch.Tensor):
                goal_np = goal_point.cpu().numpy()
            else:
                goal_np = np.array(goal_point)

            if goal_np.ndim > 1:
                goal_np = goal_np.squeeze()
            goal_np = goal_np[:2]

            reward = compute_goal_direction_projection(
                waypoints_np,
                goal_np,
                eps=self.eps,
            )
            rewards.append(reward)

        return torch.tensor(rewards, device=device, dtype=dtype)