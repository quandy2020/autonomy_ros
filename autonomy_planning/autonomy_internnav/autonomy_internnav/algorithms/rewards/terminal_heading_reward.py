"""终点朝向奖励 (TerminalHeadingRewardFn).

衡量轨迹"末端朝向"是否对准目标射线。仅在 **轨迹末端已经处于起点-目标连线段上的投影区间内**
且 **末端尚未足够接近目标** 且 **起点本身不在成功范围内** 时才生效, 其余情况一律输出 0
(不施加方向约束).

公式：
    1) 末端方向向量 (用最后 K 个点估计, 默认 K=4):
           heading_vec = waypoints[-1] - waypoints[-K]
    2) 目标射线向量 (起点 -> 目标):
           target_ray  = goal_xy - start_xy,  其中 start_xy = waypoints[0]
    3) 夹角:
           cos_theta   = dot(heading_vec, target_ray) / (||heading_vec|| * ||target_ray||)
           theta_deg   = arccos(clip(cos_theta, -1, 1)) * 180/pi
    4) 偏离量 (>= 0):
           penalty = max(0, theta_deg - free_angle_deg) / (180 - free_angle_deg)

    返回 penalty (0~1, 越大越差). CompositeRewardFn 中配合**负权重**使用.

触发条件 (任何一项不满足都直接返回 0, 不施加方向约束):
    a) waypoints 长度 >= window_points (默认 4) 才能取最后 K 个点;
    b) ||heading_vec|| > heading_eps   (避免末端几乎不动时方向无意义);
    c) ||goal_xy - p_end|| > success_threshold  (距目标过近时方向意义弱);
    d) 末端在起点-目标连线段上的投影系数 t ∈ [0, 1]:
           start = waypoints[0]
           t = dot(p_end - start, goal - start) / ||goal - start||^2
       即 "末端处在起点指向目标的前进段中". 若 t < 0 (回头) 或 t > 1 (越过目标),
       则朝向约束容易给出反向激励, 直接禁用更稳健.
    e) ||goal_xy - start_xy|| > success_threshold
       若起点本身已在成功范围内, 不再计算末端航向角奖励.
"""

import math
from typing import Any, Dict, List, Optional

import numpy as np
import torch

from .base_reward import RewardFn, register_reward_fn
from .trajectory_utils import (
    actions_to_waypoints_with_start,
    derive_success_effective_threshold,
    ensure_waypoints_include_start,
)

def _project_on_segment_t(
    p_end: np.ndarray, goal: np.ndarray, start: np.ndarray
) -> float:
    """计算 p_end 在线段 start->goal 上的投影系数 t.

    t = ((p_end - start) · (goal - start)) / ||goal - start||^2
    t ∈ [0, 1]: 投影落在线段内 (起点 → 目标).
    """
    seg = goal - start
    seg_sq = float(np.dot(seg, seg))
    if seg_sq <= 1e-12:
        return float("nan")
    return float(np.dot(p_end - start, seg) / seg_sq)


def compute_terminal_heading_reward(
    waypoints: np.ndarray,
    goal_xy: np.ndarray,
    window_points: int = 4,
    free_angle_deg: float = 45.0,
    success_threshold: float = 0.3,
    success_effective_threshold: float | None = None,
    heading_eps: float = 1e-3,
    goal_eps: float = 1e-6,
) -> float:
    """计算单条轨迹末端朝向偏离量 (>=0, 越小越好).

    Args:
        waypoints: shape=[T, 2], 累积位置 (米), 起点默认 (0,0).
        goal_xy: shape=[2], 目标点 (米).
        window_points: 末端窗口大小 K, heading = p[-1] - p[-K].
        free_angle_deg: 免罚角度阈值 (度), 默认 45.
        success_threshold: 距目标若 < 该阈值, 视作"已到达", 不再约束方向.
        success_effective_threshold: 起点若落在该阈值内, 不再计算终点朝向奖励.
        heading_eps: ||heading_vec|| 下限, 低于此值视为末端未动.
        goal_eps: ||goal_vec|| 下限, 低于此值视为已抵达.

    Returns:
        float: max(0, theta_deg-free_angle_deg)/(180-free_angle_deg), 0~1.
        不触发时返回 0.
    """
    if window_points < 2:
        raise ValueError(f"window_points 必须 >= 2, 当前={window_points}")
    if free_angle_deg < 0 or free_angle_deg > 180:
        raise ValueError(
            f"free_angle_deg 必须 ∈ [0, 180], 当前={free_angle_deg}"
        )
    if success_threshold < 0:
        raise ValueError(
            f"success_threshold 必须 >= 0, 当前={success_threshold}"
        )
    if (
        success_effective_threshold is not None
        and success_effective_threshold < 0
    ):
        raise ValueError(
            "success_effective_threshold 必须 >= 0, "
            f"当前={success_effective_threshold}"
        )

    waypoints = ensure_waypoints_include_start(waypoints)
    goal = np.asarray(goal_xy, dtype=np.float64).reshape(-1)[:2]

    if waypoints.size == 0:
        return 0.0
    if waypoints.ndim != 2 or waypoints.shape[-1] < 2:
        raise ValueError(
            f"waypoints 必须形如 [T, >=2], 当前 shape={waypoints.shape}"
        )
    if waypoints.shape[0] < window_points:
        # 触发条件 (a) 不满足
        return 0.0

    p_end = waypoints[-1]
    p_back = waypoints[-window_points]
    heading_vec = p_end - p_back
    start = waypoints[0, :2]
    target_ray = goal - start
    goal_vec_end = goal - p_end

    h_norm = float(np.linalg.norm(heading_vec))
    ray_norm = float(np.linalg.norm(target_ray))
    end_goal_norm = float(np.linalg.norm(goal_vec_end))

    # (b) 末端几乎不动
    if h_norm < heading_eps:
        return 0.0
    # 起点落在成功奖励生效阈值内
    if success_effective_threshold is not None and ray_norm <= max(
        success_effective_threshold, goal_eps
    ):
        return 0.0
    # (e) 起点本身已在成功范围内
    if ray_norm < max(success_threshold, goal_eps):
        return 0.0
    # (c) 末端已经足够接近目标
    if end_goal_norm < max(success_threshold, goal_eps):
        return 0.0

    # (d) 投影需在 [0, 1] 内
    t = _project_on_segment_t(p_end, goal, start)
    if math.isnan(t) or t < 0.0 or t > 1.0:
        return 0.0

    # 计算夹角 (0 ~ 180°): 末端方向 vs 起点->目标射线
    cos_theta = float(np.dot(heading_vec, target_ray) / (h_norm * ray_norm))
    cos_theta = max(-1.0, min(1.0, cos_theta))
    theta_deg = math.degrees(math.acos(cos_theta))

    # 线性偏离量。对任意 free_angle_deg 均归一到 [0, 1].
    over = max(0.0, theta_deg - float(free_angle_deg))
    denom = 180.0 - float(free_angle_deg)
    if denom <= 1e-12:
        return 0.0
    return float(over / denom)


@register_reward_fn("terminal_heading")
class TerminalHeadingRewardFn(RewardFn):
    """终点朝向奖励函数 (惩罚项, 0~1, 越小越好).

    config 字段:
        window_points: 末端方向估计窗口 K, 默认 4 (heading = p[-1] - p[-4]).
        free_angle_deg: 免罚角度阈值 (度), 默认 45.
        success_threshold: 距目标 < 此阈值时不再约束方向 (米), 默认 0.3.
            建议与 SuccessRewardFn 的 threshold 保持一致.
        action_scale: NavDP 反归一化因子, 默认 4.0.
        heading_eps / goal_eps: 数值稳定阈值, 通常无需修改.

    输出:
        torch.Tensor [batch], 0~1 偏离量. CompositeRewardFn 中应配合**负权重**使用.

    依赖字段:
        trajectories[i]['actions']    [T, >=2]
        trajectories[i]['goal_point'] [2] 或 [3] (取前 2 维)
        trajectories[i]['waypoints']  可选, 若上游已预计算则直接复用
    """

    name: str = "terminal_heading"

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)
        self.window_points = int(self.config.get("window_points", 4))
        if self.window_points < 2:
            raise ValueError(
                f"window_points 必须 >= 2, 当前={self.window_points}"
            )
        self.free_angle_deg = float(self.config.get("free_angle_deg", 45.0))
        if self.free_angle_deg < 0 or self.free_angle_deg > 180:
            raise ValueError(
                f"free_angle_deg 必须 ∈ [0, 180], 当前={self.free_angle_deg}"
            )
        self.success_threshold = float(
            self.config.get("success_threshold", 0.3)
        )
        if self.success_threshold < 0:
            raise ValueError(
                f"success_threshold 必须 >= 0, 当前={self.success_threshold}"
            )
        self.action_scale = float(self.config.get("action_scale", 4.0))
        explicit_effective = self.config.get("success_effective_threshold")
        if explicit_effective is not None:
            self.success_effective_threshold = float(explicit_effective)
            if self.success_effective_threshold < 0:
                raise ValueError(
                    "success_effective_threshold 必须 >= 0, "
                    f"当前={self.success_effective_threshold}"
                )
        else:
            self.success_effective_threshold = derive_success_effective_threshold(
                target_speed=float(self.config.get("target_speed", 0.8)),
                frame_dt=float(self.config.get("frame_dt", 1.0 / 30.0)),
                sample_interval=int(self.config.get("sample_interval", 4)),
                horizon_points=int(
                    self.config.get("success_effective_horizon_points", 24)
                ),
            )
        self.heading_eps = float(self.config.get("heading_eps", 1e-3))
        self.goal_eps = float(self.config.get("goal_eps", 1e-6))

    def _extract_goal(self, traj: Dict[str, Any], idx: int) -> np.ndarray:
        goal_point = traj.get("goal_point")
        if goal_point is None:
            raise ValueError(
                f"轨迹 {idx} 缺少 goal_point, 无法计算终点朝向奖励"
            )
        if isinstance(goal_point, torch.Tensor):
            g = goal_point.detach().cpu().numpy()
        else:
            g = np.asarray(goal_point)
        if g.ndim > 1:
            g = g.squeeze()
        return g.reshape(-1)[:2].astype(np.float64)

    def _extract_waypoints(
        self, traj: Dict[str, Any], idx: int
    ) -> np.ndarray:
        if traj.get("waypoints_with_start") is not None:
            wp = traj["waypoints_with_start"]
            if isinstance(wp, torch.Tensor):
                wp = wp.detach().cpu().numpy()
            else:
                wp = np.asarray(wp)
            return wp.astype(np.float64)
        if traj.get("waypoints") is not None:
            wp = traj["waypoints"]
            if isinstance(wp, torch.Tensor):
                wp = wp.detach().cpu().numpy()
            else:
                wp = np.asarray(wp)
            return ensure_waypoints_include_start(
                wp.astype(np.float64),
                traj.get("start_pos"),
            )
        actions = traj.get("actions")
        if actions is None:
            raise ValueError(
                f"轨迹 {idx} 缺少 actions / waypoints, 无法计算终点朝向奖励"
            )
        return actions_to_waypoints_with_start(
            actions,
            action_scale=self.action_scale,
            start_xy=traj.get("start_pos"),
        )

    def compute(
        self,
        trajectories: List[Dict[str, Any]],
        **kwargs: Any,
    ) -> torch.Tensor:
        device = kwargs.get("device", torch.device("cpu"))
        dtype = kwargs.get("dtype", torch.float32)

        rewards: List[float] = []
        for idx, traj in enumerate(trajectories):
            waypoints = self._extract_waypoints(traj, idx)
            goal = self._extract_goal(traj, idx)
            reward = compute_terminal_heading_reward(
                waypoints,
                goal,
                window_points=self.window_points,
                free_angle_deg=self.free_angle_deg,
                success_threshold=self.success_threshold,
                success_effective_threshold=self.success_effective_threshold,
                heading_eps=self.heading_eps,
                goal_eps=self.goal_eps,
            )
            rewards.append(reward)

        return torch.tensor(rewards, device=device, dtype=dtype)
