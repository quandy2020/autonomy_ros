"""TTC (Time-To-Collision) 奖励 (TTCRewardFn).

基于物理意义的TTC计算：假设机器人保持当前运动趋势（速度和航向角），
最快还有多少秒就会与障碍物碰撞。

物理TTC计算原理:
    1. 机器人速度: 从actions推导的真实物理速度 (m/s)
    2. 机器人航向角: 从每步位移方向推导 (dx, dy -> heading)
    3. 障碍物速度: 假设为静止障碍物 (数据集中无障碍物速度信息)
    4. 计算方法: 基于相对运动学，计算机器人沿运动方向到达最近障碍物的时间

TTC计算步骤 (对每个时刻 t，考虑机器人当前位置 P_t 和速度向量 v_t):
    1. 计算机器人速度向量: v_t = (P_{t+1} - P_t) / dt
    2. 计算机器人航向角: theta_t = atan2(vy, vx)
    3. 对于每个障碍物 O_j:
       - rel_pos = O_j - P_t (相对位置向量)
       - 计算障碍物在运动方向上的投影距离和垂直距离
       - ttc = 投影距离 / 速度 (秒)
    
    4. 风险判定: ttc > 0 且 ttc < ttc_threshold 且 垂直距离 < ttc_radius

得分计算:
    渐进式惩罚: TTC值越小惩罚越大，惩罚值在[0, 1]区间连续变化
    - 基础惩罚: TTC在[0, threshold]区间，惩罚从1.0线性衰减到0.0
    - 速度自适应: 高速时惩罚适度增加（每0.5m/s增加speed_boost_rate）
    - 静止不惩罚: 速度低于阈值时不计惩罚
    **进入成功判定范围(success_threshold)的轨迹点不计算TTC风险，得分强制为0**
    整条轨迹TTC得分 = sum(每个步的惩罚值)，范围[0, H]，H为轨迹点数(默认24)

输出: [batch_size] float tensor, 值 >= 0, 越大越差.
在 CompositeRewardFn 中配合**负权重**使用 (ttc_weight 默认 -8.0).

参考: docs/ttc_architecture_summary.md
"""

from typing import Any, Dict, List, Optional

import torch

from .base_reward import RewardFn, register_reward_fn
from .trajectory_utils import (
    actions_to_waypoints_with_start,
    derive_dt,
)


@register_reward_fn("ttc")
class TTCRewardFn(RewardFn):
    """TTC (Time-To-Collision) 风险代价奖励函数.

    输出值 >= 0, 越大代表碰撞风险越高; 在组合奖励中配合负权重使用.
    """

    name: str = "ttc"

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)
        # TTC 时间窗口阈值 (秒)
        self.ttc_threshold = float(self.config.get("ttc_threshold", 2.0))
        # 危险判定半径 (米)
        self.ttc_radius = float(self.config.get("ttc_radius", 0.35))
        # 动作缩放因子
        self.action_scale = float(self.config.get("action_scale", 4.0))
        # 时间参数: 用于计算真实速度 (m/s)
        self.frame_dt = float(self.config.get("frame_dt", 1.0 / 30.0))
        self.sample_interval = int(self.config.get("sample_interval", 4))
        self.dt = derive_dt(self.frame_dt, self.sample_interval)
        
        # 成功判定阈值: 轨迹点在此范围内时不计算TTC风险
        self.success_threshold = float(self.config.get("success_threshold", 0.3))
        if self.success_threshold <= 0:
            raise ValueError(
                f"success_threshold 必须为正数, 当前值: {self.success_threshold}"
            )
        
        # 渐进式惩罚新增参数
        # 速度加权系数: 每增加 0.5m/s, 惩罚增加的比例
        self.speed_boost_rate = float(self.config.get("speed_boost_rate", 0.2))

    def _compute_ttc_penalty(
        self,
        ttc: torch.Tensor,
        risk_mask: torch.Tensor,
        step_speed: torch.Tensor,
        H: int,
        device: torch.device,
    ) -> torch.Tensor:
        """渐进式TTC惩罚计算.
        
        Args:
            ttc: 每个障碍物的TTC值 [H, N]
            risk_mask: 风险判定掩码 [H, N] (TTC>0 & TTC<阈值 & 距离小于半径 & 运动状态)
            step_speed: 每步速度 [H]
            H: 轨迹步数
            device: 计算设备
        
        Returns:
            step_penalty: 每步的惩罚值 [H], 范围 [0, 1]
        """
        # 1. 基础惩罚：分段线性函数
        # TTC在(0, threshold]区间：惩罚从1.0线性衰减到0.0
        # TTC <= 0 的障碍物在后方，基础惩罚为 0
        ttc_clamped = torch.clamp(ttc, 0.0, self.ttc_threshold)
        base_penalty = 1.0 - (ttc_clamped / self.ttc_threshold)
        # TTC <= 0 时，base_penalty 会是 1.0，需要修正为 0
        base_penalty = torch.where(ttc > 0, base_penalty, torch.zeros_like(base_penalty))
        
        # 2. 速度自适应加权
        # 高速时需要更大的安全余量，惩罚适度增加
        # 速度因子：速度每增加0.5m/s，惩罚增加 speed_boost_rate
        speed_boost = 1.0 + self.speed_boost_rate * torch.clamp(
            step_speed.unsqueeze(1) / 0.5, 0.0, 5.0
        )
        adaptive_penalty = base_penalty * speed_boost
        
        # 3. 只保留风险区域的惩罚
        # 注：risk_mask 已包含运动状态判断，静止步的 risk_mask 全为 False
        adaptive_penalty = torch.where(risk_mask, adaptive_penalty, torch.zeros_like(adaptive_penalty))
        
        # 4. 每个时间步取最危险障碍物的惩罚值
        step_penalty = adaptive_penalty.max(dim=1)[0]
        
        # 5. 裁剪到 [0, 1] 区间，避免浮点误差导致超出范围
        return torch.clamp(step_penalty, 0.0, 1.0)

    def _compute_one(self, traj: Dict[str, Any], device: torch.device, dtype: torch.dtype) -> float:
        """计算单条轨迹的 TTC 代价."""
        # 获取轨迹点 (含起点)
        if traj.get("waypoints_with_start") is not None:
            points = traj["waypoints_with_start"].to(device=device, dtype=dtype)
        else:
            wp_full = actions_to_waypoints_with_start(
                traj["actions"],
                action_scale=self.action_scale,
            )
            points = torch.from_numpy(wp_full).to(device=device, dtype=dtype)

        # 规则：轨迹点在成功判定范围(success_threshold)内时不计算TTC风险
        goal_point = traj.get("goal_point")
        if goal_point is not None:
            if isinstance(goal_point, torch.Tensor):
                goal = goal_point.to(device=device, dtype=dtype)
            else:
                goal = torch.tensor(goal_point, device=device, dtype=dtype)
            if goal.ndim > 1:
                goal = goal.squeeze()
            goal = goal[:2]  # 只取 xy
        else:
            goal = None

        # 获取障碍物
        if "inflation_points" not in traj or traj["inflation_points"] is None:
            # 缺少障碍物信息, 无法计算 TTC, 返回 0
            return 0.0
        obstacles = traj["inflation_points"].to(device=device, dtype=dtype)

        # 过滤填充点
        if obstacles.shape[0] == 0:
            return 0.0
        obstacles = obstacles[:, :2]
        valid_mask = obstacles.abs().amax(dim=-1) < 1e5
        obstacles = obstacles[valid_mask]
        if obstacles.shape[0] == 0:
            return 0.0

        H = points.shape[0] - 1  # 轨迹段数
        if H < 1:
            return 0.0

        # 计算轨迹段向量 (单位: 米)
        segments = points[1:] - points[:-1]  # [H, 2]
        
        # 计算真实物理速度 (m/s)
        step_speed = torch.norm(segments, dim=-1) / self.dt  # [H]
        
        # 前置条件: 至少有一步处于运动状态
        if (step_speed > 1e-6).sum() == 0:
            return 0.0

        # 计算机器人速度向量 (m/s)
        velocity_vectors = segments / self.dt  # [H, 2]

        # 向量化 TTC 计算
        # starts: [H, 1, 2], obstacles: [1, N, 2]
        starts = points[:-1].unsqueeze(1)  # [H, 1, 2]
        obs = obstacles.unsqueeze(0)        # [1, N, 2]

        # 相对位置向量 (障碍物相对于机器人当前位置)
        rel_pos = obs - starts  # [H, N, 2]

        # 机器人速度向量 (广播)
        robot_vel = velocity_vectors.unsqueeze(1)  # [H, 1, 2]

        # 计算障碍物在机器人运动方向上的投影距离
        # 投影距离 = dot(rel_pos, robot_vel_direction)
        speed = step_speed.unsqueeze(1)  # [H, 1], 复用 step_speed
        speed_for_direction = step_speed.view(-1, 1, 1)  # [H, 1, 1]
        
        # 机器人运动方向的单位向量
        robot_vel_direction = robot_vel / (speed_for_direction + 1e-12)  # [H, 1, 2]
        
        # 障碍物在运动方向上的投影距离 (正值表示前方，负值表示后方)
        projection = (rel_pos * robot_vel_direction).sum(dim=-1)  # [H, N]
        
        # 障碍物到运动轨迹的垂直距离
        # 使用叉积的z分量 (2D叉积): cross_z = x1*y2 - y1*x2
        cross_z = rel_pos[..., 0] * robot_vel_direction[..., 1] - \
                  rel_pos[..., 1] * robot_vel_direction[..., 0]  # [H, N]
        perpendicular_dist = torch.abs(cross_z)  # [H, N]
        
        # TTC 计算: 假设机器人沿当前方向直线运动
        # TTC = projection / speed (秒)
        # 障碍物数据已在前方FOV过滤中处理，无需额外判断前方条件
        ttc = projection / (speed + 1e-12)  # [H, N]
        
        # 对于速度接近零的步，ttc 设为无穷大
        ttc = torch.where(
            step_speed.unsqueeze(1) > 1e-12, 
            ttc, 
            torch.full_like(ttc, float('inf'))
        )

        # 障碍物在 TTC 时刻与机器人的最近距离等于垂直距离
        closest_dist = perpendicular_dist  # [H, N]

        # 运动掩码
        moving = step_speed > 1e-6  # [H]

        # 成功判定掩码: 轨迹点在 success_threshold 范围内时不计算TTC
        if goal is not None:
            point_dists = torch.norm(points[:-1, :2] - goal, dim=-1)  # [H]
            near_goal = point_dists <= self.success_threshold  # [H]
        else:
            near_goal = torch.zeros(H, dtype=torch.bool, device=device)  # [H]

        # 风险掩码: 
        # 1. ttc > 0 (未来会碰撞)
        # 2. ttc < ttc_threshold (在风险时间窗口内)
        # 3. closest_dist < ttc_radius (碰撞时距离小于危险半径)
        # 4. 机器人处于运动状态
        # 5. 轨迹点不在成功判定范围内
        risk = (
            (ttc > 0)
            & (ttc < self.ttc_threshold)
            & (closest_dist < self.ttc_radius)
            & moving.unsqueeze(1)
            & (~near_goal.unsqueeze(1))
        )  # [H, N]

        # 使用渐进式惩罚计算
        step_penalty = self._compute_ttc_penalty(ttc, risk, step_speed, H, device)
        
        # 累加所有时间步的惩罚作为整条轨迹的TTC得分
        ttc_final = step_penalty.sum()  # 标量，范围[0, H]

        return ttc_final.item()

    def compute(self, trajectories: List[Dict[str, Any]], **kwargs: Any) -> torch.Tensor:
        """计算 batch TTC 代价.

        Args:
            trajectories: rollout 轨迹列表, 每个元素必须包含:
                - actions: [predict_size, 3] 增量动作
                - inflation_points: [N, 2] 局部坐标系下的膨胀障碍点
            **kwargs: device, dtype 等.

        Returns:
            torch.Tensor: 形状 [batch_size] 的 float, 值 >= 0, 越大越差.
        """
        device = kwargs.get("device", torch.device("cpu"))
        dtype = kwargs.get("dtype", torch.float32)

        if len(trajectories) == 0:
            return torch.empty(0, device=device, dtype=dtype)

        rewards = []
        for traj in trajectories:
            rewards.append(self._compute_one(traj, device, dtype))

        return torch.tensor(rewards, device=device, dtype=dtype)
