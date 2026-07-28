"""组合奖励 (CompositeRewardFn).

按权重组合多个分项奖励, 并返回分项明细供日志/离线评估使用.

支持的分项:
    - collision           (>=0)  原始膨胀点碰撞
    - occupancy           (>=0)  原始膨胀点占据
    - success             (0/1)  到达目标成功率
    - progress            (>=0)  进度差
    - projected_collision (>=0)  深度图投影碰撞 (可选, 需 depth_image)
    - projected_occupancy (>=0)  深度图投影占据 (可选)
    - speed_limit         (>=0)  legacy 限速软惩罚 (兼容旧 pipeline; 默认权重 0)
    - target_speed        (>=0)  围绕目标速度对称偏离量 (新增)
    - smooth              (>=0)  步长均值惩罚 (新增)
    - terminal_heading    (0~1)  终点朝向偏离 (新增)
    - ttc                 (>=0)  Time-To-Collision 风险代价 (新增)
    - goal_direction_projection (>=0) 目标方向投影奖励 (新增)

总奖励 = sum(weight_k * detail_k).

注意:
    - target_speed / smooth / terminal_heading 输出"偏离量"(>=0), 与 speed_limit
      一致, 因此其权重应配置为**负数**才能形成惩罚.
    - 默认权重: target_speed_weight = -1.0, smooth_weight = -1.0,
      terminal_heading_weight = -1.0; 用户可按需调整.
    - speed_limit_weight 默认 0.0 (即关闭旧限速分项), 改为使用 target_speed.
"""

from typing import Any, Dict, List, Optional

import torch

from .base_reward import RewardFn, register_reward_fn
from .collision_reward import CollisionRewardFn
from .occupancy_reward import OccupancyRewardFn, ProjectedOccupancyRewardFn
from .progress_reward import ProgressRewardFn
from .projected_collision_reward import ProjectedCollisionRewardFn
from .smooth_reward import SmoothRewardFn
from .speed_limit_reward import SpeedLimitRewardFn
from .success_reward import SuccessRewardFn
from .target_speed_reward import TargetSpeedRewardFn
from .terminal_heading_reward import TerminalHeadingRewardFn
from .trajectory_utils import actions_to_waypoints, actions_to_waypoints_with_start
from .goal_direction_projection_reward import GoalDirectionProjectionRewardFn
from .ttc_reward import TTCRewardFn


@register_reward_fn("composite")
class CompositeRewardFn(RewardFn):
    """组合奖励函数, 按权重组合各分项奖励.

    可选支持投影障碍物奖励（深度图实时投影）, 以及 target_speed / smooth /
    terminal_heading 三个新分项.
    """

    name: str = "composite"

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)
        # ----- 基础分项权重 -----
        self.collision_weight = float(self.config.get("collision_weight", -5.0))
        self.occupancy_weight = float(self.config.get("occupancy_weight", -1.0))
        self.success_weight = float(self.config.get("success_weight", 10.0))
        self.progress_weight = float(self.config.get("progress_weight", 3.0))

        # ----- 投影分项权重 (默认 0, 关闭) -----
        self.projected_collision_weight = float(
            self.config.get("projected_collision_weight", 0.0)
        )
        self.projected_occupancy_weight = float(
            self.config.get("projected_occupancy_weight", 0.0)
        )

        # ----- 限速 / 目标速度 / 平滑 / 终点朝向 -----
        # legacy speed_limit (默认关闭, 由 target_speed 取代)
        self.speed_limit_weight = float(self.config.get("speed_limit_weight", 0.0))
        # 新增三项 (默认全部 -1.0, 即"偏离量"被作为惩罚累加)
        self.target_speed_weight = float(
            self.config.get("target_speed_weight", -1.0)
        )
        self.smooth_weight = float(self.config.get("smooth_weight", -1.0))
        self.terminal_heading_weight = float(
            self.config.get("terminal_heading_weight", -1.0)
        )
        # TTC (Time-To-Collision) 风险代价, 输出 >= 0, 权重应为负
        self.ttc_weight = float(self.config.get("ttc_weight", -8.0))

        # ----- 构造各子奖励 (共享 config 字段) -----
        # RewardCfg / CLI 使用带前缀的 key (如 smooth_reduce),
        # 而子函数内部用短 key (如 reduce). 这里做一次映射,
        # 同时保留原始长 key, 保证单独实例化子函数时两种 key 均可读到.
        sub_cfg = dict(self.config)
        _KEY_ALIASES: dict = {
            # SmoothRewardFn
            "smooth_reduce": "reduce",
            # TargetSpeedRewardFn
            "target_speed_slope": "slope",
            # TerminalHeadingRewardFn
            "terminal_heading_window_points": "window_points",
            "terminal_heading_free_angle_deg": "free_angle_deg",
            "terminal_heading_success_threshold": "success_threshold",
        }
        for long_key, short_key in _KEY_ALIASES.items():
            if long_key in sub_cfg and short_key not in sub_cfg:
                sub_cfg[short_key] = sub_cfg[long_key]

        self.collision_fn = CollisionRewardFn(config=sub_cfg)
        self.occupancy_fn = OccupancyRewardFn(config=sub_cfg)
        self.success_fn = SuccessRewardFn(config=sub_cfg)
        self.progress_fn = ProgressRewardFn(config=sub_cfg)
        self.projected_collision_fn = ProjectedCollisionRewardFn(config=sub_cfg)
        self.projected_occupancy_fn = ProjectedOccupancyRewardFn(config=sub_cfg)
        self.speed_limit_fn = SpeedLimitRewardFn(config=sub_cfg)
        self.target_speed_fn = TargetSpeedRewardFn(config=sub_cfg)
        self.smooth_fn = SmoothRewardFn(config=sub_cfg)
        self.terminal_heading_fn = TerminalHeadingRewardFn(config=sub_cfg)
        self.ttc_fn = TTCRewardFn(config=sub_cfg)

        # 目标方向投影奖励 (默认关闭, 设正权重启用)
        self.goal_direction_projection_weight = float(
            self.config.get("goal_direction_projection_weight", 0.0)
        )
        self.goal_direction_projection_fn = GoalDirectionProjectionRewardFn(config=sub_cfg)

    # ---------------------------------------------------------------------
    # 投影模式判定
    # ---------------------------------------------------------------------
    def _is_projected_mode(self, trajectories: List[Dict[str, Any]]) -> bool:
        """判断是否进入纯投影模式 (用深度图投影替代 inflation_points).

        仅当 **所有** 轨迹都有 depth_image + camera_intrinsic 且 **全部都没有**
        inflation_points 时才视为投影模式。
        混合情况 (部分有 inflation_points 部分没有) 属于数据问题, 优先使用
        inflation_points 模式, 避免静默地把障碍物奖励全部置 0.
        """
        all_have_projected = all(
            "depth_image" in traj and traj["depth_image"] is not None
            and "camera_intrinsic" in traj and traj["camera_intrinsic"] is not None
            for traj in trajectories
        )
        any_have_inflation = any(
            "inflation_points" in traj and traj["inflation_points"] is not None
            for traj in trajectories
        )
        return all_have_projected and not any_have_inflation

    def _compute_projected_rewards(
        self,
        trajectories: List[Dict[str, Any]],
        **kwargs: Any,
    ) -> tuple:
        """计算投影障碍物奖励（如果轨迹包含必需字段）。"""
        has_projected_fields = all(
            "depth_image" in traj and traj["depth_image"] is not None
            and "camera_intrinsic" in traj and traj["camera_intrinsic"] is not None
            for traj in trajectories
        )

        if has_projected_fields and (
            self.projected_collision_weight != 0.0
            or self.projected_occupancy_weight != 0.0
        ):
            projected_collision = self.projected_collision_fn.compute(
                trajectories, **kwargs
            )
            projected_occupancy = self.projected_occupancy_fn.compute(
                trajectories, **kwargs
            )
            return projected_collision, projected_occupancy

        return None, None

    def _with_shared_waypoints(
        self,
        trajectories: List[Dict[str, Any]],
        device: torch.device,
        dtype: torch.dtype,
    ) -> List[Dict[str, Any]]:
        """统一预计算轨迹点，供多个奖励项复用.

        约定:
        - 所有奖励统一在数据默认的局部坐标系下计算
        - ``waypoints``: 不含起点, 形状 [T, 2]，供碰撞/占据类奖励复用
        - ``waypoints_with_start``: 含局部原点起点, 形状 [T+1, 2]，
          供 success/progress/terminal_heading 复用
        """
        action_scale = float(self.config.get("action_scale", 4.0))
        prepared: List[Dict[str, Any]] = []
        for traj in trajectories:
            new_traj = dict(traj)
            if new_traj.get("waypoints") is None:
                wp = actions_to_waypoints(
                    new_traj["actions"],
                    action_scale=action_scale,
                )
                new_traj["waypoints"] = torch.from_numpy(wp).to(device=device, dtype=dtype)
            if new_traj.get("waypoints_with_start") is None:
                wp_full = actions_to_waypoints_with_start(
                    new_traj["actions"],
                    action_scale=action_scale,
                )
                new_traj["waypoints_with_start"] = torch.from_numpy(wp_full).to(device=device, dtype=dtype)
            prepared.append(new_traj)
        return prepared

    # ---------------------------------------------------------------------
    # 主入口
    # ---------------------------------------------------------------------
    def compute(
        self,
        trajectories: List[Dict[str, Any]],
        **kwargs: Any,
    ) -> Dict[str, torch.Tensor]:
        """计算组合奖励并返回分项明细.

        Returns:
            Dict 包含:
                "reward":              [batch] 总奖励
                "collision":           [batch]
                "occupancy":           [batch]
                "success":             [batch]
                "progress":            [batch]
                "speed_limit":         [batch] legacy 限速 (默认权重 0)
                "target_speed":        [batch] 新增, >=0
                "smooth":              [batch] 新增, >=0
                "terminal_heading":    [batch] 新增, 0~1
                "ttc":                 [batch] 新增, >=0
                "projected_collision": [batch] 可选
                "projected_occupancy": [batch] 可选
        """
        device = kwargs.get("device", torch.device("cpu"))
        dtype = kwargs.get("dtype", torch.float32)
        trajectories = self._with_shared_waypoints(trajectories, device=device, dtype=dtype)

        is_projected_mode = self._is_projected_mode(trajectories)

        # 投影模式下原始 collision/occupancy 不可用 (缺 inflation_points), 一律置 0
        if is_projected_mode:
            collision = torch.zeros(len(trajectories), device=device, dtype=dtype)
            occupancy = torch.zeros(len(trajectories), device=device, dtype=dtype)
            ttc = torch.zeros(len(trajectories), device=device, dtype=dtype)
        else:
            collision = self.collision_fn.compute(trajectories, **kwargs)
            occupancy = self.occupancy_fn.compute(trajectories, **kwargs)
            # TTC 也依赖 inflation_points, 与 collision/occupancy 一起计算
            if self.ttc_weight != 0.0:
                ttc = self.ttc_fn.compute(trajectories, **kwargs)
            else:
                ttc = torch.zeros(len(trajectories), device=device, dtype=dtype)

        success = self.success_fn.compute(trajectories, **kwargs)
        progress = self.progress_fn.compute(trajectories, **kwargs)

        # legacy 限速
        if self.speed_limit_weight != 0.0:
            speed_limit = self.speed_limit_fn.compute(trajectories, **kwargs)
        else:
            speed_limit = torch.zeros(
                len(trajectories), device=device, dtype=dtype
            )

        # 新增 3 项: 权重为 0 时跳过计算以省时
        if self.target_speed_weight != 0.0:
            target_speed = self.target_speed_fn.compute(trajectories, **kwargs)
        else:
            target_speed = torch.zeros(
                len(trajectories), device=device, dtype=dtype
            )

        if self.smooth_weight != 0.0:
            smooth = self.smooth_fn.compute(trajectories, **kwargs)
        else:
            smooth = torch.zeros(len(trajectories), device=device, dtype=dtype)

        if self.terminal_heading_weight != 0.0:
            terminal_heading = self.terminal_heading_fn.compute(
                trajectories, **kwargs
            )
        else:
            terminal_heading = torch.zeros(
                len(trajectories), device=device, dtype=dtype
            )

        if self.goal_direction_projection_weight != 0.0:
            goal_direction_projection = self.goal_direction_projection_fn.compute(
                trajectories, **kwargs
            )
        else:
            goal_direction_projection = torch.zeros(
                len(trajectories), device=device, dtype=dtype
            )

        reward = (
            self.collision_weight * collision
            + self.occupancy_weight * occupancy
            + self.success_weight * success
            + self.progress_weight * progress
            + self.speed_limit_weight * speed_limit
            + self.target_speed_weight * target_speed
            + self.smooth_weight * smooth
            + self.terminal_heading_weight * terminal_heading
            + self.ttc_weight * ttc
            + self.goal_direction_projection_weight * goal_direction_projection
        )

        result = {
            "reward": reward,
            "collision": collision,
            "occupancy": occupancy,
            "success": success,
            "progress": progress,
            "speed_limit": speed_limit,
            "target_speed": target_speed,
            "smooth": smooth,
            "terminal_heading": terminal_heading,
            "ttc": ttc,
            "goal_direction_projection": goal_direction_projection,
        }

        # 投影分项 (可选)
        projected_collision, projected_occupancy = self._compute_projected_rewards(
            trajectories, **kwargs
        )
        if projected_collision is not None:
            result["projected_collision"] = projected_collision
            reward = reward + self.projected_collision_weight * projected_collision
            result["reward"] = reward

        if projected_occupancy is not None:
            result["projected_occupancy"] = projected_occupancy
            reward = reward + self.projected_occupancy_weight * projected_occupancy
            result["reward"] = reward

        return result
