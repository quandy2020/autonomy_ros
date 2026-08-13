"""Go2W-specific reward computers (register on import)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import torch

from autonomy_navrl.plugins.rewards import BaseRewardComputer, RewardStepContext, reward_registry


@dataclass
class TanhScaleTerm:
    std: float
    weight: float


@dataclass
class SmoothBrakingTerm:
    max_dist: float
    max_speed: float
    weight: float


@dataclass
class PrecisionParkingConfig:
    """Parking-phase reward: micro-moves + yaw trim, relaxed near-goal braking."""

    parking_zone_m: float = 0.40
    yaw_gate_dist_m: float = 0.50
    position_tolerance_m: float = 0.08
    yaw_tolerance_rad: float = 0.12

    progress_scale: float = 1.0
    proximity_scale: float = -0.5
    collision_penalty: float = -5.0
    goal_reached: float = 50.0
    timeout_penalty: float = -1.0
    termination_collision_penalty: float = -400.0
    action_rate_weight: float = -0.03
    action_jerk_weight: float = -0.03

    position_tanh: list[TanhScaleTerm] = field(default_factory=list)
    yaw_tanh: list[TanhScaleTerm] = field(default_factory=list)
    smooth_braking: list[SmoothBrakingTerm] = field(default_factory=list)

    micro_motion_weight: float = 3.0
    micro_motion_yaw_weight: float = 2.5
    micro_motion_max_lin_mps: float = 0.25
    micro_motion_max_ang_rps: float = 1.0

    parking_max_lin_mps: float = 0.22
    parking_overspeed_weight: float = -3.0

    stuck_dist_m: float = 0.25
    stuck_yaw_thresh_rad: float = 0.12
    stuck_yaw_penalty: float = -2.5

    iou_blend_weight: float = 1.5
    use_bbox_iou: bool = False
    iou_threshold: float = 0.95

    @classmethod
    def from_dict(cls, reward_cfg: dict[str, Any]) -> PrecisionParkingConfig:
        pp = reward_cfg.get('precision_parking', {})
        pos_terms = [
            TanhScaleTerm(std=float(t['std']), weight=float(t['weight']))
            for t in pp.get('position_tanh', [])
        ]
        if not pos_terms:
            pos_terms = [
                TanhScaleTerm(0.25, 0.5),
                TanhScaleTerm(0.10, 2.0),
                TanhScaleTerm(0.04, 4.0),
                TanhScaleTerm(0.02, 6.0),
            ]
        yaw_terms = [
            TanhScaleTerm(std=float(t['std']), weight=float(t['weight']))
            for t in pp.get('yaw_tanh', [])
        ]
        if not yaw_terms:
            yaw_terms = [
                TanhScaleTerm(0.35, 0.5),
                TanhScaleTerm(0.15, 2.5),
                TanhScaleTerm(0.08, 4.0),
                TanhScaleTerm(0.04, 6.0),
            ]
        braking_terms = [
            SmoothBrakingTerm(
                max_dist=float(t['max_dist']),
                max_speed=float(t['max_speed']),
                weight=float(t['weight']),
            )
            for t in pp.get('smooth_braking', [])
        ]
        if not braking_terms:
            braking_terms = [
                SmoothBrakingTerm(0.80, 0.40, -5.0),
                SmoothBrakingTerm(0.30, 0.18, -4.0),
            ]
        micro = pp.get('micro_motion', {})
        return cls(
            parking_zone_m=float(pp.get('parking_zone_m', 0.40)),
            yaw_gate_dist_m=float(pp.get('yaw_gate_dist_m', 0.50)),
            position_tolerance_m=float(
                pp.get('position_tolerance_m', reward_cfg.get('position_tolerance_m', 0.08))
            ),
            yaw_tolerance_rad=float(
                pp.get('yaw_tolerance_rad', reward_cfg.get('yaw_tolerance_rad', 0.12))
            ),
            progress_scale=float(reward_cfg.get('progress_scale', 1.0)),
            proximity_scale=float(reward_cfg.get('proximity_penalty_scale', -0.5)),
            collision_penalty=float(reward_cfg.get('collision_penalty', -5.0)),
            goal_reached=float(reward_cfg.get('goal_reached', 50.0)),
            timeout_penalty=float(reward_cfg.get('timeout_penalty', -1.0)),
            termination_collision_penalty=float(
                pp.get('termination_collision_penalty', -400.0)
            ),
            action_rate_weight=float(pp.get('action_rate_weight', -0.03)),
            action_jerk_weight=float(pp.get('action_jerk_weight', -0.03)),
            position_tanh=pos_terms,
            yaw_tanh=yaw_terms,
            smooth_braking=braking_terms,
            micro_motion_weight=float(micro.get('weight', 3.0)),
            micro_motion_yaw_weight=float(micro.get('yaw_weight', 2.5)),
            micro_motion_max_lin_mps=float(micro.get('max_lin_mps', 0.25)),
            micro_motion_max_ang_rps=float(micro.get('max_ang_rps', 1.0)),
            parking_max_lin_mps=float(pp.get('parking_max_lin_mps', 0.22)),
            parking_overspeed_weight=float(pp.get('parking_overspeed_weight', -3.0)),
            stuck_dist_m=float(pp.get('stuck_dist_m', 0.25)),
            stuck_yaw_thresh_rad=float(pp.get('stuck_yaw_thresh_rad', 0.12)),
            stuck_yaw_penalty=float(pp.get('stuck_yaw_penalty', -2.5)),
            iou_blend_weight=float(pp.get('iou_blend_weight', 1.5)),
            use_bbox_iou=bool(pp.get('use_bbox_iou', False)),
            iou_threshold=float(pp.get('iou_threshold', 0.95)),
        )


@reward_registry.register('precision_iou')
class PrecisionIoURewardComputer(BaseRewardComputer):
    """Precision alignment reward with pose IoU metric (Go2W / Isaac Lab style)."""

    name = 'precision_iou'

    def __init__(self, env) -> None:
        super().__init__(env)
        reward_cfg = getattr(self._env.cfg, '_reward_raw', {})
        params = reward_cfg.get('precision_iou', {})
        self._pos_tol = float(params.get('position_tolerance_m', env.cfg.goal_tolerance_m))
        self._yaw_tol = float(params.get('yaw_tolerance_rad', env.cfg.yaw_tolerance_rad))
        self._iou_threshold = float(params.get('iou_threshold', 0.95))
        self._success_bonus = float(params.get('success_bonus', 50.0))
        self._use_bbox_iou = bool(params.get('use_bbox_iou', False))
        self._yaw_shaping_weight = float(params.get('yaw_shaping_weight', 2.0))
        self._yaw_shaping_std = float(params.get('yaw_shaping_std', 0.15))

    def reset(self, env_ids: torch.Tensor) -> None:
        del env_ids

    def _compute_iou(
        self,
        distance: torch.Tensor,
        yaw_error: torch.Tensor,
        dx_body: torch.Tensor | None = None,
        dy_body: torch.Tensor | None = None,
    ) -> torch.Tensor:
        if self._use_bbox_iou and dx_body is not None and dy_body is not None:
            from autonomy_navrl.plugins.go2w.iou import bbox_iou_from_body_pose
            return bbox_iou_from_body_pose(dx_body, dy_body, yaw_error)
        return self._pose_iou(distance, yaw_error, self._pos_tol, self._yaw_tol)

    @staticmethod
    def _pose_iou(distance: torch.Tensor, yaw_error: torch.Tensor, pos_tol: float, yaw_tol: float):
        pos_score = torch.clamp(1.0 - distance / max(pos_tol, 1e-4), min=0.0, max=1.0)
        yaw_score = torch.clamp(1.0 - yaw_error.abs() / max(yaw_tol, 1e-4), min=0.0, max=1.0)
        return pos_score * yaw_score

    def compute(self, ctx: RewardStepContext) -> tuple[torch.Tensor, dict[str, float]]:
        cfg = self._env.cfg
        dx_body = dy_body = None
        task = self._env._task
        if self._use_bbox_iou and hasattr(task, '_body_goal_features'):
            pos_xy = self._env._robot_pos_xy()
            yaw = self._env._robot_yaw()
            dx_body, dy_body, _, _, _ = task._body_goal_features(pos_xy, yaw)
        iou = self._compute_iou(ctx.distance, ctx.yaw_error, dx_body, dy_body)
        reward = iou * 2.0 + ctx.progress * cfg.reward_progress_scale
        reward += ctx.proximity * cfg.reward_proximity_scale
        near = (ctx.distance < cfg.goal_tolerance_m * 2.0).float()
        reward += (
            self._yaw_shaping_weight
            * (1.0 - torch.tanh(ctx.yaw_error.abs() / self._yaw_shaping_std))
            * near
        )
        success = iou >= self._iou_threshold
        reward = torch.where(success, reward + self._success_bonus, reward)
        reward = torch.where(ctx.collision_terminated, cfg.reward_collision_penalty, reward)
        reward = torch.where(ctx.timeout, reward + cfg.reward_timeout_penalty, reward)
        reward = self._env._task.modify_reward(reward, distance=ctx.distance, yaw_error=ctx.yaw_error)
        return reward, {
            'reward_mean': float(reward.mean().detach().cpu()),
            'goal_rate': float(success.float().mean().detach().cpu()),
            'iou_mean': float(iou.mean().detach().cpu()),
            'collision_rate': float(ctx.collision_terminated.float().mean().detach().cpu()),
        }


@reward_registry.register('precision_parking')
class PrecisionParkingRewardComputer(BaseRewardComputer):
    """Near-goal parking reward: yaw shaping, holonomic micro-moves, relaxed braking."""

    name = 'precision_parking'

    def __init__(self, env) -> None:
        super().__init__(env)
        reward_cfg = getattr(env.cfg, '_reward_raw', {})
        self._cfg = PrecisionParkingConfig.from_dict(reward_cfg)
        self._prev_action_rate: torch.Tensor | None = None
        control = getattr(env.cfg._framework, 'control', None) if env.cfg._framework else None
        self._max_vx = float(getattr(control, 'max_vx', 0.5) if control else 0.5)
        self._max_vy = float(getattr(control, 'max_vy', 0.5) if control else 0.5)
        self._max_w = float(getattr(control, 'max_w', 1.0) if control else 1.0)

    def reset(self, env_ids: torch.Tensor) -> None:
        if self._prev_action_rate is None:
            return
        self._prev_action_rate[env_ids] = 0.0

    @staticmethod
    def _signed_align(error: torch.Tensor, command: torch.Tensor, scale: float) -> torch.Tensor:
        return torch.clamp(torch.sign(error) * command / max(scale, 1e-4), -1.0, 1.0)

    def _body_errors(
        self, ctx: RewardStepContext,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        task = self._env._task
        if hasattr(task, '_body_goal_features'):
            pos_xy = self._env._robot_pos_xy()
            yaw = self._env._robot_yaw()
            dx_body, dy_body, _, _, _ = task._body_goal_features(pos_xy, yaw)
            return dx_body, dy_body
        goal_delta = self._env._goals - self._env._robot_pos_xy()
        bearing = torch.atan2(goal_delta[:, 1], goal_delta[:, 0]) - self._env._robot_yaw()
        bearing = torch.atan2(torch.sin(bearing), torch.cos(bearing))
        dist = ctx.distance.clamp(min=1e-4)
        dx_body = dist * torch.cos(bearing)
        dy_body = dist * torch.sin(bearing)
        return dx_body, dy_body

    def compute(self, ctx: RewardStepContext) -> tuple[torch.Tensor, dict[str, float]]:
        cfg = self._cfg
        reward = ctx.progress * cfg.progress_scale

        for term in cfg.position_tanh:
            reward += term.weight * (1.0 - torch.tanh(ctx.distance / term.std))

        yaw_gate = (ctx.distance < cfg.yaw_gate_dist_m).float()
        for term in cfg.yaw_tanh:
            reward += term.weight * (1.0 - torch.tanh(ctx.yaw_error.abs() / term.std)) * yaw_gate

        for term in cfg.smooth_braking:
            target_speed = (ctx.distance / term.max_dist) * term.max_speed
            target_speed = torch.clamp(target_speed, 0.0, term.max_speed)
            speed_error = torch.square(torch.clamp(ctx.lin_speed_b - target_speed, min=0.0))
            reward += term.weight * speed_error

        reward += ctx.proximity * cfg.proximity_scale

        action_rate = torch.sum(torch.square(ctx.commands - ctx.prev_commands), dim=-1)
        reward += cfg.action_rate_weight * action_rate
        current_rate = ctx.commands - ctx.prev_commands
        if self._prev_action_rate is None:
            self._prev_action_rate = torch.zeros_like(current_rate)
        jerk = current_rate - self._prev_action_rate
        reward += cfg.action_jerk_weight * torch.sum(torch.square(jerk), dim=-1)
        self._prev_action_rate = current_rate.detach()

        dx_body, dy_body = self._body_errors(ctx)
        parking = (ctx.distance < cfg.parking_zone_m).float()
        pos_align = 0.5 * (
            self._signed_align(dx_body, ctx.commands[:, 0], self._max_vx)
            + self._signed_align(dy_body, ctx.commands[:, 1], self._max_vy)
        )
        yaw_align = self._signed_align(ctx.yaw_error, ctx.commands[:, 2], self._max_w)
        reward += parking * cfg.micro_motion_weight * (
            pos_align + cfg.micro_motion_yaw_weight * yaw_align
        )

        overspeed = torch.clamp(ctx.lin_speed_b - cfg.parking_max_lin_mps, min=0.0)
        reward += parking * cfg.parking_overspeed_weight * torch.square(overspeed)

        stuck = (
            (ctx.distance < cfg.stuck_dist_m)
            & (ctx.yaw_error.abs() > cfg.stuck_yaw_thresh_rad)
        ).float()
        reward += stuck * cfg.stuck_yaw_penalty * ctx.yaw_error.abs()

        if cfg.iou_blend_weight > 0.0:
            if cfg.use_bbox_iou:
                from autonomy_navrl.plugins.go2w.iou import bbox_iou_from_body_pose
                iou = bbox_iou_from_body_pose(dx_body, dy_body, ctx.yaw_error)
            else:
                pos_score = torch.clamp(
                    1.0 - ctx.distance / max(cfg.position_tolerance_m, 1e-4), min=0.0, max=1.0,
                )
                yaw_score = torch.clamp(
                    1.0 - ctx.yaw_error.abs() / max(cfg.yaw_tolerance_rad, 1e-4),
                    min=0.0, max=1.0,
                )
                iou = pos_score * yaw_score
            reward += cfg.iou_blend_weight * iou
        else:
            iou = torch.zeros_like(ctx.distance)

        reward = torch.where(ctx.reached, reward + cfg.goal_reached, reward)
        reward = torch.where(ctx.proximity > 0.25, cfg.collision_penalty, reward)
        reward = torch.where(ctx.collision_terminated, cfg.termination_collision_penalty, reward)
        reward = torch.where(ctx.timeout, cfg.timeout_penalty, reward)
        reward = self._env._task.modify_reward(reward, distance=ctx.distance, yaw_error=ctx.yaw_error)

        return reward, {
            'reward_mean': float(reward.mean().detach().cpu()),
            'goal_rate': float(ctx.reached.float().mean().detach().cpu()),
            'iou_mean': float(iou.mean().detach().cpu()),
            'collision_rate': float(ctx.collision_terminated.float().mean().detach().cpu()),
            'progress_mean': float(ctx.progress.mean().detach().cpu()),
            'yaw_error_mean': float(ctx.yaw_error.abs().mean().detach().cpu()),
        }
