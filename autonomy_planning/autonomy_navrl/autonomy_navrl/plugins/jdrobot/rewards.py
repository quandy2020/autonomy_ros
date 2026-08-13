"""Reward terms adapted from Isaac Lab jdrobot precision navigation MDP."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import torch


@dataclass
class TanhPositionTerm:
    std: float
    weight: float


@dataclass
class SmoothBrakingTerm:
    max_dist: float
    max_speed: float
    weight: float


@dataclass
class JdrobotRewardConfig:
    """YAML-driven jdrobot-style reward weights."""

    position_tanh: list[TanhPositionTerm] = field(default_factory=list)
    smooth_braking: list[SmoothBrakingTerm] = field(default_factory=list)
    progress_scale: float = 1.0
    proximity_scale: float = -0.5
    collision_penalty: float = -5.0
    goal_reached: float = 10.0
    timeout_penalty: float = -1.0
    action_rate_weight: float = -0.05
    action_jerk_weight: float = -0.05
    bearing_near_goal_weight: float = -2.0
    bearing_near_goal_dist_m: float = 0.5
    vx_w_product_weight: float = -0.1
    termination_collision_penalty: float = -400.0

    @classmethod
    def from_dict(cls, reward_cfg: dict[str, Any]) -> JdrobotRewardConfig:
        jd = reward_cfg.get('jdrobot', {})
        tanh_terms = [
            TanhPositionTerm(std=float(t['std']), weight=float(t['weight']))
            for t in jd.get('position_tanh', [])
        ]
        if not tanh_terms:
            tanh_terms = [
                TanhPositionTerm(0.2, 0.5),
                TanhPositionTerm(0.1, 2.0),
                TanhPositionTerm(0.02, 3.0),
                TanhPositionTerm(0.01, 4.0),
            ]
        braking_terms = [
            SmoothBrakingTerm(
                max_dist=float(t['max_dist']),
                max_speed=float(t['max_speed']),
                weight=float(t['weight']),
            )
            for t in jd.get('smooth_braking', [])
        ]
        if not braking_terms:
            braking_terms = [
                SmoothBrakingTerm(0.72, 0.36, -8.0),
                SmoothBrakingTerm(0.12, 0.04, -12.0),
            ]
        return cls(
            position_tanh=tanh_terms,
            smooth_braking=braking_terms,
            progress_scale=float(reward_cfg.get('progress_scale', 1.0)),
            proximity_scale=float(reward_cfg.get('proximity_penalty_scale', -0.5)),
            collision_penalty=float(reward_cfg.get('collision_penalty', -5.0)),
            goal_reached=float(reward_cfg.get('goal_reached', 10.0)),
            timeout_penalty=float(reward_cfg.get('timeout_penalty', -1.0)),
            action_rate_weight=float(jd.get('action_rate_weight', -0.05)),
            action_jerk_weight=float(jd.get('action_jerk_weight', -0.05)),
            bearing_near_goal_weight=float(jd.get('bearing_near_goal_weight', -2.0)),
            bearing_near_goal_dist_m=float(jd.get('bearing_near_goal_dist_m', 0.5)),
            vx_w_product_weight=float(jd.get('vx_w_product_weight', -0.1)),
            termination_collision_penalty=float(
                jd.get('termination_collision_penalty', -400.0)
            ),
        )


class JdrobotRewardComputer:
    """Compute jdrobot-inspired navigation rewards on DirectRLEnv tensors."""

    def __init__(self, cfg: JdrobotRewardConfig, device: torch.device) -> None:
        self.cfg = cfg
        self.device = device
        self._prev_action_rate = None

    def reset(self, env_ids: torch.Tensor) -> None:
        if self._prev_action_rate is None:
            return
        self._prev_action_rate[env_ids] = 0.0

    def compute(
        self,
        *,
        distance: torch.Tensor,
        bearing: torch.Tensor,
        progress: torch.Tensor,
        proximity: torch.Tensor,
        commands: torch.Tensor,
        prev_commands: torch.Tensor,
        lin_speed_b: torch.Tensor,
        reached: torch.Tensor,
        collision_terminated: torch.Tensor,
        timeout: torch.Tensor,
    ) -> tuple[torch.Tensor, dict[str, float]]:
        cfg = self.cfg
        reward = progress * cfg.progress_scale

        for term in cfg.position_tanh:
            reward += term.weight * (1.0 - torch.tanh(distance / term.std))

        for term in cfg.smooth_braking:
            target_speed = (distance / term.max_dist) * term.max_speed
            target_speed = torch.clamp(target_speed, 0.0, term.max_speed)
            speed_error = torch.square(torch.clamp(lin_speed_b - target_speed, min=0.0))
            reward += term.weight * speed_error

        reward += proximity * cfg.proximity_scale

        action_rate = torch.sum(torch.square(commands - prev_commands), dim=-1)
        reward += cfg.action_rate_weight * action_rate

        current_rate = commands - prev_commands
        if self._prev_action_rate is None:
            self._prev_action_rate = torch.zeros_like(current_rate)
        jerk = current_rate - self._prev_action_rate
        reward += cfg.action_jerk_weight * torch.sum(torch.square(jerk), dim=-1)
        self._prev_action_rate = current_rate.detach()

        near_goal = (distance < cfg.bearing_near_goal_dist_m).float()
        reward += cfg.bearing_near_goal_weight * torch.square(bearing) * near_goal

        reward += cfg.vx_w_product_weight * torch.abs(commands[:, 0] * commands[:, 2])

        reward = torch.where(reached, reward + cfg.goal_reached, reward)
        reward = torch.where(proximity > 0.25, cfg.collision_penalty, reward)
        reward = torch.where(collision_terminated, cfg.termination_collision_penalty, reward)
        reward = torch.where(timeout, cfg.timeout_penalty, reward)

        metrics = {
            'reward_mean': float(reward.mean().detach().cpu()),
            'goal_rate': float(reached.float().mean().detach().cpu()),
            'collision_rate': float((proximity > 0.35).float().mean().detach().cpu()),
            'progress_mean': float(progress.mean().detach().cpu()),
            'position_tanh_mean': float(
                (1.0 - torch.tanh(distance / cfg.position_tanh[-1].std)).mean().detach().cpu()
            ),
        }
        return reward, metrics
