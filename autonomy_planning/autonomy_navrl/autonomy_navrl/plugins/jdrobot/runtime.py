"""JDRobot runtime helpers (curriculum, action smoothing, reward config)."""

from __future__ import annotations

from typing import Any


def build_action_smoothing_cfg(control_cfg: dict[str, Any]):
    from autonomy_navrl.plugins.jdrobot.action_smoothing import ActionSmoothingConfig

    return ActionSmoothingConfig.from_dict(control_cfg)


def build_goal_curriculum(task_cfg: dict[str, Any]):
    from autonomy_navrl.plugins.jdrobot.curriculum import GoalDistanceCurriculum

    return GoalDistanceCurriculum.from_dict(task_cfg)


def build_jdrobot_reward_cfg(reward_cfg: dict[str, Any]):
    from autonomy_navrl.plugins.jdrobot.rewards import JdrobotRewardConfig

    return JdrobotRewardConfig.from_dict(reward_cfg)


def create_action_smoother(num_envs: int, device, cfg, control_dt: float):
    from autonomy_navrl.plugins.jdrobot.action_smoothing import VelocityActionSmoother

    if cfg is None or not cfg.enabled:
        return None
    return VelocityActionSmoother(num_envs, device, cfg, control_dt)
