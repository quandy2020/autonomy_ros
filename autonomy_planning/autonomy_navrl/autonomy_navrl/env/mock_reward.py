"""Reward computation for mock (non-Isaac) navigation environments."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from autonomy_navrl.preprocessing.rgbd import depth_collision_score


@dataclass
class RewardConfig:
    """Reward shaping parameters."""

    progress_scale: float = 1.0
    goal_reached: float = 10.0
    collision_penalty: float = -5.0
    proximity_penalty_scale: float = -0.5
    action_smoothness_scale: float = -0.01
    timeout_penalty: float = -1.0
    goal_tolerance_m: float = 0.5
    collision_threshold_m: float = 0.35


class NavigationReward:
    """Compute scalar reward from navigation and RGBD signals."""

    def __init__(self, config: RewardConfig) -> None:
        self._config = config
        self._prev_distance: np.ndarray | None = None
        self._prev_action: np.ndarray | None = None

    def reset(self, distance_to_goal: np.ndarray) -> None:
        """Reset per-episode reward state."""
        self._prev_distance = distance_to_goal.astype(np.float32).copy()
        self._prev_action = np.zeros((distance_to_goal.shape[0], 3), dtype=np.float32)

    def compute(
        self,
        distance_to_goal: np.ndarray,
        depth_maps: np.ndarray,
        actions: np.ndarray,
        terminated_collision: np.ndarray,
        terminated_goal: np.ndarray,
        truncated: np.ndarray,
    ) -> tuple[np.ndarray, dict[str, float]]:
        """Compute reward vector and aggregated metrics."""
        if self._prev_distance is None or self._prev_action is None:
            self.reset(distance_to_goal)

        progress = self._prev_distance - distance_to_goal
        reward = progress * self._config.progress_scale

        proximity = np.array([
            depth_collision_score(depth, self._config.collision_threshold_m)
            for depth in depth_maps
        ], dtype=np.float32)
        reward += proximity * self._config.proximity_penalty_scale

        action_delta = actions - self._prev_action
        smooth_penalty = np.sum(action_delta * action_delta, axis=-1)
        reward += smooth_penalty * self._config.action_smoothness_scale

        reward = np.where(terminated_goal, reward + self._config.goal_reached, reward)
        reward = np.where(terminated_collision, self._config.collision_penalty, reward)
        reward = np.where(truncated, reward + self._config.timeout_penalty, reward)

        self._prev_distance = distance_to_goal.astype(np.float32).copy()
        self._prev_action = actions.astype(np.float32).copy()

        metrics = {
            'reward_mean': float(np.mean(reward)),
            'progress_mean': float(np.mean(progress)),
            'proximity_mean': float(np.mean(proximity)),
            'goal_rate': float(np.mean(terminated_goal)),
            'collision_rate': float(np.mean(terminated_collision)),
        }
        return reward.astype(np.float32), metrics
