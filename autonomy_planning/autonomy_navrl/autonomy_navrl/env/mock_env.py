"""Mock vectorized environment for CI pipeline checks only (no Isaac sensors).

RGBD observations are zeros — visual navigation training must use the Isaac Lab
backend with TiledCamera + Imu sensors (``--backend isaac``).
"""

from __future__ import annotations

import warnings
from typing import Any

import numpy as np

from autonomy_navrl.control.commands import scale_action_to_command
from autonomy_navrl.core.spec import FrameworkConfig
from autonomy_navrl.env.base_env import BaseNavrlEnv, EnvStepResult
from autonomy_navrl.env.mock_reward import NavigationReward, RewardConfig
from autonomy_navrl.core.sampling import sample_goal_positions
from autonomy_navrl.core.state import PROPRIO_STATE_DIM, build_proprioceptive_state


class MockNavrlEnv(BaseNavrlEnv):
    """State-only 2D stub for PPO/CLI smoke tests without Isaac Sim."""

    def __init__(self, config: dict[str, Any]) -> None:
        warnings.warn(
            'Mock backend does not provide Isaac Sim RGBD/IMU sensors. '
            'Use --backend isaac for visual navigation training.',
            stacklevel=2,
        )
        self._config = config
        self._num_envs = int(config.get('num_envs', 4))
        sensor_cfg = config.get('sensors', {})
        rgb_cfg = sensor_cfg.get('rgb', {})
        self._width = int(rgb_cfg.get('width', 128))
        self._height = int(rgb_cfg.get('height', 96))
        self._max_depth = float(sensor_cfg.get('depth', {}).get('max_range_m', 5.0))
        control_cfg = config.get('control', {})
        try:
            fw = FrameworkConfig.from_yaml_dict(config)
            self._action_dim = fw.action.dim
        except ValueError:
            self._action_dim = int(control_cfg.get('action_dim', 3))
        self._max_vx = float(control_cfg.get('max_vx', 1.0))
        self._max_vy = float(control_cfg.get('max_vy', 0.5))
        self._max_w = float(control_cfg.get('max_w', 1.5))
        task_cfg = config.get('task', {})
        self._goal_tolerance = float(task_cfg.get('goal_tolerance_m', 0.5))
        self._max_steps = int(task_cfg.get('max_episode_steps', 500))
        reward_cfg = config.get('reward', {})
        self._reward = NavigationReward(
            RewardConfig(
                progress_scale=float(reward_cfg.get('progress_scale', 1.0)),
                goal_reached=float(reward_cfg.get('goal_reached', 10.0)),
                collision_penalty=float(reward_cfg.get('collision_penalty', -5.0)),
                proximity_penalty_scale=float(reward_cfg.get('proximity_penalty_scale', -0.5)),
                action_smoothness_scale=float(reward_cfg.get('action_smoothness_scale', -0.01)),
                timeout_penalty=float(reward_cfg.get('timeout_penalty', -1.0)),
                goal_tolerance_m=self._goal_tolerance,
                collision_threshold_m=float(
                    sensor_cfg.get('depth', {}).get('collision_threshold_m', 0.35)
                ),
            )
        )
        self._state_dim = int(config.get('policy', {}).get('state_dim', PROPRIO_STATE_DIM))
        self._rng = np.random.default_rng(int(config.get('seed', 42)))
        self._positions = np.zeros((self._num_envs, 2), dtype=np.float32)
        self._yaws = np.zeros((self._num_envs,), dtype=np.float32)
        self._goals = np.zeros((self._num_envs, 2), dtype=np.float32)
        self._step_counts = np.zeros((self._num_envs,), dtype=np.int32)
        self._obstacle_distance = np.full((self._num_envs,), 2.0, dtype=np.float32)

    @property
    def num_envs(self) -> int:
        return self._num_envs

    @property
    def action_dim(self) -> int:
        return 3

    def reset(self) -> dict[str, np.ndarray]:
        self._positions.fill(0.0)
        self._yaws.fill(0.0)
        self._goals = sample_goal_positions(
            self._num_envs,
            float(self._config.get('task', {}).get('arena_size_m', 16.0)),
            float(self._config.get('task', {}).get('goal_distance_m', 8.0)) * 0.5,
            self._rng,
        )
        self._step_counts.fill(0)
        self._obstacle_distance = self._rng.uniform(0.8, 3.0, size=self._num_envs).astype(np.float32)
        distance = np.linalg.norm(self._goals - self._positions, axis=-1)
        self._reward.reset(distance)
        return self._build_observation()

    def step(self, actions: np.ndarray) -> EnvStepResult:
        actions = np.asarray(actions, dtype=np.float32).reshape(self._num_envs, 3)
        commands = [
            scale_action_to_command(row, self._max_vx, self._max_vy, self._max_w).as_array()
            for row in actions
        ]
        command_array = np.stack(commands, axis=0)
        dt = 0.05
        for index in range(self._num_envs):
            vx, vy, w = command_array[index]
            cos_yaw = np.cos(self._yaws[index])
            sin_yaw = np.sin(self._yaws[index])
            self._positions[index, 0] += (cos_yaw * vx - sin_yaw * vy) * dt
            self._positions[index, 1] += (sin_yaw * vx + cos_yaw * vy) * dt
            self._yaws[index] += w * dt
            self._obstacle_distance[index] = max(
                0.2,
                self._obstacle_distance[index]
                - float(self._rng.uniform(-0.05, 0.08))
                - max(0.0, 1.5 - np.linalg.norm(command_array[index, :2])) * 0.01,
            )

        self._step_counts += 1
        distance = np.linalg.norm(self._goals - self._positions, axis=-1)
        depth_maps = self._synthetic_depth()
        terminated_goal = distance < self._goal_tolerance
        terminated_collision = self._obstacle_distance < 0.35
        truncated = self._step_counts >= self._max_steps
        terminated = terminated_goal | terminated_collision
        reward, metrics = self._reward.compute(
            distance,
            depth_maps,
            actions,
            terminated_collision,
            terminated_goal,
            truncated,
        )
        obs = self._build_observation()
        reset_mask = terminated | truncated
        if np.any(reset_mask):
            reset_ids = np.where(reset_mask)[0]
            new_goals = sample_goal_positions(
                reset_ids.shape[0],
                float(self._config.get('task', {}).get('arena_size_m', 16.0)),
                float(self._config.get('task', {}).get('goal_distance_m', 8.0)) * 0.5,
                self._rng,
            )
            self._positions[reset_ids] = 0.0
            self._yaws[reset_ids] = 0.0
            self._goals[reset_ids] = new_goals
            self._step_counts[reset_ids] = 0
            self._obstacle_distance[reset_ids] = self._rng.uniform(
                0.8, 3.0, size=reset_ids.shape[0]
            ).astype(np.float32)
            self._reward.reset(np.linalg.norm(self._goals - self._positions, axis=-1))
        return EnvStepResult(
            observation=obs,
            reward=reward,
            terminated=terminated.astype(np.bool_),
            truncated=truncated.astype(np.bool_),
            info={'metrics': metrics, 'backend': 'mock'},
        )

    def close(self) -> None:
        return

    def _synthetic_depth(self) -> np.ndarray:
        depth = np.full((self._num_envs, self._height, self._width), self._max_depth, np.float32)
        for index in range(self._num_envs):
            depth[index, :, :] = self._obstacle_distance[index]
        return depth

    def _build_observation(self) -> dict[str, np.ndarray]:
        rgbd = np.zeros((self._num_envs, 4, self._height, self._width), dtype=np.float32)
        state = np.zeros((self._num_envs, self._state_dim), dtype=np.float32)
        for index in range(self._num_envs):
            state[index] = build_proprioceptive_state(
                self._positions[index],
                float(self._yaws[index]),
                self._goals[index],
                state_dim=self._state_dim,
            )
        return {'rgbd': rgbd, 'state': state}
