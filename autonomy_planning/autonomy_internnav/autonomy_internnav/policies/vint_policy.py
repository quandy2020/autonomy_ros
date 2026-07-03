"""ViNT policy inference (image-goal / no-goal)."""

from __future__ import annotations

import numpy as np

from autonomy_internnav.config import Config
from autonomy_internnav.paths import baseline_config_path, resolve_checkpoint
from autonomy_internnav.policies.base import InferenceResult, PolicyInference, to_numpy


class ViNTPolicyInference(PolicyInference):
    """ViNT visual navigation transformer."""

    def __init__(self, cfg: Config, intrinsic: np.ndarray | None = None) -> None:
        super().__init__(intrinsic)
        self._cfg = cfg
        self._agent = None

    def _ensure_agent(self):
        if self._agent is not None:
            return self._agent

        from autonomy_internnav.baselines.vint.vint_agent import ViNTAgent

        intrinsic = self._intrinsic if self._intrinsic is not None else np.eye(3, dtype=np.float64)
        model_path = resolve_checkpoint(self._cfg.checkpoint)
        model_config = self._cfg.model_config or baseline_config_path('vint', 'vint.yaml')
        robot_config = self._cfg.robot_config or baseline_config_path('vint', 'robot_config.yaml')

        self._agent = ViNTAgent(
            image_intrinsic=intrinsic,
            model_path=model_path,
            model_config_path=model_config,
            robot_config_path=robot_config,
            device=self._cfg.device,
        )
        self._agent.reset(self._cfg.batch_size)
        return self._agent

    def reset(self) -> None:
        if self._agent is not None:
            self._agent.reset(self._cfg.batch_size)

    def step_pointgoal(self, goal_xy, rgb_bgr, depth_m) -> InferenceResult:
        raise NotImplementedError(
            'ViNT does not support point goals; set goal_type:=image and publish image_goal')

    def step_imagegoal(self, goal_rgb, rgb_bgr, depth_m) -> InferenceResult:
        agent = self._ensure_agent()
        images = rgb_bgr[np.newaxis, ...]
        goal = goal_rgb[np.newaxis, ...]
        _waypoints, trajectory = agent.step_imagegoal(goal, images)
        traj = to_numpy(trajectory)
        if traj.ndim == 3:
            traj = traj[0]
        return InferenceResult(
            trajectory=traj,
            all_trajectory=traj[np.newaxis, ...],
            values=np.zeros(1, dtype=np.float32),
        )

    def step_nogoal(self, rgb_bgr, depth_m) -> InferenceResult:
        agent = self._ensure_agent()
        images = rgb_bgr[np.newaxis, ...]
        _waypoints, trajectory = agent.step_nogoal(images)
        traj = to_numpy(trajectory)
        if traj.ndim == 3:
            traj = traj[0]
        return InferenceResult(
            trajectory=traj,
            all_trajectory=traj[np.newaxis, ...],
            values=np.zeros(1, dtype=np.float32),
        )
