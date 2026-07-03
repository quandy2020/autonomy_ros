"""VIPlanner policy inference."""

from __future__ import annotations

import numpy as np

from autonomy_internnav.config import Config
from autonomy_internnav.paths import baseline_config_path, resolve_asset
from autonomy_internnav.policies.base import InferenceResult, PolicyInference, to_numpy


class VIPlannerPolicyInference(PolicyInference):
    """VIPlanner semantic navigation policy."""

    def __init__(self, cfg: Config, intrinsic: np.ndarray | None = None) -> None:
        super().__init__(intrinsic)
        self._cfg = cfg
        self._agent = None

    def _ensure_agent(self):
        if self._agent is not None:
            return self._agent

        from autonomy_internnav.baselines.viplanner.viplanner_agent import VIPlannerAgent

        import torch

        intrinsic = self._intrinsic if self._intrinsic is not None else np.eye(3, dtype=np.float64)
        model_path = resolve_checkpoint(self._cfg.checkpoint)
        model_config = (
            self._cfg.model_config
            or baseline_config_path('viplanner', 'viplanner.yaml')
        )
        if not self._cfg.m2f_checkpoint or not self._cfg.m2f_config:
            raise FileNotFoundError(
                'VIPlanner requires m2f_checkpoint and m2f_config ROS parameters '
                '(Mask2Former weights and mmdet config)')
        m2f_ckpt = resolve_asset(self._cfg.m2f_checkpoint)
        m2f_cfg = self._cfg.m2f_config

        self._agent = VIPlannerAgent(
            image_intrinsic=torch.as_tensor(intrinsic, dtype=torch.float32),
            m2f_path=m2f_ckpt,
            m2f_config_path=m2f_cfg,
            model_path=model_path,
            model_config_path=model_config,
            device=self._cfg.device,
        )
        return self._agent

    def reset(self) -> None:
        return

    def step_pointgoal(self, goal_xy, rgb_bgr, depth_m) -> InferenceResult:
        agent = self._ensure_agent()
        rgb_rgb = rgb_bgr[..., ::-1]
        images = rgb_rgb[np.newaxis, ...]
        depths = depth_m[np.newaxis, ...]
        goals = np.asarray(goal_xy, dtype=np.float32).reshape(1, 3)
        _keypoints, trajectory, fear = agent.step_pointgoal(images, depths, goals)
        traj = to_numpy(trajectory)
        if traj.ndim == 3:
            traj = traj[0]
        values = to_numpy(fear).reshape(-1)
        return InferenceResult(
            trajectory=traj,
            all_trajectory=traj[np.newaxis, ...],
            values=values,
        )
