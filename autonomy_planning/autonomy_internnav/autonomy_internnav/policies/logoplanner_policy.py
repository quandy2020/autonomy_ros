"""LoGoPlanner policy inference."""

from __future__ import annotations

import numpy as np

from autonomy_internnav.config import Config
from autonomy_internnav.paths import resolve_checkpoint
from autonomy_internnav.policies.base import InferenceResult, PolicyInference, to_numpy


class LoGoPlannerPolicyInference(PolicyInference):
    """LoGoPlanner localization-grounded policy."""

    def __init__(self, cfg: Config, intrinsic: np.ndarray | None = None) -> None:
        super().__init__(intrinsic)
        self._cfg = cfg
        self._agent = None

    def _ensure_agent(self):
        if self._agent is not None:
            return self._agent

        from autonomy_internnav.baselines.logoplanner.policy_agent import LoGoPlanner_Agent

        intrinsic = self._intrinsic if self._intrinsic is not None else np.eye(3, dtype=np.float64)
        checkpoint = resolve_checkpoint(self._cfg.checkpoint)

        self._agent = LoGoPlanner_Agent(
            image_intrinsic=intrinsic,
            image_size=self._cfg.image_size,
            memory_size=self._cfg.memory_size,
            predict_size=self._cfg.predict_size,
            temporal_depth=self._cfg.temporal_depth,
            heads=self._cfg.heads,
            token_dim=self._cfg.token_dim,
            navi_model=checkpoint,
            device=self._cfg.device,
        )
        self._agent.reset(self._cfg.batch_size, self._cfg.stop_threshold)
        return self._agent

    def reset(self) -> None:
        if self._agent is not None:
            self._agent.reset(self._cfg.batch_size, self._cfg.stop_threshold)

    def step_pointgoal(self, goal_xy, rgb_bgr, depth_m) -> InferenceResult:
        agent = self._ensure_agent()
        goals = np.asarray(goal_xy, dtype=np.float32).reshape(1, 3)
        images = rgb_bgr[np.newaxis, ...]
        depths = depth_m[np.newaxis, ...]
        trajectory, all_trajectory, values, mask, _pd = agent.step_pointgoal(
            goals, images, depths)
        values_np = to_numpy(values)
        flat = np.asarray(values_np, dtype=np.float64).reshape(-1)
        stopped = flat.size > 0 and float(flat.max()) < self._cfg.stop_threshold
        return InferenceResult(
            trajectory=to_numpy(trajectory),
            all_trajectory=to_numpy(all_trajectory),
            values=values_np,
            trajectory_mask=to_numpy(mask) if mask is not None else None,
            stopped=stopped,
        )
