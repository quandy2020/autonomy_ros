"""NavDP policy inference."""

from __future__ import annotations

import numpy as np

from autonomy_internnav.config import Config
from autonomy_internnav.paths import resolve_checkpoint
from autonomy_internnav.policies.base import InferenceResult, PolicyInference, to_numpy


class NavDPPolicyInference(PolicyInference):
    """NavDP diffusion policy."""

    def __init__(self, cfg: Config, intrinsic: np.ndarray | None = None) -> None:
        super().__init__(intrinsic)
        self._cfg = cfg
        self._agent = None

    def _ensure_agent(self):
        if self._agent is not None:
            return self._agent

        from autonomy_internnav.baselines.navdp.policy_agent import NavDP_Agent

        intrinsic = self._intrinsic if self._intrinsic is not None else np.eye(3, dtype=np.float64)
        checkpoint = resolve_checkpoint(self._cfg.checkpoint)

        self._agent = NavDP_Agent(
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
        self._reset_agent()
        return self._agent

    def _reset_agent(self) -> None:
        if self._agent is not None:
            self._agent.reset(
                self._cfg.batch_size,
                self._cfg.stop_threshold,
                self._cfg.sample_num,
            )

    def reset(self) -> None:
        self._reset_agent()

    def _pack_point_inputs(self, goal, rgb_bgr, depth_m):
        images = rgb_bgr[np.newaxis, ...]
        depths = depth_m[np.newaxis, ...]
        goals = np.asarray(goal, dtype=np.float32).reshape(1, 3)
        return goals, images, depths

    def _pack_image_goal(self, goal_rgb, rgb_bgr, depth_m):
        images = rgb_bgr[np.newaxis, ...]
        depths = depth_m[np.newaxis, ...]
        goals = np.asarray(goal_rgb, dtype=np.float32)[np.newaxis, ...]
        return goals, images, depths

    def _build_result(
        self,
        trajectory,
        all_trajectory,
        values,
        mask,
    ) -> InferenceResult:
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

    def step_pointgoal(self, goal_xy, rgb_bgr, depth_m) -> InferenceResult:
        agent = self._ensure_agent()
        goals, images, depths = self._pack_point_inputs(goal_xy, rgb_bgr, depth_m)
        trajectory, all_trajectory, values, mask = agent.step_pointgoal(goals, images, depths)
        return self._build_result(trajectory, all_trajectory, values, mask)

    def step_imagegoal(self, goal_rgb, rgb_bgr, depth_m) -> InferenceResult:
        agent = self._ensure_agent()
        goals, images, depths = self._pack_image_goal(goal_rgb, rgb_bgr, depth_m)
        trajectory, all_trajectory, values, mask = agent.step_imagegoal(goals, images, depths)
        return self._build_result(trajectory, all_trajectory, values, mask)

    def step_pixelgoal(self, pixel_uv, rgb_bgr, depth_m) -> InferenceResult:
        agent = self._ensure_agent()
        images = rgb_bgr[np.newaxis, ...]
        depths = depth_m[np.newaxis, ...]
        pixels = np.asarray(pixel_uv, dtype=np.float32).reshape(1, 2)
        trajectory, all_trajectory, values, mask = agent.step_pixelgoal(pixels, images, depths)
        return self._build_result(trajectory, all_trajectory, values, mask)

    def step_point_image_goal(self, goal_xy, goal_rgb, rgb_bgr, depth_m) -> InferenceResult:
        agent = self._ensure_agent()
        point_goals, images, depths = self._pack_point_inputs(goal_xy, rgb_bgr, depth_m)
        image_goals = np.asarray(goal_rgb, dtype=np.float32)[np.newaxis, ...]
        trajectory, all_trajectory, values, mask = agent.step_point_image_goal(
            point_goals, image_goals, images, depths)
        return self._build_result(trajectory, all_trajectory, values, mask)

    def step_nogoal(self, rgb_bgr, depth_m) -> InferenceResult:
        agent = self._ensure_agent()
        images = rgb_bgr[np.newaxis, ...]
        depths = depth_m[np.newaxis, ...]
        trajectory, all_trajectory, values, mask = agent.step_nogoal(images, depths)
        return self._build_result(trajectory, all_trajectory, values, mask)
