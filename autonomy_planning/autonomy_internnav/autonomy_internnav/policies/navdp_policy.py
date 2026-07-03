"""NavDP policy inference for the ROS bridge."""

from __future__ import annotations

import numpy as np
import torch

from autonomy_internnav.config import Config
from autonomy_internnav.navdp.agent import NavDP_Agent
from autonomy_internnav.paths import resolve_checkpoint
from autonomy_internnav.policies.base import InferenceResult, PolicyInference, to_numpy


class NavDPPolicyInference(PolicyInference):
    """NavDP diffusion policy ROS adapter."""

    def __init__(self, cfg: Config, intrinsic: np.ndarray | None = None) -> None:
        super().__init__(intrinsic)
        self._cfg = cfg
        self._agent: NavDP_Agent | None = None

    @PolicyInference.intrinsic.setter
    def intrinsic(self, value: np.ndarray | None) -> None:
        self._intrinsic = value
        if self._agent is not None and value is not None:
            self._agent.image_intrinsic = value

    def _ensure_agent(self) -> NavDP_Agent:
        if self._agent is not None:
            return self._agent

        intrinsic = (
            self._intrinsic if self._intrinsic is not None
            else np.eye(3, dtype=np.float64)
        )
        self._agent = NavDP_Agent(
            image_intrinsic=intrinsic,
            image_size=self._cfg.image_size,
            memory_size=self._cfg.memory_size,
            predict_size=self._cfg.predict_size,
            temporal_depth=self._cfg.temporal_depth,
            heads=self._cfg.heads,
            token_dim=self._cfg.token_dim,
            navi_model=resolve_checkpoint(self._cfg.checkpoint),
            device=self._cfg.device,
        )
        self._reset_agent()
        return self._agent

    def render_overlay(self, rgb_bgr, all_trajectory, values) -> np.ndarray | None:
        agent = self._ensure_agent()
        images = rgb_bgr[np.newaxis, ...]
        max_samples = None
        if not self._cfg.overlay_show_all_samples:
            max_samples = max(1, int(self._cfg.overlay_max_samples))
        return agent.project_trajectory(
            images,
            np.asarray(all_trajectory),
            np.asarray(values),
            max_samples=max_samples,
        )

    def _reset_agent(self) -> None:
        if self._agent is None:
            return
        self._agent.reset(
            self._cfg.batch_size,
            self._cfg.stop_threshold,
            self._cfg.sample_num,
        )
        self._agent.project_overlay = not self._cfg.overlay_decouple

    def reset(self) -> None:
        self._reset_agent()

    def _maybe_seed_inference(self) -> None:
        if self._cfg.inference_seed < 0:
            return
        torch.manual_seed(self._cfg.inference_seed)
        if torch.cuda.is_available():
            torch.cuda.manual_seed_all(self._cfg.inference_seed)

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

    def _run_step(self, step_fn) -> InferenceResult:
        agent = self._ensure_agent()
        self._maybe_seed_inference()
        with torch.inference_mode():
            trajectory, all_trajectory, values, mask = step_fn(agent)
        return self._build_result(trajectory, all_trajectory, values, mask)

    def _pack_rgbd(self, rgb_bgr: np.ndarray, depth_m: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        return rgb_bgr[np.newaxis, ...], depth_m[np.newaxis, ...]

    def step_pointgoal(self, goal_xy, rgb_bgr, depth_m) -> InferenceResult:
        goals = np.asarray(goal_xy, dtype=np.float32).reshape(1, 3)
        images, depths = self._pack_rgbd(rgb_bgr, depth_m)
        return self._run_step(
            lambda agent: agent.step_pointgoal(goals, images, depths),
        )

    def step_imagegoal(self, goal_rgb, rgb_bgr, depth_m) -> InferenceResult:
        goals = np.asarray(goal_rgb, dtype=np.float32)[np.newaxis, ...]
        images, depths = self._pack_rgbd(rgb_bgr, depth_m)
        return self._run_step(
            lambda agent: agent.step_imagegoal(goals, images, depths),
        )

    def step_pixelgoal(self, pixel_uv, rgb_bgr, depth_m) -> InferenceResult:
        pixels = np.asarray(pixel_uv, dtype=np.float32).reshape(1, 2)
        images, depths = self._pack_rgbd(rgb_bgr, depth_m)
        return self._run_step(
            lambda agent: agent.step_pixelgoal(pixels, images, depths),
        )

    def step_point_image_goal(self, goal_xy, goal_rgb, rgb_bgr, depth_m) -> InferenceResult:
        point_goals = np.asarray(goal_xy, dtype=np.float32).reshape(1, 3)
        image_goals = np.asarray(goal_rgb, dtype=np.float32)[np.newaxis, ...]
        images, depths = self._pack_rgbd(rgb_bgr, depth_m)
        return self._run_step(
            lambda agent: agent.step_point_image_goal(
                point_goals, image_goals, images, depths,
            ),
        )

    def step_nogoal(self, rgb_bgr, depth_m) -> InferenceResult:
        images, depths = self._pack_rgbd(rgb_bgr, depth_m)
        return self._run_step(
            lambda agent: agent.step_nogoal(images, depths),
        )
