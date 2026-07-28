"""InternVLA-N1 dual-system ROS policy wrapper."""

from __future__ import annotations

import numpy as np

from autonomy_internnav.config import Config
from autonomy_internnav.internvla_n1.agent.base import Agent
import autonomy_internnav.internvla_n1.agent.internvla_n1_agent  # noqa: F401
from autonomy_internnav.internvla_n1.configs import AgentCfg
from autonomy_internnav.policies.base import InferenceResult, PolicyInference


_DISCRETE_TO_VEL = {
    0: (0.0, 0.0),
    1: (0.3, 0.0),
    2: (0.0, 0.5),
    3: (0.0, -0.5),
    5: (-0.1, 0.0),
}


class InternVLAN1PolicyInference(PolicyInference):
    """Wrap InternVLA-N1 Agent for ROS bridge (discrete action → short trajectory)."""

    def __init__(self, cfg: Config, intrinsic: np.ndarray | None = None) -> None:
        super().__init__(intrinsic)
        self._cfg = cfg
        self._agent = None
        self._instruction = getattr(cfg, 'instruction', '') or 'navigate to the goal'

    def _ensure_agent(self) -> None:
        if self._agent is not None:
            return
        settings = {
            'policy_name': 'InternVLAN1_Policy',
            'device': self._cfg.device,
            'width': self._cfg.image_size,
            'height': self._cfg.image_size,
            'hfov': 90.0,
            'infer_mode': getattr(self._cfg, 'infer_mode', 'partial_async'),
            'sys2_max_forward_step': getattr(self._cfg, 'sys2_max_forward_step', 8),
            'vis_debug': False,
            'model': {
                'policy_name': 'InternVLAN1_Policy',
                'device': self._cfg.device,
                'ckpt_path': self._cfg.checkpoint,
            },
        }
        agent_cfg = AgentCfg(
            model_name='internvla_n1',
            ckpt_path=self._cfg.checkpoint,
            model_settings=settings,
        )
        self._agent = Agent.init(agent_cfg)

    def reset(self) -> None:
        if self._agent is not None:
            self._agent.reset()

    def _obs(self, rgb_bgr: np.ndarray, depth_m: np.ndarray) -> list[dict]:
        rgb = rgb_bgr[..., ::-1]
        depth_mm = (depth_m * 10000.0).astype(np.float32)
        if depth_mm.ndim == 2:
            depth_mm = depth_mm[..., np.newaxis]
        return [{'rgb': rgb, 'depth': depth_mm, 'instruction': self._instruction}]

    def _discrete_to_trajectory(self, action_idx: int) -> np.ndarray:
        linear, angular = _DISCRETE_TO_VEL.get(int(action_idx), (0.0, 0.0))
        traj = np.zeros((self._cfg.predict_size, 3), dtype=np.float32)
        for i in range(1, self._cfg.predict_size):
            dt = 0.1
            traj[i, 0] = traj[i - 1, 0] + linear * dt
            traj[i, 2] = traj[i - 1, 2] + angular * dt
        return traj

    def step_pointgoal(
        self,
        goal_xy: np.ndarray,
        rgb_bgr: np.ndarray,
        depth_m: np.ndarray,
    ) -> InferenceResult:
        self._ensure_agent()
        out = self._agent.step(self._obs(rgb_bgr, depth_m))
        action = out.get('action', [0])[0]
        traj = self._discrete_to_trajectory(action)
        return InferenceResult(
            trajectory=traj,
            all_trajectory=traj[None, ...],
            values=np.array([0.0], dtype=np.float32),
            trajectory_mask=None,
            stopped=int(action) == 0,
        )

    def step_imagegoal(
        self,
        goal_rgb: np.ndarray,
        rgb_bgr: np.ndarray,
        depth_m: np.ndarray,
    ) -> InferenceResult:
        return self.step_pointgoal(np.zeros(2, dtype=np.float32), rgb_bgr, depth_m)

    def step_nogoal(self, rgb_bgr: np.ndarray, depth_m: np.ndarray) -> InferenceResult:
        return self.step_pointgoal(np.zeros(2, dtype=np.float32), rgb_bgr, depth_m)
