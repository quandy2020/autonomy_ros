"""Checkpoint inference for deployment."""

from __future__ import annotations

from typing import Any

import numpy as np
import torch

from autonomy_navrl.core.types import DeployConfig, VelocityCommand
from autonomy_navrl.control.commands import clip_velocity_command, scale_action_to_command
from autonomy_navrl.models.checkpoint import load_checkpoint_payload
from autonomy_navrl.models.factory import create_nav_policy
from autonomy_navrl.preprocessing.rgbd import stack_rgbd


class PolicyInference:
    """Load a trained checkpoint and produce velocity commands."""

    def __init__(self, config: DeployConfig) -> None:
        self._config = config
        self._device = torch.device(
            config.device if torch.cuda.is_available() else 'cpu'
        )
        payload = load_checkpoint_payload(config.checkpoint, self._device)
        raw_config: dict[str, Any] = payload.get('config', {})
        self._policy = create_nav_policy(raw_config, device=self._device)
        self._policy.load_state_dict(payload['policy_state_dict'])
        self._policy.eval()
        self._max_depth = float(
            raw_config.get('sensors', {}).get('depth', {}).get('max_range_m', 5.0)
        )

    @property
    def config(self) -> DeployConfig:
        return self._config

    def infer(
        self,
        rgb: np.ndarray,
        depth_m: np.ndarray,
        state_vector: np.ndarray,
    ) -> VelocityCommand:
        """Run deterministic inference."""
        rgbd = stack_rgbd(
            rgb,
            depth_m,
            self._config.image_width,
            self._config.image_height,
            self._max_depth,
        )
        rgbd_t = torch.as_tensor(rgbd, device=self._device).unsqueeze(0)
        state = np.asarray(state_vector, dtype=np.float32).reshape(1, -1)
        state_t = torch.as_tensor(state, device=self._device)
        with torch.no_grad():
            action, _, _, _ = self._policy.act_tensors(rgbd_t, state_t, deterministic=True)
        command = scale_action_to_command(
            action.squeeze(0).cpu().numpy(),
            self._config.max_vx,
            self._config.max_vy,
            self._config.max_w,
        )
        return clip_velocity_command(
            command,
            self._config.max_vx,
            self._config.max_vy,
            self._config.max_w,
        )
