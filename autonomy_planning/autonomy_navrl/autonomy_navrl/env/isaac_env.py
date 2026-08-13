"""Isaac Lab environment adapter."""

from __future__ import annotations

from typing import Any

import numpy as np

from autonomy_navrl.core.spec import FrameworkConfig
from autonomy_navrl.env.base_env import BaseNavrlEnv, EnvStepResult


class IsaacLabNavrlEnv(BaseNavrlEnv):
    """Adapter around Isaac Lab DirectRLEnv for PPO training."""

    def __init__(self, config: dict[str, Any]) -> None:
        self._config = config
        self._framework = FrameworkConfig.from_yaml_dict(config)
        self._action_dim = self._framework.action.dim
        self._validate_config()
        self._import_isaaclab()
        self._env = None
        self._bootstrap_simulation()

    @property
    def num_envs(self) -> int:
        assert self._env is not None
        return self._env.num_envs

    @property
    def action_dim(self) -> int:
        return self._action_dim

    def reset(self) -> dict[str, np.ndarray]:
        assert self._env is not None
        print('[INFO]: env.reset() (simulation play + first camera frame)...', flush=True)
        obs_dict, _ = self._env.reset()
        print('[INFO]: env.reset() done.', flush=True)
        return self._to_numpy_obs(obs_dict)

    def step(self, actions: np.ndarray) -> EnvStepResult:
        import torch

        assert self._env is not None
        actions_t = torch.as_tensor(actions, dtype=torch.float32, device=self._env.device)
        obs_dict, reward, terminated, truncated, extras = self._env.step(actions_t)
        metrics = dict(extras.get('metrics', {}))
        if not metrics:
            metrics = {
                'reward_mean': float(reward.mean().detach().cpu()),
                'goal_rate': float(terminated.float().mean().detach().cpu()),
            }
        return EnvStepResult(
            observation=self._to_numpy_obs(obs_dict),
            reward=reward.detach().cpu().numpy().astype(np.float32),
            terminated=terminated.detach().cpu().numpy().astype(bool),
            truncated=truncated.detach().cpu().numpy().astype(bool),
            info={'metrics': metrics, 'backend': 'isaac', 'extras': extras},
        )

    def close(self) -> None:
        if self._env is not None:
            self._env.close()
            self._env = None

    @property
    def direct_env(self):
        """Underlying Isaac Lab DirectRLEnv (for spawn override / diagnostics)."""
        assert self._env is not None
        return self._env

    def set_robot_spawn(self, x: float, y: float, yaw: float, *, env_id: int = 0, z: float | None = None) -> None:
        """Randomize robot start pose in env-local frame (single-env demos)."""
        import torch

        assert self._env is not None
        env_ids = torch.tensor([env_id], device=self._env.device)
        self._env.set_robot_spawn(env_ids, x, y, yaw, z=z)

    def observation_dict(self) -> dict[str, np.ndarray]:
        assert self._env is not None
        return self._to_numpy_obs(self._env._get_observations())

    def capture_viewport_frame(self) -> np.ndarray | None:
        """Return one Isaac viewport RGB frame (HxWx3 uint8) or None if disabled."""
        assert self._env is not None
        if getattr(self._env, 'render_mode', None) != 'rgb_array':
            return None
        return self._env.render(recompute=True)

    def _resolve_render_mode(self) -> str | None:
        video_cfg = self._config.get('video', {})
        source = str(video_cfg.get('source', '')).lower()
        if source == 'viewport':
            return 'rgb_array'
        return None

    def _validate_config(self) -> None:
        if not self._framework.robot.urdf_path:
            raise ValueError('robot.urdf_path must be set for Isaac Lab backend')

    @staticmethod
    def _import_isaaclab() -> None:
        try:
            import isaaclab  # noqa: F401
        except ImportError as exc:
            raise ImportError(
                'Isaac Lab is not installed. Run training inside the Isaac Lab Docker '
                'image or use backend=mock for development.'
            ) from exc

    def _bootstrap_simulation(self) -> None:
        from autonomy_navrl.env.isaac.cfg import build_navrl_cfg
        from autonomy_navrl.env.isaac.direct_env import NavrlDirectEnv

        env_cfg = build_navrl_cfg(self._config)
        robot_kind = self._framework.robot.kind
        render_mode = self._resolve_render_mode()
        print(
            f'[INFO]: Loading {robot_kind} URDF + sensors '
            '(first run may take several minutes for USD/RTX warmup)...',
            flush=True,
        )
        self._env = NavrlDirectEnv(env_cfg, render_mode=render_mode)
        viz_cfg = getattr(env_cfg, '_viz_cfg', None)
        if viz_cfg is not None and viz_cfg.enabled:
            self._env.set_debug_vis(True)
            print(
                f'[INFO]: Pose visualization enabled: {", ".join(viz_cfg.modules)}',
                flush=True,
            )
        print('[INFO]: Isaac Lab sensors: TiledCamera (rgb + depth)' + (
            ' + Imu' if self._framework.sensors.imu_enabled else ''
        ), flush=True)
        print(f'[INFO]:   camera prim: {env_cfg.tiled_camera.prim_path}', flush=True)
        if env_cfg.imu is not None:
            print(f'[INFO]:   imu prim    : {env_cfg.imu.prim_path}', flush=True)
        print(f'[INFO]:   task        : {env_cfg.task_kind}', flush=True)
        print(f'[INFO]:   action      : {self._framework.action.model} (dim={self._action_dim})', flush=True)
        print(f'[INFO]:   num_envs    : {env_cfg.scene.num_envs}', flush=True)
        print('[INFO]: Isaac Lab env initialized.', flush=True)

    @staticmethod
    def _to_numpy_obs(obs_dict: dict) -> dict[str, np.ndarray]:
        return {
            'rgbd': obs_dict['rgbd'].detach().cpu().numpy().astype(np.float32),
            'state': obs_dict['state'].detach().cpu().numpy().astype(np.float32),
        }
