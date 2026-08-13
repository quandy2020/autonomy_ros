"""Go2W TorchScript locomotion policy (Isaac Lab unitree_go2w compatible)."""

from __future__ import annotations

from pathlib import Path

import torch

from autonomy_navrl.plugins.locomotion.base import BaseLocomotionController, locomotion_registry

# Joint order must match Isaac Lab low_level_go2w_rough_env_cfg (preserve_order=True).
GO2W_LEG_JOINT_NAMES: tuple[str, ...] = (
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
)
GO2W_WHEEL_JOINT_NAMES: tuple[str, ...] = (
    'FR_foot_joint', 'FL_foot_joint', 'RR_foot_joint', 'RL_foot_joint',
)
GO2W_ALL_JOINT_NAMES: tuple[str, ...] = GO2W_LEG_JOINT_NAMES + GO2W_WHEEL_JOINT_NAMES

# Isaac Lab UnitreeGo2WRoughEnvCfg policy obs (base_lin_vel + height_scan disabled):
# ang_vel(3)*0.25 + gravity(3) + vel_cmd(3) + joint_pos(16) + joint_vel(16)*0.05 + last_action(16)
GO2W_JIT_OBS_DIM = 57

# Isaac Lab action scales (UnitreeGo2WRoughEnvCfg).
GO2W_HIP_POS_SCALE = 0.125
GO2W_LEG_POS_SCALE = 0.25
GO2W_WHEEL_VEL_SCALE = 5.0


def default_go2w_policy_path() -> str:
    pkg_root = Path(__file__).resolve().parents[4]
    autonomy_root = pkg_root.parents[3]
    candidates = [
        pkg_root / 'weights' / 'go2w' / 'policy.pt',
        autonomy_root / 'IsaacLab' / 'source' / 'isaaclab_tasks' / 'isaaclab_tasks'
        / 'manager_based' / 'navigation' / 'config' / 'unitree_go2w'
        / 'assets' / 'go2w_description' / 'policy.pt',
        Path('/workspace/autonomy/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based'
             '/navigation/config/unitree_go2w/assets/go2w_description/policy.pt'),
        Path('/workspace/isaaclab/source/isaaclab_tasks/isaaclab_tasks/manager_based'
             '/navigation/config/unitree_go2w/assets/go2w_description/policy.pt'),
        Path('/workspace/isaaclab/source/isaaclab_tasks/isaaclab_tasks/manager_based'
             '/navigation/config/unitree_go2w/assets/go2w_description/unitree_go2w_flat.pt'),
    ]
    for path in candidates:
        if path.is_file() and path.stat().st_size >= 10_000:
            return str(path)
    return str(candidates[0])


def _resolve_joint_indices(joint_names: list[str], expected: tuple[str, ...]) -> list[int]:
    name_to_index = {name: index for index, name in enumerate(joint_names)}
    missing = [name for name in expected if name not in name_to_index]
    if missing:
        raise RuntimeError(
            f'Go2W JIT missing joints {missing}. Available: {joint_names}'
        )
    return [name_to_index[name] for name in expected]


def _leg_pos_scales(
    device: torch.device,
    *,
    hip_scale: float = GO2W_HIP_POS_SCALE,
    leg_scale: float = GO2W_LEG_POS_SCALE,
) -> torch.Tensor:
    scales = torch.full((len(GO2W_LEG_JOINT_NAMES),), leg_scale, device=device)
    for index, name in enumerate(GO2W_LEG_JOINT_NAMES):
        if name.endswith('_hip_joint'):
            scales[index] = hip_scale
    return scales


@locomotion_registry.register('go2w_jit')
class Go2WJitLocomotion(BaseLocomotionController):
    """Hierarchical control: nav (vx,vy,w) -> JIT locomotion -> joint targets."""

    name = 'go2w_jit'

    def __init__(self, spec, env) -> None:
        super().__init__(spec, env)
        policy_path = spec.policy_path or default_go2w_policy_path()
        if not Path(policy_path).is_file():
            raise FileNotFoundError(
                f'Go2W JIT policy not found: {policy_path}. '
                'Run: bash scripts/navrl.sh setup go2w policy'
            )
        self._policy = torch.jit.load(policy_path, map_location=env.device).eval()
        self._counter = 0
        self._last_actions = torch.zeros(env.num_envs, 16, device=env.device)
        self._cached_leg_targets: torch.Tensor | None = None
        self._cached_wheel_vel: torch.Tensor | None = None
        self._warmup_remaining = 0
        self._leg_joint_ids: list[int] | None = None
        self._wheel_joint_ids: list[int] | None = None
        self._all_joint_ids: list[int] | None = None
        self._leg_pos_scales: torch.Tensor | None = None
        self._wheel_vel_scale = float(spec.joint_vel_action_scale)
        self._hip_scale = float(spec.hip_joint_pos_scale)
        self._leg_scale = float(spec.joint_pos_action_scale)

    def _ensure_initialized(self) -> None:
        """Resolve joint indices after sim play (Articulation.data is unavailable in _setup_scene)."""
        if self._leg_joint_ids is not None:
            return
        robot_names = list(self.env._robot.data.joint_names)
        self._leg_joint_ids = _resolve_joint_indices(robot_names, GO2W_LEG_JOINT_NAMES)
        self._wheel_joint_ids = _resolve_joint_indices(robot_names, GO2W_WHEEL_JOINT_NAMES)
        self._all_joint_ids = _resolve_joint_indices(robot_names, GO2W_ALL_JOINT_NAMES)
        self._leg_pos_scales = _leg_pos_scales(
            self.env.device,
            hip_scale=self._hip_scale,
            leg_scale=self._leg_scale,
        )
        self._init_cached_targets()

    def _init_cached_targets(self) -> None:
        """Seed PD targets to default standing pose (applied every physics sub-step)."""
        robot = self.env._robot
        default_pos = robot.data.default_joint_pos
        self._cached_leg_targets = default_pos[:, self._leg_joint_ids].clone()
        self._cached_wheel_vel = torch.zeros(
            self.env.num_envs, len(self._wheel_joint_ids), device=self.env.device,
        )

    def _apply_cached_targets(self) -> None:
        """Push cached joint targets — must run every physics sub-step like Isaac Lab."""
        assert self._cached_leg_targets is not None and self._cached_wheel_vel is not None
        robot = self.env._robot
        robot.set_joint_position_target(
            self._cached_leg_targets, joint_ids=self._leg_joint_ids,
        )
        robot.set_joint_velocity_target(
            self._cached_wheel_vel, joint_ids=self._wheel_joint_ids,
        )

    def reset(self, env_ids: torch.Tensor) -> None:
        self._last_actions[env_ids] = 0.0
        self._counter = 0
        # Hold zero velocity command briefly so the JIT policy stabilizes stance after reset.
        self._warmup_remaining = max(int(self.spec.decimation) * 4, 4)
        if self._leg_joint_ids is not None:
            self._init_cached_targets()

    def _clip_velocity_cmd(self, vx: torch.Tensor, vy: torch.Tensor, w: torch.Tensor):
        if not self.spec.enable_clip:
            return vx, vy, w
        return (
            torch.clamp(vx, -self.spec.clip_vx, self.spec.clip_vx),
            torch.clamp(vy, -self.spec.clip_vy, self.spec.clip_vy),
            torch.clamp(w, -self.spec.clip_w, self.spec.clip_w),
        )

    def _build_observation(
        self,
        vx: torch.Tensor,
        vy: torch.Tensor,
        w: torch.Tensor,
    ) -> torch.Tensor:
        """Build low-level policy observation (57-dim, Isaac Lab Go2W layout).

        Order matches ``UnitreeGo2WRoughEnvCfg`` policy group:
        ang_vel(3)*0.25 | gravity(3) | vel_cmd(3) | joint_pos(16) | joint_vel(16)*0.05 | last_action(16)

        ``joint_pos`` uses ``joint_pos_rel_without_wheel``: all 16 joints in order, wheel slots zeroed.
        """
        self._ensure_initialized()
        robot = self.env._robot
        vx, vy, w = self._clip_velocity_cmd(vx, vy, w)

        ang_vel = robot.data.root_ang_vel_b * 0.25
        gravity = robot.data.projected_gravity_b
        # JIT policy expects physical velocity commands (m/s, rad/s) — same as Isaac Lab
        # processed_actions fed to velocity_commands obs. Do NOT multiply by spec.scale again.
        vel_cmd = torch.stack([vx, vy, w], dim=-1)

        default_pos = robot.data.default_joint_pos
        joint_pos_rel = (
            robot.data.joint_pos[:, self._all_joint_ids]
            - default_pos[:, self._all_joint_ids]
        )
        # Wheel joints stay at zero in policy obs (continuous wheel coords are not used).
        joint_pos_rel[:, len(GO2W_LEG_JOINT_NAMES):] = 0.0
        joint_vel = robot.data.joint_vel[:, self._all_joint_ids] * 0.05

        obs = torch.cat([
            ang_vel,
            gravity,
            vel_cmd,
            joint_pos_rel,
            joint_vel,
            self._last_actions,
        ], dim=-1)
        if obs.shape[-1] != GO2W_JIT_OBS_DIM:
            raise RuntimeError(
                f'Go2W JIT observation dim {obs.shape[-1]}, expected {GO2W_JIT_OBS_DIM}'
            )
        return obs

    def apply(self) -> None:
        self._ensure_initialized()
        decimation = max(self.spec.decimation, 1)
        if self._counter % decimation == 0:
            vx, vy, w = self.velocity_command()
            if self._warmup_remaining > 0:
                vx = torch.zeros_like(vx)
                vy = torch.zeros_like(vy)
                w = torch.zeros_like(w)
                self._warmup_remaining -= 1
            vx, vy, w = self._clip_velocity_cmd(vx, vy, w)
            obs = self._build_observation(vx, vy, w)

            with torch.inference_mode():
                actions = self._policy(obs)
            if actions.shape[-1] != 16:
                raise RuntimeError(f'Go2W JIT policy output dim {actions.shape[-1]}, expected 16')

            self._last_actions = actions.detach().clone()
            pos_actions = actions[:, :12]
            vel_actions = actions[:, 12:16]

            default_pos = self.env._robot.data.default_joint_pos
            self._cached_leg_targets = (
                default_pos[:, self._leg_joint_ids] + pos_actions * self._leg_pos_scales
            )
            self._cached_wheel_vel = vel_actions * self._wheel_vel_scale

        self._apply_cached_targets()
        self._counter += 1
