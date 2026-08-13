"""S10 TorchScript locomotion policy (velocity command -> joint targets).

Observation/action layout must match ``S10RoughEnvCfg`` / Isaac-Velocity-Rough-S10-v0:
  ang_vel*0.25(3) + gravity(3) + vel_cmd(3)
  + joint_pos_rel_no_wheel(16) + joint_vel*0.05(16) + last_action(16)
"""

from __future__ import annotations

from pathlib import Path

import torch

from autonomy_navrl.plugins.locomotion.base import BaseLocomotionController, locomotion_registry
from autonomy_navrl.plugins.locomotion.wheel_joints import resolve_joint_indices

# Joint order must match Isaac Lab S10 velocity env (preserve_order=True).
S10_LEG_JOINT_NAMES: tuple[str, ...] = (
    'fl_hipx_joint', 'fl_hipy_joint', 'fl_knee_joint',
    'fr_hipx_joint', 'fr_hipy_joint', 'fr_knee_joint',
    'hl_hipx_joint', 'hl_hipy_joint', 'hl_knee_joint',
    'hr_hipx_joint', 'hr_hipy_joint', 'hr_knee_joint',
)
S10_WHEEL_JOINT_NAMES: tuple[str, ...] = (
    'fl_wheel_joint', 'fr_wheel_joint', 'hl_wheel_joint', 'hr_wheel_joint',
)
S10_ALL_JOINT_NAMES: tuple[str, ...] = S10_LEG_JOINT_NAMES + S10_WHEEL_JOINT_NAMES

S10_JIT_OBS_DIM = 57
S10_ANG_VEL_SCALE = 0.25
S10_JOINT_VEL_SCALE = 0.05
S10_LEG_POS_SCALE = 0.25
S10_HIPX_POS_SCALE = 0.125
S10_WHEEL_VEL_SCALE = 5.0


def default_s10_policy_path() -> str:
    pkg_root = Path(__file__).resolve().parents[4]
    autonomy_root = pkg_root.parents[3]
    candidates = [
        pkg_root / 'weights' / 's10' / 'policy.pt',
        autonomy_root / 'checkpoints' / 's10_locomotion' / 'exported' / 'policy.pt',
        Path('/workspace/autonomy/checkpoints/s10_locomotion/exported/policy.pt'),
    ]
    for path in candidates:
        if path.is_file() and path.stat().st_size >= 10_000:
            return str(path)
    return str(candidates[0])


@locomotion_registry.register('s10_jit')
class S10JitLocomotion(BaseLocomotionController):
    """Hierarchical control: nav (vx,vy,w) -> JIT locomotion -> joint targets.

    Prefers Isaac Lab IMU (imu_link) for ang_vel / projected gravity when present,
    matching the training observation pipeline.
    """

    name = 's10_jit'

    def __init__(self, spec, env) -> None:
        super().__init__(spec, env)
        self._hold_stance = bool(getattr(spec, 'hold_stance', False))
        self._policy = None
        if not self._hold_stance:
            policy_path = spec.policy_path or default_s10_policy_path()
            if not Path(policy_path).is_file():
                raise FileNotFoundError(
                    f'S10 JIT policy not found: {policy_path}. '
                    'Train low-level policy first: bash scripts/navrl.sh train-s10-loco'
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
        self._wheel_vel_scale = float(spec.joint_vel_action_scale or S10_WHEEL_VEL_SCALE)
        self._leg_scale = float(spec.joint_pos_action_scale or S10_LEG_POS_SCALE)
        self._hipx_scale = float(getattr(spec, 'hip_joint_pos_scale', None) or S10_HIPX_POS_SCALE)
        self._leg_pos_scales: torch.Tensor | None = None

    def _ensure_initialized(self) -> None:
        if self._leg_joint_ids is not None:
            return
        robot_names = list(self.env._robot.data.joint_names)
        self._leg_joint_ids = resolve_joint_indices(robot_names, S10_LEG_JOINT_NAMES)
        self._wheel_joint_ids = resolve_joint_indices(robot_names, S10_WHEEL_JOINT_NAMES)
        self._all_joint_ids = resolve_joint_indices(robot_names, S10_ALL_JOINT_NAMES)
        scales = torch.full((len(S10_LEG_JOINT_NAMES),), self._leg_scale, device=self.env.device)
        for index, name in enumerate(S10_LEG_JOINT_NAMES):
            if name.endswith('_hipx_joint'):
                scales[index] = self._hipx_scale
        self._leg_pos_scales = scales
        self._init_cached_targets()

    def _init_cached_targets(self) -> None:
        robot = self.env._robot
        default_pos = robot.data.default_joint_pos
        self._cached_leg_targets = default_pos[:, self._leg_joint_ids].clone()
        self._cached_wheel_vel = torch.zeros(
            self.env.num_envs, len(self._wheel_joint_ids), device=self.env.device,
        )

    def _apply_cached_targets(self) -> None:
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
        # Hold zero velocity command while still running JIT (Go2W pattern) so the
        # stance controller settles before tracking starts.
        self._warmup_remaining = max(int(self.spec.decimation) * 8, 8)
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

    def _imu_ang_vel_and_gravity(self) -> tuple[torch.Tensor, torch.Tensor]:
        """Prefer scene IMU (training layout); fall back to articulation root."""
        imu = getattr(self.env, '_imu', None)
        if imu is not None:
            return imu.data.ang_vel_b, imu.data.projected_gravity_b
        robot = self.env._robot
        return robot.data.root_ang_vel_b, robot.data.projected_gravity_b

    def _build_observation(
        self,
        vx: torch.Tensor,
        vy: torch.Tensor,
        w: torch.Tensor,
    ) -> torch.Tensor:
        self._ensure_initialized()
        robot = self.env._robot
        vx, vy, w = self._clip_velocity_cmd(vx, vy, w)

        ang_vel_raw, gravity = self._imu_ang_vel_and_gravity()
        ang_vel = ang_vel_raw * S10_ANG_VEL_SCALE
        vel_cmd = torch.stack([vx, vy, w], dim=-1)

        default_pos = robot.data.default_joint_pos
        joint_pos_rel = (
            robot.data.joint_pos[:, self._all_joint_ids]
            - default_pos[:, self._all_joint_ids]
        )
        # Continuous wheels: zero relative position slots (matches training mdp).
        joint_pos_rel[:, len(S10_LEG_JOINT_NAMES):] = 0.0
        joint_vel = robot.data.joint_vel[:, self._all_joint_ids] * S10_JOINT_VEL_SCALE

        obs = torch.cat([
            ang_vel,
            gravity,
            vel_cmd,
            joint_pos_rel,
            joint_vel,
            self._last_actions,
        ], dim=-1)
        if obs.shape[-1] != S10_JIT_OBS_DIM:
            raise RuntimeError(
                f'S10 JIT observation dim {obs.shape[-1]}, expected {S10_JIT_OBS_DIM}'
            )
        return obs

    def apply(self) -> None:
        self._ensure_initialized()
        if self._hold_stance:
            # Diagnostic: PD-hold default standing pose, ignore JIT.
            self._init_cached_targets()
            self._apply_cached_targets()
            self._counter += 1
            return
        decimation = max(self.spec.decimation, 1)
        if self._counter % decimation == 0:
            vx, vy, w = self.velocity_command()
            if self._warmup_remaining > 0:
                vx = torch.zeros_like(vx)
                vy = torch.zeros_like(vy)
                w = torch.zeros_like(w)
                self._warmup_remaining -= 1
            obs = self._build_observation(vx, vy, w)

            with torch.inference_mode():
                actions = self._policy(obs)
            if actions.shape[-1] != 16:
                raise RuntimeError(f'S10 JIT policy output dim {actions.shape[-1]}, expected 16')

            self._last_actions = actions.detach().clone()
            pos_actions = actions[:, :12]
            vel_actions = actions[:, 12:16]

            default_pos = self.env._robot.data.default_joint_pos
            assert self._leg_pos_scales is not None
            self._cached_leg_targets = (
                default_pos[:, self._leg_joint_ids] + pos_actions * self._leg_pos_scales
            )
            self._cached_wheel_vel = vel_actions * self._wheel_vel_scale

        self._apply_cached_targets()
        self._counter += 1
