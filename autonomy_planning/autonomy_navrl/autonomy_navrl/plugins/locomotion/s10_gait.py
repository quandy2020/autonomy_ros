"""S10 kinematic gait: articulated legs + wheel drive.

Modes (auto-selected from velocity command, or forced via ``gait_mode``):
  stand   — hold default balance pose, wheels idle
  march   — in-place diagonal stepping (legs lift, wheels idle)
  walk    — forward/back + turn with visible leg lift
  strafe  — lateral crab-step with leg lift + side wheel mix

Walk/strafe share the same lift pattern; direction comes from (vx, vy).
"""

from __future__ import annotations

import math

import torch

from autonomy_navrl.plugins.locomotion.base import BaseLocomotionController, locomotion_registry
from autonomy_navrl.plugins.locomotion.s10_jit import S10_LEG_JOINT_NAMES, S10_WHEEL_JOINT_NAMES
from autonomy_navrl.plugins.locomotion.wheel_joints import resolve_joint_indices

# Per-leg joint slots inside the 12-dim leg target vector (FL, FR, HL, HR).
_HIPX = (0, 3, 6, 9)
_HIPY = (1, 4, 7, 10)
_KNEE = (2, 5, 8, 11)

# Diagonal pairs for trot: (FL, HR) vs (FR, HL).
_PAIR_A = (0, 3)  # FL, HR
_PAIR_B = (1, 2)  # FR, HL
_LEFT = (0, 2)    # FL, HL
_RIGHT = (1, 3)   # FR, HR


@locomotion_registry.register('s10_gait')
class S10GaitLocomotion(BaseLocomotionController):
    """Balance stance with active leg articulation and wheeled propulsion."""

    name = 's10_gait'

    def __init__(self, spec, env) -> None:
        super().__init__(spec, env)
        self._leg_joint_ids: list[int] | None = None
        self._wheel_joint_ids: list[int] | None = None
        self._default_leg: torch.Tensor | None = None
        self._phase = torch.zeros(env.num_envs, device=env.device)
        self._dt = float(env.cfg.sim.dt)

        self._wheel_gain = float(spec.joint_vel_action_scale or 5.0)
        self._yaw_arm = 0.70
        self._mode = str(getattr(spec, 'gait_mode', 'auto') or 'auto').lower()
        self._freq = float(getattr(spec, 'gait_step_freq_hz', 1.5) or 1.5)
        self._hipy_amp = float(getattr(spec, 'gait_hipy_amp', 0.14) or 0.14)
        self._knee_amp = float(getattr(spec, 'gait_knee_amp', 0.22) or 0.22)
        self._hipx_amp = float(getattr(spec, 'gait_hipx_amp', 0.10) or 0.10)
        self._march_idle = bool(getattr(spec, 'gait_march_when_idle', False))
        self._idle_eps = 0.04
        # Lock legs only for very slow parking creep (precision alignment).
        self._precision_speed = float(getattr(spec, 'gait_precision_speed_mps', 0.07) or 0.07)
        if bool(getattr(spec, 'hold_stance', False)):
            self._mode = 'stand'

    def _ensure_initialized(self) -> None:
        if self._leg_joint_ids is not None:
            return
        names = list(self.env._robot.data.joint_names)
        self._leg_joint_ids = resolve_joint_indices(names, S10_LEG_JOINT_NAMES)
        self._wheel_joint_ids = resolve_joint_indices(names, S10_WHEEL_JOINT_NAMES)
        self._default_leg = self.env._robot.data.default_joint_pos[:, self._leg_joint_ids].clone()

    def reset(self, env_ids: torch.Tensor) -> None:
        self._phase[env_ids] = 0.0
        if self._leg_joint_ids is not None:
            self._default_leg = self.env._robot.data.default_joint_pos[:, self._leg_joint_ids].clone()

    def _clip_cmd(self, vx, vy, w):
        if not self.spec.enable_clip:
            return vx, vy, w
        return (
            torch.clamp(vx, -self.spec.clip_vx, self.spec.clip_vx),
            torch.clamp(vy, -self.spec.clip_vy, self.spec.clip_vy),
            torch.clamp(w, -self.spec.clip_w, self.spec.clip_w),
        )

    def _resolve_mode(self, vx, vy, w) -> torch.Tensor:
        """Return per-env mode id: 0=stand, 1=march, 2=walk, 3=strafe."""
        speed = torch.sqrt(vx * vx + vy * vy)
        yaw_mag = w.abs()
        idle = (speed < self._idle_eps) & (yaw_mag < self._idle_eps)
        strafe = vy.abs() > (vx.abs() + 0.05)

        mode = torch.full((self.env.num_envs,), 2, device=self.env.device, dtype=torch.long)
        if self._mode == 'stand':
            mode[:] = 0
        elif self._mode == 'march':
            mode[:] = 1
        elif self._mode == 'strafe':
            mode[:] = 3
        elif self._mode == 'walk':
            mode[:] = 2
        else:
            precision = speed < self._precision_speed
            mode = torch.where(precision & idle, torch.zeros_like(mode), mode)
            mode = torch.where(idle, torch.zeros_like(mode), mode)
            if self._march_idle:
                mode = torch.where(idle, torch.ones_like(mode), mode)
            mode = torch.where(strafe & ~idle & ~precision, torch.full_like(mode, 3), mode)
        return mode

    def _swing_lift(self, leg: int, phase: torch.Tensor) -> torch.Tensor:
        """Per-leg swing lift: blend diagonal trot and lateral crab-step."""
        s = torch.sin(phase)
        # Diagonal trot lift for this leg.
        if leg in _PAIR_A:
            diag = torch.relu(s)
        else:
            diag = torch.relu(torch.sin(phase + math.pi))
        # Lateral pairs: left legs together vs right legs together.
        if leg in _LEFT:
            lat = torch.relu(s)
        else:
            lat = torch.relu(torch.sin(phase + math.pi))
        return torch.maximum(diag, lat)

    def _leg_targets(self, mode: torch.Tensor, vx, vy, w) -> torch.Tensor:
        assert self._default_leg is not None
        targets = self._default_leg.clone()
        phase = self._phase

        speed = torch.sqrt(vx * vx + vy * vy)
        eps = 1e-4
        long_w = vx.abs() / (speed + eps)
        lat_w = vy.abs() / (speed + eps)
        fwd = torch.clamp(vx / 0.35, -1.0, 1.0)
        lat = torch.clamp(vy / 0.30, -1.0, 1.0)

        activity = torch.clamp(speed / 0.35 + w.abs() / 1.2, 0.0, 1.5)
        march = mode == 1
        walk = mode == 2
        strafe = mode == 3
        active = march | walk | strafe
        # Keep visible lift once moving, but scale smoothly with speed.
        activity = torch.where(active, torch.clamp(activity, 0.45, 1.5), activity)
        live = active.float()

        hipy_amp = self._hipy_amp * torch.where(
            march, 0.85 * torch.ones_like(activity), activity,
        )
        knee_amp = self._knee_amp * torch.where(
            march, 0.85 * torch.ones_like(activity), activity,
        )
        hipx_amp = self._hipx_amp * activity

        for leg in range(4):
            lift = self._swing_lift(leg, phase) * live
            hipx_i = _HIPX[leg]
            hipy_i = _HIPY[leg]
            knee_i = _KNEE[leg]
            side = 1.0 if leg in _LEFT else -1.0  # left +, right -

            # Thigh pitch: clear lift on swing.
            d_hipy = hipy_amp * lift
            # Longitudinal reach (subtle — avoid yaw drift on straight-line walk).
            d_hipy = d_hipy + hipy_amp * 0.12 * lift * long_w * fwd

            # Knee flex: main visual for foot lift.
            d_knee = -knee_amp * lift

            # Hip roll: lateral reach on strafe / holonomic vy.
            lat_mix = torch.where(strafe | walk, lat_w, torch.zeros_like(lat_w))
            d_hipx = hipx_amp * lift * (
                lat * side * lat_mix
                + 0.45 * torch.sin(phase + (0.0 if leg in _LEFT else math.pi)) * side * (march | strafe).float()
            )

            targets[:, hipx_i] = targets[:, hipx_i] + d_hipx
            targets[:, hipy_i] = targets[:, hipy_i] + d_hipy
            targets[:, knee_i] = targets[:, knee_i] + d_knee

        return targets

    def _wheel_targets(self, mode: torch.Tensor, vx, vy, w) -> torch.Tensor:
        gain = self._wheel_gain
        arm = self._yaw_arm
        # Flip yaw mix: +w must produce CCW (see s10_stance_drive).
        yaw = -arm * w
        low_vx = (vx.abs() < 0.05).float()
        vx_eff = vx + 0.15 * torch.tanh(3.0 * w.abs()) * low_vx
        w0 = -(vx_eff + vy + yaw) * gain
        w1 = -(vx_eff - vy - yaw) * gain
        w2 = -(vx_eff - vy + yaw) * gain
        w3 = -(vx_eff + vy - yaw) * gain
        wheels = torch.stack([w0, w1, w2, w3], dim=-1)
        freeze = ((mode == 0) | (mode == 1)).float().unsqueeze(-1)
        return wheels * (1.0 - freeze)

    def apply(self) -> None:
        self._ensure_initialized()
        vx, vy, w = self._clip_cmd(*self.velocity_command())
        mode = self._resolve_mode(vx, vy, w)

        moving = (mode != 0).float()
        self._phase = (self._phase + 2.0 * math.pi * self._freq * self._dt * moving) % (2.0 * math.pi)

        leg_targets = self._leg_targets(mode, vx, vy, w)
        wheel_targets = self._wheel_targets(mode, vx, vy, w)

        robot = self.env._robot
        robot.set_joint_position_target(leg_targets, joint_ids=self._leg_joint_ids)
        robot.set_joint_velocity_target(wheel_targets, joint_ids=self._wheel_joint_ids)
