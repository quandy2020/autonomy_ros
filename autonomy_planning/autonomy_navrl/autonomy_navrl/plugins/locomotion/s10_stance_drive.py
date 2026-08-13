"""S10 baseline locomotion: hold default stance and drive wheels."""

from __future__ import annotations

import torch

from autonomy_navrl.plugins.locomotion.base import BaseLocomotionController, locomotion_registry
from autonomy_navrl.plugins.locomotion.s10_jit import S10_LEG_JOINT_NAMES, S10_WHEEL_JOINT_NAMES
from autonomy_navrl.plugins.locomotion.wheel_joints import resolve_joint_indices

_HIPX = (0, 3, 6, 9)


@locomotion_registry.register('s10_stance_drive')
class S10StanceDriveLocomotion(BaseLocomotionController):
    """Keep S10 in a balanced standing pose while commanding wheel speeds.

    S10 regular wheels scrub poorly when legs are locked. Yaw uses:
      1. Strong left/right differential (sign matched to +w = CCW).
      2. Small forward roll so tires roll instead of scrub.
      3. Mild hipx lean into the turn.
    """

    name = 's10_stance_drive'

    def __init__(self, spec, env) -> None:
        super().__init__(spec, env)
        self._leg_joint_ids: list[int] | None = None
        self._wheel_joint_ids: list[int] | None = None
        self._cached_leg_targets: torch.Tensor | None = None
        self._default_leg: torch.Tensor | None = None
        self._wheel_gain = float(spec.joint_vel_action_scale or 5.0)
        self._yaw_arm = 1.20
        self._yaw_assist_vx = 0.22
        self._hipx_steer = 0.18
        self._yaw_gain_boost = 1.8

    def _ensure_initialized(self) -> None:
        if self._leg_joint_ids is not None:
            return
        joint_names = list(self.env._robot.data.joint_names)
        self._leg_joint_ids = resolve_joint_indices(joint_names, S10_LEG_JOINT_NAMES)
        self._wheel_joint_ids = resolve_joint_indices(joint_names, S10_WHEEL_JOINT_NAMES)
        self._init_cached_targets()

    def _init_cached_targets(self) -> None:
        robot = self.env._robot
        default_pos = robot.data.default_joint_pos
        self._default_leg = default_pos[:, self._leg_joint_ids].clone()
        self._cached_leg_targets = self._default_leg.clone()

    def reset(self, env_ids: torch.Tensor) -> None:
        del env_ids
        if self._leg_joint_ids is not None:
            self._init_cached_targets()

    def apply(self) -> None:
        self._ensure_initialized()
        assert self._default_leg is not None
        vx, vy, w = self.velocity_command()
        arm = self._yaw_arm

        # Measured: previous (+arm*w on FL) produced CW for +w. Flip yaw mix
        # so +w → CCW (positive robot yaw), matching bearing / yaw_error signs.
        yaw = -arm * w
        # Boost differential magnitude while yawing so scrubbing can overcome stiction.
        gain = self._wheel_gain * (1.0 + (self._yaw_gain_boost - 1.0) * torch.tanh(2.5 * w.abs()))

        # Tiny forward roll while turning reduces scrubbing stiction.
        low_vx = (vx.abs() < 0.08).float()
        vx_eff = vx + self._yaw_assist_vx * torch.tanh(3.0 * w.abs()) * low_vx

        # Wheel order FL, FR, HL, HR. Negate: URDF axis opposite +vx.
        w0 = -(vx_eff + vy + yaw) * gain
        w1 = -(vx_eff - vy - yaw) * gain
        w2 = -(vx_eff - vy + yaw) * gain
        w3 = -(vx_eff + vy - yaw) * gain
        wheel_vel = torch.stack([w0, w1, w2, w3], dim=-1)

        legs = self._default_leg.clone()
        # Lean into CCW turn: +w → left hipx +, right hipx -.
        steer = self._hipx_steer * torch.clamp(w, -1.0, 1.0)
        legs[:, _HIPX[0]] = legs[:, _HIPX[0]] + steer
        legs[:, _HIPX[2]] = legs[:, _HIPX[2]] + steer
        legs[:, _HIPX[1]] = legs[:, _HIPX[1]] - steer
        legs[:, _HIPX[3]] = legs[:, _HIPX[3]] - steer
        self._cached_leg_targets = legs

        robot = self.env._robot
        robot.set_joint_position_target(self._cached_leg_targets, joint_ids=self._leg_joint_ids)
        robot.set_joint_velocity_target(wheel_vel, joint_ids=self._wheel_joint_ids)
