"""Kinematic root velocity — default low-level for diff/quadruped placeholders."""

from __future__ import annotations

import torch

from autonomy_navrl.control.commands import body_velocity_to_world
from autonomy_navrl.plugins.locomotion.base import BaseLocomotionController, locomotion_registry


@locomotion_registry.register('root_velocity')
class RootVelocityLocomotion(BaseLocomotionController):
    """Write body-frame (vx, vy, w) to root COM velocity (legacy behavior).

    For wheeled-legged URDFs (Go2W), also drive wheel joint velocities. Root-only
    velocity writes do not move a grounded articulated robot with locked feet.
    """

    name = 'root_velocity'

    def __init__(self, spec, env) -> None:
        super().__init__(spec, env)
        self._wheel_joint_ids: list[int] | None = None
        self._wheeled_legged = False
        framework = getattr(env.cfg, '_framework', None)
        if framework is not None and framework.robot.kind == 'wheeled_legged':
            self._wheeled_legged = True
        self._wheel_gain = 8.0
        self._yaw_arm = 0.25

    def _ensure_wheel_joints(self) -> None:
        if self._wheel_joint_ids is not None or not self._wheeled_legged:
            return
        from autonomy_navrl.plugins.locomotion.wheel_joints import resolve_wheel_joint_indices

        self._wheel_joint_ids = resolve_wheel_joint_indices(
            list(self.env._robot.data.joint_names),
        )

    def reset(self, env_ids: torch.Tensor) -> None:
        del env_ids

    def apply(self) -> None:
        vx, vy, w = self.velocity_command()
        yaw = self.env._robot_yaw()
        root_velocity = body_velocity_to_world(vx, vy, w, yaw)
        robot = self.env._robot
        robot.write_root_com_velocity_to_sim(root_velocity)

        self._ensure_wheel_joints()
        if not self._wheel_joint_ids:
            return
        gain = self._wheel_gain
        arm = self._yaw_arm
        n_wheels = len(self._wheel_joint_ids)
        if n_wheels == 4:
            # Go2W: FR, FL, RR, RL — S10: FL, FR, HL, HR (same holonomic mix layout).
            w0 = (vx + vy + arm * w) * gain
            w1 = (vx - vy - arm * w) * gain
            w2 = (vx - vy + arm * w) * gain
            w3 = (vx + vy - arm * w) * gain
            targets = torch.stack([w0, w1, w2, w3], dim=-1)
        else:
            targets = torch.full(
                (self.env.num_envs, n_wheels), vx * gain, device=self.env.device,
            )
        robot.set_joint_velocity_target(targets, joint_ids=self._wheel_joint_ids)
