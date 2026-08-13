"""Navigation task plugins (goal sampling, success, extra state)."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Any

import torch

from autonomy_navrl.core.registry import Registry
from autonomy_navrl.core.spec import TaskSpec
from autonomy_navrl.core.geometry import body_goal_features
from autonomy_navrl.core.sampling import sample_goal_positions

if TYPE_CHECKING:
    from autonomy_navrl.env.isaac.direct_env import NavrlDirectEnv

task_registry: Registry[type['BaseTask']] = Registry('task')


class BaseTask(ABC):
    """Task-specific goal reset, success criteria, and state features."""

    name: str

    def __init__(self, spec: TaskSpec, env: NavrlDirectEnv) -> None:
        self.spec = spec
        self.env = env
        self.params = spec.params

    @abstractmethod
    def reset_goals(self, env_ids: torch.Tensor) -> None:
        """Sample or assign goal pose for resetting envs."""

    @abstractmethod
    def goal_state_features(
        self,
        pos_xy: torch.Tensor,
        yaw: torch.Tensor,
    ) -> torch.Tensor:
        """Extra goal-relative features appended to state (before IMU/odom)."""

    def modify_reward(
        self,
        reward: torch.Tensor,
        *,
        distance: torch.Tensor,
        yaw_error: torch.Tensor,
    ) -> torch.Tensor:
        return reward

    def on_step(self, pos_xy: torch.Tensor, yaw: torch.Tensor) -> None:
        """Optional per-step hook (IoU update, waypoint advance, etc.)."""
        del pos_xy, yaw

    def on_reset_envs(self, env_ids: torch.Tensor) -> None:
        """Optional hook after goal reset."""
        del env_ids

    def reward_metrics_extra(self) -> dict[str, float]:
        """Optional extra scalars merged into training metrics."""
        return {}

    @abstractmethod
    def is_success(self, distance: torch.Tensor, yaw_error: torch.Tensor) -> torch.Tensor:
        """Per-env success mask."""


@task_registry.register('goal_nav')
class GoalNavTask(BaseTask):
    name = 'goal_nav'

    def reset_goals(self, env_ids: torch.Tensor) -> None:
        env = self.env
        cpu_ids = env_ids.detach().cpu().numpy()
        goal_radius = env._goal_curriculum.goal_radius(int(env.common_step_counter))
        new_goals = sample_goal_positions(
            cpu_ids.shape[0],
            env.cfg.arena_size_m,
            goal_radius,
            env._numpy_rng(),
        )
        env._goals[env_ids, 0] = torch.as_tensor(new_goals[:, 0], device=env.device)
        env._goals[env_ids, 1] = torch.as_tensor(new_goals[:, 1], device=env.device)
        env._goal_yaws[env_ids] = 0.0

    def goal_state_features(self, pos_xy: torch.Tensor, yaw: torch.Tensor) -> torch.Tensor:
        env = self.env
        goal_delta = env._goals - pos_xy
        distance = torch.linalg.norm(goal_delta, dim=-1, keepdim=True)
        bearing = torch.atan2(goal_delta[:, 1], goal_delta[:, 0]) - yaw
        bearing = torch.atan2(torch.sin(bearing), torch.cos(bearing)).unsqueeze(-1)
        cos_yaw = torch.cos(yaw)
        sin_yaw = torch.sin(yaw)
        dx_body = cos_yaw * goal_delta[:, 0:1] + sin_yaw * goal_delta[:, 1:2]
        dy_body = -sin_yaw * goal_delta[:, 0:1] + cos_yaw * goal_delta[:, 1:2]
        return torch.cat([dx_body, dy_body, distance, bearing], dim=-1)

    def is_success(self, distance: torch.Tensor, yaw_error: torch.Tensor) -> torch.Tensor:
        del yaw_error
        return distance < self.env.cfg.goal_tolerance_m


@task_registry.register('precision_pose')
class PrecisionPoseTask(BaseTask):
    """Generic precision alignment (position + yaw, no robot-specific IoU)."""

    name = 'precision_pose'

    def _body_goal_features(
        self, pos_xy: torch.Tensor, yaw: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        from autonomy_navrl.core.geometry import body_goal_features

        env = self.env
        return body_goal_features(
            env._goals, env._goal_yaws, pos_xy, yaw, env._yaw_error,
        )

    def reset_goals(self, env_ids: torch.Tensor) -> None:
        env = self.env
        cpu_ids = env_ids.detach().cpu().numpy()
        count = cpu_ids.shape[0]
        pose_range = self.params.get('pose_range')
        if pose_range:
            rng = env._numpy_rng()
            x_range = pose_range.get('x', (0.0, 0.0))
            y_range = pose_range.get('y', (0.0, 0.0))
            yaw_range = pose_range.get('yaw', (0.0, 0.0))
            xs = rng.uniform(float(x_range[0]), float(x_range[1]), count)
            ys = rng.uniform(float(y_range[0]), float(y_range[1]), count)
            yaws = rng.uniform(float(yaw_range[0]), float(yaw_range[1]), count)
            env._goals[env_ids, 0] = torch.as_tensor(xs, device=env.device, dtype=torch.float32)
            env._goals[env_ids, 1] = torch.as_tensor(ys, device=env.device, dtype=torch.float32)
            env._goal_yaws[env_ids] = torch.as_tensor(yaws, device=env.device, dtype=torch.float32)
        else:
            target = self.params.get('target_pose', [0.0, 0.0, 0.0])
            env._goals[env_ids, 0] = float(target[0])
            env._goals[env_ids, 1] = float(target[1])
            env._goal_yaws[env_ids] = float(target[2])

    def goal_state_features(self, pos_xy: torch.Tensor, yaw: torch.Tensor) -> torch.Tensor:
        dx_body, dy_body, distance, bearing, yaw_err = self._body_goal_features(pos_xy, yaw)
        return torch.cat([
            dx_body.unsqueeze(-1),
            dy_body.unsqueeze(-1),
            distance.unsqueeze(-1),
            bearing.unsqueeze(-1),
            yaw_err.unsqueeze(-1),
        ], dim=-1)

    def modify_reward(
        self,
        reward: torch.Tensor,
        *,
        distance: torch.Tensor,
        yaw_error: torch.Tensor,
    ) -> torch.Tensor:
        env = self.env
        if env._reward_profile != 'jdrobot':
            return reward
        near_goal = (distance < env.cfg.goal_tolerance_m * 2.0).float()
        return reward + env.cfg.orientation_reward_scale * torch.square(yaw_error) * near_goal

    def is_success(self, distance: torch.Tensor, yaw_error: torch.Tensor) -> torch.Tensor:
        env = self.env
        reached_pos = distance < env.cfg.goal_tolerance_m
        yaw_ok = yaw_error.abs() < env.cfg.yaw_tolerance_rad
        reached = reached_pos & yaw_ok
        stop_speed = float(getattr(env.cfg, 'require_stop_speed_mps', 0.0))
        if stop_speed > 0.0:
            lin_speed = torch.linalg.norm(env._robot.data.root_lin_vel_b[:, :2], dim=-1)
            reached = reached & (lin_speed <= stop_speed)
        return reached


@task_registry.register('waypoint_nav')
class WaypointNavTask(BaseTask):
    """Visit a list of waypoints sequentially (x, y, yaw optional per point)."""

    name = 'waypoint_nav'

    def __init__(self, spec: TaskSpec, env: NavrlDirectEnv) -> None:
        super().__init__(spec, env)
        raw_wps = self.params.get('waypoints') or [[8.0, 0.0, 0.0]]
        self._waypoints = [
            (float(wp[0]), float(wp[1]), float(wp[2]) if len(wp) > 2 else 0.0)
            for wp in raw_wps
        ]
        self._wp_index = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)

    def _apply_waypoint(self, env_ids: torch.Tensor, indices: torch.Tensor) -> None:
        env = self.env
        for slot, env_id in enumerate(env_ids):
            idx = int(indices[slot].item()) % len(self._waypoints)
            x, y, yaw = self._waypoints[idx]
            env._goals[env_id, 0] = x
            env._goals[env_id, 1] = y
            env._goal_yaws[env_id] = yaw

    def reset_goals(self, env_ids: torch.Tensor) -> None:
        self._wp_index[env_ids] = 0
        self._apply_waypoint(env_ids, self._wp_index[env_ids])

    def goal_state_features(self, pos_xy: torch.Tensor, yaw: torch.Tensor) -> torch.Tensor:
        env = self.env
        goal_delta = env._goals - pos_xy
        distance = torch.linalg.norm(goal_delta, dim=-1, keepdim=True)
        bearing = torch.atan2(goal_delta[:, 1], goal_delta[:, 0]) - yaw
        bearing = torch.atan2(torch.sin(bearing), torch.cos(bearing)).unsqueeze(-1)
        cos_yaw = torch.cos(yaw)
        sin_yaw = torch.sin(yaw)
        dx_body = cos_yaw * goal_delta[:, 0:1] + sin_yaw * goal_delta[:, 1:2]
        dy_body = -sin_yaw * goal_delta[:, 0:1] + cos_yaw * goal_delta[:, 1:2]
        wp_progress = (self._wp_index.float() / max(len(self._waypoints), 1)).unsqueeze(-1)
        return torch.cat([dx_body, dy_body, distance, bearing, wp_progress], dim=-1)

    def is_success(self, distance: torch.Tensor, yaw_error: torch.Tensor) -> torch.Tensor:
        del yaw_error
        env = self.env
        at_goal = distance < env.cfg.goal_tolerance_m
        last = len(self._waypoints) - 1
        advance = at_goal & (self._wp_index < last)
        if advance.any():
            ids = torch.nonzero(advance, as_tuple=False).squeeze(-1)
            if ids.ndim == 0:
                ids = ids.unsqueeze(0)
            self._wp_index[ids] += 1
            self._apply_waypoint(ids, self._wp_index[ids])
        return at_goal & (self._wp_index >= last)


def create_task(spec: TaskSpec, env: NavrlDirectEnv) -> BaseTask:
    return task_registry.create(spec.kind, spec=spec, env=env)
