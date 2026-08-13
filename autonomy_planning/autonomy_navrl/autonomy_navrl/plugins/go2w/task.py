"""Go2W precision-pose task with optional bbox IoU consecutive success."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from autonomy_navrl.core.geometry import body_goal_features
from autonomy_navrl.plugins.go2w.success_tracker import ConsecutiveIoUConfig, ConsecutiveIoUTracker
from autonomy_navrl.plugins.tasks import PrecisionPoseTask, task_registry

if TYPE_CHECKING:
    from autonomy_navrl.core.spec import TaskSpec
    from autonomy_navrl.env.isaac.direct_env import NavrlDirectEnv


@task_registry.register('go2w_precision_pose')
class Go2WPrecisionPoseTask(PrecisionPoseTask):
    """Go2W precision alignment with Isaac Lab-style consecutive bbox IoU success."""

    name = 'go2w_precision_pose'

    def __init__(self, spec: TaskSpec, env: NavrlDirectEnv) -> None:
        super().__init__(spec, env)
        self._iou_cfg = ConsecutiveIoUConfig.from_dict(self.params.get('consecutive_iou'))
        self._iou_tracker = ConsecutiveIoUTracker(
            self._iou_cfg, env.num_envs, env.device,
        )

    def _body_goal_features(
        self, pos_xy: torch.Tensor, yaw: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        env = self.env
        return body_goal_features(
            env._goals, env._goal_yaws, pos_xy, yaw, env._yaw_error,
        )

    def on_step(self, pos_xy: torch.Tensor, yaw: torch.Tensor) -> None:
        if not self._iou_cfg.enabled:
            return
        dx_body, dy_body, _, _, yaw_err = self._body_goal_features(pos_xy, yaw)
        self._iou_tracker.update(dx_body, dy_body, yaw_err)

    def on_reset_envs(self, env_ids: torch.Tensor) -> None:
        if self._iou_cfg.enabled:
            self._iou_tracker.reset(env_ids)

    def reward_metrics_extra(self) -> dict[str, float]:
        if not self._iou_cfg.enabled:
            return {}
        tracker = self._iou_tracker
        return {
            'iou_mean': float(tracker.current_iou.mean().detach().cpu()),
            'consecutive_iou': float(tracker.consecutive_success.mean().detach().cpu()),
        }

    @property
    def iou_tracker(self) -> ConsecutiveIoUTracker:
        return self._iou_tracker

    def is_success(self, distance: torch.Tensor, yaw_error: torch.Tensor) -> torch.Tensor:
        if self._iou_cfg.enabled:
            return self._iou_tracker.consecutive_success >= float(self._iou_cfg.num_success)
        return super().is_success(distance, yaw_error)
