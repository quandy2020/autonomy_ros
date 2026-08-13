"""Visualization helpers (lazy imports — avoid Isaac Sim at import time)."""

from __future__ import annotations

from typing import TYPE_CHECKING

__all__ = [
    'PoseVizConfig',
    'build_goal_and_velocity_markers',
    'build_pose_viz_manager',
]

if TYPE_CHECKING:
    from autonomy_navrl.viz.isaac.config import PoseVizConfig
    from autonomy_navrl.viz.isaac.manager import build_pose_viz_manager
    from autonomy_navrl.viz.rviz_markers import build_goal_and_velocity_markers


def __getattr__(name: str):
    if name == 'PoseVizConfig':
        from autonomy_navrl.viz.isaac.config import PoseVizConfig

        return PoseVizConfig
    if name == 'build_pose_viz_manager':
        from autonomy_navrl.viz.isaac.manager import build_pose_viz_manager

        return build_pose_viz_manager
    if name == 'build_goal_and_velocity_markers':
        from autonomy_navrl.viz.rviz_markers import build_goal_and_velocity_markers

        return build_goal_and_velocity_markers
    raise AttributeError(f'module {__name__!r} has no attribute {name!r}')
