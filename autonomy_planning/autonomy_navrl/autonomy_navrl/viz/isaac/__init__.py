"""Isaac Sim viewport visualization (lazy imports)."""

from __future__ import annotations

from typing import TYPE_CHECKING

from autonomy_navrl.viz.isaac.config import PoseVizConfig

__all__ = ['PoseVizConfig', 'PoseVizManager', 'build_pose_viz_manager']

if TYPE_CHECKING:
    from autonomy_navrl.viz.isaac.manager import PoseVizManager, build_pose_viz_manager


def __getattr__(name: str):
    if name in ('PoseVizManager', 'build_pose_viz_manager'):
        from autonomy_navrl.viz.isaac.manager import PoseVizManager, build_pose_viz_manager

        return {'PoseVizManager': PoseVizManager, 'build_pose_viz_manager': build_pose_viz_manager}[name]
    raise AttributeError(f'module {__name__!r} has no attribute {name!r}')
