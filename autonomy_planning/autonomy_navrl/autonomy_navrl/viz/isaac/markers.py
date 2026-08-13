"""Marker prototype configs for Isaac pose visualization."""

from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.markers import VisualizationMarkersCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR


def frame_marker_cfg(prim_path: str, scale: float = 0.35) -> VisualizationMarkersCfg:
    return VisualizationMarkersCfg(
        prim_path=prim_path,
        markers={
            'frame': sim_utils.UsdFileCfg(
                usd_path=f'{ISAAC_NUCLEUS_DIR}/Props/UIElements/frame_prim.usd',
                scale=(scale, scale, scale),
            ),
        },
    )


def arrow_marker_cfg(prim_path: str, color: tuple[float, float, float]) -> VisualizationMarkersCfg:
    return VisualizationMarkersCfg(
        prim_path=prim_path,
        markers={
            'arrow': sim_utils.UsdFileCfg(
                usd_path=f'{ISAAC_NUCLEUS_DIR}/Props/UIElements/arrow_x.usd',
                scale=(1.0, 0.12, 0.12),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color),
            ),
        },
    )


def sphere_marker_cfg(prim_path: str, radius: float, color: tuple[float, float, float]) -> VisualizationMarkersCfg:
    return VisualizationMarkersCfg(
        prim_path=prim_path,
        markers={
            'sphere': sim_utils.SphereCfg(
                radius=radius,
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color, opacity=0.85),
            ),
        },
    )


def tolerance_disk_cfg(prim_path: str) -> VisualizationMarkersCfg:
    return VisualizationMarkersCfg(
        prim_path=prim_path,
        markers={
            'disk': sim_utils.CylinderCfg(
                radius=1.0,
                height=0.015,
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.2, 0.9, 0.3), opacity=0.35),
            ),
        },
    )


def footprint_rect_cfg(
    prim_path: str,
    color: tuple[float, float, float],
    *,
    opacity: float = 0.45,
) -> VisualizationMarkersCfg:
    """Unit cuboid scaled to (length, width, height) for parking-spot / robot footprint."""
    return VisualizationMarkersCfg(
        prim_path=prim_path,
        markers={
            'rect': sim_utils.CuboidCfg(
                size=(1.0, 1.0, 1.0),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color, opacity=opacity),
            ),
        },
    )
