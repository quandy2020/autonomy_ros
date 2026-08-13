"""Configuration for modular Isaac pose visualization."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


DEFAULT_PRECISION_MODULES = (
    'target_footprint',
    'target_heading_arrow',
    'robot_footprint',
    'robot_heading_arrow',
)

MODULE_REGISTRY_KEYS = frozenset(
    {
        'target_pose_frame',
        'robot_pose_frame',
        'target_sphere',
        'target_footprint',
        'robot_footprint',
        'target_heading_arrow',
        'robot_heading_arrow',
        'position_tolerance',
        'velocity_command',
    }
)


@dataclass
class PoseVizConfig:
    """Which pose/orientation markers to draw in Isaac Sim."""

    enabled: bool = False
    auto_enable_on_livestream: bool = True
    modules: list[str] = field(default_factory=lambda: list(DEFAULT_PRECISION_MODULES))
    frame_scale: float = 0.35
    arrow_length: float = 0.45
    marker_z_offset: float = 0.05
    spot_length: float = 0.9
    spot_width: float = 0.3
    robot_length: float = 0.9
    robot_width: float = 0.3
    footprint_height: float = 0.012

    @classmethod
    def from_dict(cls, raw: dict[str, Any] | None, *, livestream: int = 0) -> PoseVizConfig:
        if not raw:
            enabled = livestream > 0
            return cls(enabled=enabled)
        enabled = bool(raw.get('enabled', False))
        if bool(raw.get('auto_enable_on_livestream', True)) and livestream > 0:
            enabled = True
        modules = list(raw.get('modules') or DEFAULT_PRECISION_MODULES)
        unknown = [name for name in modules if name not in MODULE_REGISTRY_KEYS]
        if unknown:
            raise ValueError(f'Unknown viz.modules: {unknown}. Valid: {sorted(MODULE_REGISTRY_KEYS)}')
        return cls(
            enabled=enabled,
            auto_enable_on_livestream=bool(raw.get('auto_enable_on_livestream', True)),
            modules=modules,
            frame_scale=float(raw.get('frame_scale', 0.35)),
            arrow_length=float(raw.get('arrow_length', 0.45)),
            marker_z_offset=float(raw.get('marker_z_offset', 0.05)),
            spot_length=float(raw.get('spot_length', 0.9)),
            spot_width=float(raw.get('spot_width', 0.3)),
            robot_length=float(raw.get('robot_length', 0.9)),
            robot_width=float(raw.get('robot_width', 0.3)),
            footprint_height=float(raw.get('footprint_height', 0.012)),
        )
