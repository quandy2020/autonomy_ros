"""Robot morphology presets and defaults."""

from __future__ import annotations

from typing import Any

# preset_name -> default fields merged into robot YAML (user values win).
ROBOT_PRESETS: dict[str, dict[str, Any]] = {
    'go2w': {
        'kind': 'wheeled_legged',
        'base_link': 'base',
        'imu_link': 'imu',
        'camera_link': 'radar',
        'camera_optical_frame': 'camera_optical_frame',
        'spawn_position': [0.0, 0.0, 0.40],
    },
    's10': {
        'kind': 'wheeled_legged',
        'base_link': 'base_link',
        'imu_link': 'imu_link',
        'camera_link': 'camera_link',
        'camera_optical_frame': 'camera_optical_frame',
        'spawn_position': [0.0, 0.0, 0.45],
        'camera_offset': [0.12, 0.0, 0.08],
        'camera_optical_rpy': [-1.5707963268, 0.0, -1.5707963268],
        'imu_offset': [0.0075, -0.0373, 0.0556],
    },
    'humanoid_g1': {
        'kind': 'humanoid',
        'base_link': 'pelvis',
        'imu_link': 'imu_link',
        'camera_link': 'torso_link',
        'camera_optical_frame': 'camera_optical_frame',
        'spawn_position': [0.0, 0.0, 1.0],
    },
    'diff_drive_base': {
        'kind': 'diff_drive',
        'base_link': 'base_link',
        'imu_link': 'imu_link',
        'camera_link': 'camera_link',
        'camera_optical_frame': 'camera_optical_frame',
        'spawn_position': [0.0, 0.0, 0.12],
    },
}

# kind -> actuator / spawn defaults when preset not used.
KIND_DEFAULTS: dict[str, dict[str, Any]] = {
    'quadruped': {
        'spawn_z': 0.5,
        'actuator_stiffness': 80.0,
        'actuator_damping': 5.0,
    },
    'wheeled_legged': {
        'spawn_z': 0.40,
        'actuator_stiffness': 25.0,
        'actuator_damping': 0.5,
    },
    'humanoid': {
        'spawn_z': 1.0,
        'actuator_stiffness': 120.0,
        'actuator_damping': 8.0,
    },
    'diff_drive': {
        'spawn_z': 0.12,
        'actuator_stiffness': 0.0,
        'actuator_damping': 10.0,
    },
}


def apply_robot_preset(preset: str, robot_raw: dict[str, Any]) -> None:
    """Merge preset defaults; explicit robot_raw keys override preset."""
    if preset not in ROBOT_PRESETS:
        known = ', '.join(sorted(ROBOT_PRESETS))
        raise ValueError(f'Unknown robot.preset "{preset}". Known: {known}')
    defaults = dict(ROBOT_PRESETS[preset])
    for key, value in defaults.items():
        robot_raw.setdefault(key, value)


def kind_defaults(kind: str) -> dict[str, Any]:
    return dict(KIND_DEFAULTS.get(kind, KIND_DEFAULTS['quadruped']))
