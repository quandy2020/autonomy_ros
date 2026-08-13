"""S10 wheeled quadruped defaults (aligned with s10_jit observation layout)."""

from __future__ import annotations

S10_URDF_REL = 'urdf/S10-urdf-2/urdf/S10.urdf'

# Standing pose — tune after Isaac Lab locomotion training if needed.
S10_INIT_JOINT_POS: dict[str, float] = {
    'fl_hipx_joint': 0.0,
    'fr_hipx_joint': 0.0,
    'hl_hipx_joint': 0.0,
    'hr_hipx_joint': 0.0,
    'fl_hipy_joint': 0.67,
    'fr_hipy_joint': 0.67,
    'hl_hipy_joint': 0.67,
    'hr_hipy_joint': 0.67,
    'fl_knee_joint': -1.30,
    'fr_knee_joint': -1.30,
    'hl_knee_joint': -1.30,
    'hr_knee_joint': -1.30,
    'fl_wheel_joint': 0.0,
    'fr_wheel_joint': 0.0,
    'hl_wheel_joint': 0.0,
    'hr_wheel_joint': 0.0,
}

S10_ISAAC_DEFAULTS: dict = {
    'isaac': {
        'use_fabric': True,
        'replicate_physics': False,
        'env_spacing': 6.0,
        'enable_cameras': True,
        'merge_fixed_joints': False,
        'force_usd_conversion': True,
    },
    'robot': {
        'preset': 's10',
        'kind': 'wheeled_legged',
        'base_link': 'base_link',
        'imu_link': 'imu_link',
        'camera_link': 'camera_link',
        'camera_optical_frame': 'camera_optical_frame',
        'spawn_position': [0.0, 0.0, 0.40],
        'spawn_orientation': [0.0, 0.0, 0.0, 1.0],
        'init_joint_pos': S10_INIT_JOINT_POS,
        'imu_offset': [0.0, 0.0, 0.0],
    },
    'control': {
        'action_model': 'holonomic_3d',
        'max_vx': 0.5,
        'max_vy': 0.3,
        'max_w': 1.0,
    },
    'sensors': {
        'imu': {'enabled': True},
        'odom': {'enabled': True},
    },
    'locomotion': {
        'backend': 's10_gait',
        'decimation': 4,
        'scale': [0.5, 0.3, 1.0],
        'enable_clip': True,
        'clip': {'max_vx': 0.6, 'max_vy': 0.4, 'max_w': 1.0},
        'joint_vel_action_scale': 5.0,
        'gait': {
            'mode': 'auto',
            'step_freq_hz': 1.5,
            'hipy_amp': 0.14,
            'knee_amp': 0.22,
            'hipx_amp': 0.10,
            'march_when_idle': False,
            'precision_speed_mps': 0.07,
        },
    },
}
