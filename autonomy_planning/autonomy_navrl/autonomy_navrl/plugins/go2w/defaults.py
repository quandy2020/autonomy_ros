"""Go2W shared defaults (Isaac Lab unitree_go2w aligned)."""

from __future__ import annotations

GO2W_URDF_REL = 'urdf/go2w_description/urdf/go2w_description.urdf'

GO2W_INIT_JOINT_POS: dict[str, float] = {
    'FR_hip_joint': 0.0,
    'FL_hip_joint': 0.0,
    'RR_hip_joint': 0.0,
    'RL_hip_joint': 0.0,
    'FR_thigh_joint': 0.8,
    'FL_thigh_joint': 0.8,
    'RR_thigh_joint': 0.8,
    'RL_thigh_joint': 0.8,
    'FR_calf_joint': -1.5,
    'FL_calf_joint': -1.5,
    'RR_calf_joint': -1.5,
    'RL_calf_joint': -1.5,
    'FR_foot_joint': 0.0,
    'FL_foot_joint': 0.0,
    'RR_foot_joint': 0.0,
    'RL_foot_joint': 0.0,
}

GO2W_ISAAC_DEFAULTS: dict = {
    'isaac': {
        'use_fabric': True,
        'replicate_physics': True,
        'env_spacing': 6.0,
        'enable_cameras': True,
        'merge_fixed_joints': False,
        'force_usd_conversion': True,
    },
    'robot': {
        'preset': 'go2w',
        'kind': 'wheeled_legged',
        'base_link': 'base',
        'imu_link': 'imu',
        'camera_link': 'radar',
        'camera_optical_frame': 'camera_optical_frame',
        'spawn_position': [0.0, 0.0, 0.40],
        'spawn_orientation': [0.0, 0.0, 0.0, 1.0],
        'init_joint_pos': GO2W_INIT_JOINT_POS,
    },
    'sensors': {
        'rgb': {'width': 96, 'height': 72, 'fov_deg': 90.0},
        'depth': {'min_range_m': 0.1, 'max_range_m': 5.0, 'collision_threshold_m': 0.35},
        'imu': {'enabled': True},
        'odom': {'enabled': True},
    },
    'control': {
        'action_model': 'holonomic_3d',
        'max_vx': 0.5,
        'max_vy': 0.5,
        'max_w': 1.0,
        'action_smoothing': {'enabled': True, 'max_lin_acc': 0.8, 'max_ang_acc': 1.5},
    },
    'locomotion': {
        'backend': 'go2w_jit',
        'decimation': 4,
        'scale': [0.5, 0.5, 1.0],
        'enable_clip': True,
        'clip': {'max_vx': 0.5, 'max_vy': 0.5, 'max_w': 1.0},
        'joint_pos_action_scale': 0.25,
        'joint_vel_action_scale': 5.0,
        'hip_joint_pos_scale': 0.125,
    },
    'policy': {'state_dim': 20, 'hidden_dim': 256},
}

GO2W_PRECISION_TASK: dict = {
    'kind': 'go2w_precision_pose',
    'position_tolerance_m': 0.08,
    'yaw_tolerance_rad': 0.12,
    'max_episode_steps': 800,
    'consecutive_iou': {
        'enabled': True,
        'iou_threshold': 0.95,
        'num_success': 10,
    },
}

GO2W_PRECISION_REWARD: dict = {
    'profile': 'precision_iou',
    'precision_iou': {
        'use_bbox_iou': True,
        'iou_threshold': 0.95,
        'success_bonus': 50.0,
        'position_tolerance_m': 0.08,
        'yaw_tolerance_rad': 0.12,
    },
    'collision_penalty': -5.0,
    'timeout_penalty': -1.0,
}
