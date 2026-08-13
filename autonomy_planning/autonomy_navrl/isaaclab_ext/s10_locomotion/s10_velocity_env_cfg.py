"""S10 articulation and velocity-tracking env cfg (Isaac Lab RSL-RL).

Observation/action layout matches ``autonomy_navrl.plugins.locomotion.s10_jit``:
  obs 57 = imu_ang_vel*0.25(3) + imu_gravity(3) + vel_cmd(3)
         + joint_pos_rel_no_wheel(16) + joint_vel*0.05(16) + last_action(16)
  act 16 = leg_pos(12) @ scale 0.25 + wheel_vel(4) @ scale 5.0
"""

from __future__ import annotations

import os
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import DCMotorCfg, ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ImuCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import (
    LocomotionVelocityRoughEnvCfg,
    MySceneCfg,
)

from . import mdp as s10_mdp

_PKG_ROOT = Path(__file__).resolve().parents[2]
_S10_URDF = _PKG_ROOT / 'urdf' / 'S10-urdf-2' / 'urdf' / 'S10.urdf'
if not _S10_URDF.is_file():
    _S10_URDF = Path(os.environ.get(
        'S10_URDF_PATH',
        '/workspace/autonomy/src/autonomy_ros/autonomy_planning/autonomy_navrl'
        '/urdf/S10-urdf-2/urdf/S10.urdf',
    ))

# Kinematic standing height: hip→wheel ≈0.29 m + wheel radius 0.081 ≈ 0.37 m.
# Spawn slightly above contact to avoid ground penetration.
S10_SPAWN_Z = 0.40

S10_INIT_JOINT_POS = {
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

S10_LEG_JOINTS = [
    'fl_hipx_joint', 'fl_hipy_joint', 'fl_knee_joint',
    'fr_hipx_joint', 'fr_hipy_joint', 'fr_knee_joint',
    'hl_hipx_joint', 'hl_hipy_joint', 'hl_knee_joint',
    'hr_hipx_joint', 'hr_hipy_joint', 'hr_knee_joint',
]
S10_WHEEL_JOINTS = [
    'fl_wheel_joint', 'fr_wheel_joint', 'hl_wheel_joint', 'hr_wheel_joint',
]
S10_ALL_JOINTS = S10_LEG_JOINTS + S10_WHEEL_JOINTS

S10_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path=str(_S10_URDF),
        fix_base=False,
        merge_fixed_joints=False,
        make_instanceable=True,
        force_usd_conversion=True,
        activate_contact_sensors=True,
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                stiffness=0.0,
                damping=0.0,
            ),
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, S10_SPAWN_Z),
        joint_pos=S10_INIT_JOINT_POS,
    ),
    actuators={
        'legs': DCMotorCfg(
            joint_names_expr=S10_LEG_JOINTS,
            effort_limit=50.0,
            saturation_effort=50.0,
            velocity_limit=30.0,
            stiffness=80.0,
            damping=2.0,
            friction=0.0,
        ),
        'wheels': ImplicitActuatorCfg(
            joint_names_expr=S10_WHEEL_JOINTS,
            effort_limit_sim=14.0,
            velocity_limit_sim=65.5,
            stiffness=0.0,
            damping=0.5,
            friction=0.0,
        ),
    },
)


@configclass
class S10SceneCfg(MySceneCfg):
    """Rough terrain scene with body-aligned IMU on imu_link."""

    imu = ImuCfg(
        prim_path='{ENV_REGEX_NS}/Robot/imu_link',
        update_period=0.0,
        # Use raw specific force without extra gravity bias for projected-gravity obs.
        gravity_bias=(0.0, 0.0, 0.0),
    )


@configclass
class S10ActionsCfg:
    """Leg position + wheel velocity actions (16-dim, matches s10_jit)."""

    joint_pos = mdp.JointPositionActionCfg(
        asset_name='robot',
        joint_names=S10_LEG_JOINTS,
        scale={
            '.*_hipx_joint': 0.125,
            '.*_hipy_joint': 0.25,
            '.*_knee_joint': 0.25,
        },
        use_default_offset=True,
    )
    joint_vel = mdp.JointVelocityActionCfg(
        asset_name='robot',
        joint_names=S10_WHEEL_JOINTS,
        scale=5.0,
        use_default_offset=True,
    )


@configclass
class S10RoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    """S10 velocity tracking on rough terrain (train low-level JIT policy)."""

    scene: S10SceneCfg = S10SceneCfg(num_envs=4096, env_spacing=2.5)
    actions: S10ActionsCfg = S10ActionsCfg()

    def __post_init__(self) -> None:
        super().__post_init__()
        self.scene.robot = S10_CFG.replace(prim_path='{ENV_REGEX_NS}/Robot')
        self.scene.height_scanner.prim_path = '{ENV_REGEX_NS}/Robot/base_link'

        # --- Policy observations (57-dim, IMU + wheel-zeroed joints) ---
        self.observations.policy.base_lin_vel = None
        self.observations.policy.height_scan = None
        self.observations.policy.base_ang_vel = ObsTerm(
            func=mdp.imu_ang_vel,
            params={'asset_cfg': SceneEntityCfg('imu')},
            noise=Unoise(n_min=-0.2, n_max=0.2),
            scale=0.25,
        )
        self.observations.policy.projected_gravity = ObsTerm(
            func=mdp.imu_projected_gravity,
            params={'asset_cfg': SceneEntityCfg('imu')},
            noise=Unoise(n_min=-0.05, n_max=0.05),
        )
        self.observations.policy.joint_pos = ObsTerm(
            func=s10_mdp.joint_pos_rel_without_wheel,
            params={
                'asset_cfg': SceneEntityCfg(
                    'robot', joint_names=S10_ALL_JOINTS, preserve_order=True,
                ),
            },
            noise=Unoise(n_min=-0.01, n_max=0.01),
        )
        self.observations.policy.joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={
                'asset_cfg': SceneEntityCfg(
                    'robot', joint_names=S10_ALL_JOINTS, preserve_order=True,
                ),
            },
            noise=Unoise(n_min=-1.5, n_max=1.5),
            scale=0.05,
        )

        self.commands.base_velocity.ranges.lin_vel_x = (-0.6, 0.6)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.4, 0.4)
        self.commands.base_velocity.ranges.ang_vel_z = (-1.0, 1.0)
        # Was 0.1 — too many zero-cmd envs taught "stand still" over tracking.
        self.commands.base_velocity.rel_standing_envs = 0.02
        self.commands.base_velocity.debug_vis = False

        # Wheeled: no foot air-time; penalize hipy / base contacts.
        self.rewards.feet_air_time = None
        self.rewards.undesired_contacts.params['sensor_cfg'].body_names = '.*_hipy'
        self.rewards.undesired_contacts.weight = -1.0
        # Keep upright, but not so strong that the policy refuses to move.
        self.rewards.flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-0.5)
        self.rewards.dof_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=-0.5)
        # Emphasize velocity tracking (demo could stand but not follow vx/vy/w).
        self.rewards.track_lin_vel_xy_exp = RewTerm(
            func=mdp.track_lin_vel_xy_exp,
            weight=3.0,
            params={'command_name': 'base_velocity', 'std': 0.35},
        )
        self.rewards.track_ang_vel_z_exp = RewTerm(
            func=mdp.track_ang_vel_z_exp,
            weight=1.5,
            params={'command_name': 'base_velocity', 'std': 0.35},
        )
        self.rewards.action_rate_l2.weight = -0.005
        self.rewards.lin_vel_z_l2.weight = -1.0

        self.terminations.base_contact.params['sensor_cfg'].body_names = 'base_link'

        self.events.add_base_mass.params['asset_cfg'].body_names = 'base_link'
        self.events.add_base_mass.params['mass_distribution_params'] = (-1.0, 1.0)
        self.events.base_com.params['asset_cfg'].body_names = 'base_link'
        self.events.base_external_force_torque.params['asset_cfg'].body_names = 'base_link'
        # Keep reset joints near the standing pose (was 0.5–1.5 → collapse).
        self.events.reset_robot_joints.params['position_range'] = (0.9, 1.1)
        self.events.reset_robot_joints.params['velocity_range'] = (0.0, 0.0)
        self.events.reset_base.params['pose_range'] = {
            'x': (-0.5, 0.5),
            'y': (-0.5, 0.5),
            'yaw': (-3.14, 3.14),
            'roll': (-0.05, 0.05),
            'pitch': (-0.05, 0.05),
        }
        self.events.reset_base.params['velocity_range'] = {
            'x': (-0.1, 0.1),
            'y': (-0.1, 0.1),
            'z': (-0.1, 0.1),
            'roll': (-0.1, 0.1),
            'pitch': (-0.1, 0.1),
            'yaw': (-0.1, 0.1),
        }

        self.episode_length_s = 20.0
        self.decimation = 4
        self.sim.dt = 0.005


@configclass
class S10RoughEnvCfg_PLAY(S10RoughEnvCfg):
    def __post_init__(self) -> None:
        super().__post_init__()
        self.scene.num_envs = 16
        self.scene.env_spacing = 4.0
        self.observations.policy.enable_corruption = False
        self.episode_length_s = 40.0
