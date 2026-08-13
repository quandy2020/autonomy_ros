"""Isaac Lab configuration builder (multi-robot, parameterized)."""

from __future__ import annotations

from typing import Any

import isaaclab.sim as sim_utils
from isaaclab.actuators import DCMotorCfg, ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass

import autonomy_navrl.plugins.bootstrap as _bootstrap

_bootstrap.ensure_plugins()
from autonomy_navrl.core.spec import FrameworkConfig
from autonomy_navrl.plugins.robots import kind_defaults
from autonomy_navrl.plugins.jdrobot.runtime import (
    build_action_smoothing_cfg,
    build_goal_curriculum,
    build_jdrobot_reward_cfg,
)
from autonomy_navrl.plugins.isaac_sensors import build_imu_cfg, build_tiled_camera_cfg, resolve_sensor_paths


@configclass
class NavrlEnvCfg(DirectRLEnvCfg):
    """Direct RL environment configuration for Isaac Lab visual navigation."""

    decimation = 4
    action_space = 3
    state_space = 0
    observation_space = 1

    sim: SimulationCfg = SimulationCfg(dt=0.005, render_interval=4, use_fabric=False)
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=1,
        env_spacing=4.0,
        replicate_physics=False,
    )

    terrain: TerrainImporterCfg = TerrainImporterCfg(
        prim_path='/World/ground',
        terrain_type='plane',
        max_init_terrain_level=None,
        collision_group=-1,
    )

    robot_cfg: ArticulationCfg = ArticulationCfg(
        prim_path='/World/envs/env_.*/Robot',
        spawn=sim_utils.UrdfFileCfg(
            asset_path='',
            fix_base=False,
            merge_fixed_joints=True,
            make_instanceable=True,
            force_usd_conversion=False,
            joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
                gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                    stiffness=80.0,
                    damping=5.0,
                ),
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=4,
                solver_velocity_iteration_count=0,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.5)),
        actuators={
            'all_joints': ImplicitActuatorCfg(
                joint_names_expr=['.*'],
                stiffness=80.0,
                damping=5.0,
            ),
        },
    )

    tiled_camera: object = None  # set in build_navrl_cfg
    imu: object = None

    max_vx: float = 1.0
    max_vy: float = 0.5
    max_w: float = 1.5
    goal_tolerance_m: float = 0.5
    arena_size_m: float = 16.0
    goal_distance_m: float = 8.0
    max_depth_m: float = 5.0
    collision_threshold_m: float = 0.35
    state_dim: int = 19

    reward_progress_scale: float = 1.0
    reward_goal_reached: float = 10.0
    reward_collision_penalty: float = -5.0
    reward_proximity_scale: float = -0.5
    reward_smoothness_scale: float = -0.01
    reward_timeout_penalty: float = -1.0

    num_obstacles: int = 0
    obstacle_size_m: float = 0.5
    obstacle_height_m: float = 1.0
    reward_profile: str = 'default'
    task_kind: str = 'goal_nav'
    yaw_tolerance_rad: float = 0.15
    orientation_reward_scale: float = -5.0
    require_stop_speed_mps: float = 0.0
    collision_terminate: bool = True

    _framework: FrameworkConfig | None = None
    _task_params: dict[str, Any] | None = None
    _init_joint_pos: dict[str, float] | None = None
    _action_smoothing_cfg: object | None = None
    _goal_curriculum: object | None = None
    _jdrobot_reward_cfg: object | None = None
    _viz_cfg: object | None = None
    _reward_raw: dict[str, Any] | None = None
    _locomotion_spec: object | None = None


def _resolve_env_spacing(num_envs: int, arena_size_m: float, isaac_cfg: dict[str, Any]) -> float:
    if 'env_spacing' in isaac_cfg:
        return float(isaac_cfg['env_spacing'])
    return max(arena_size_m * 0.55, 4.0)


def build_navrl_cfg(config: dict[str, Any]) -> NavrlEnvCfg:
    """Build Isaac Lab env cfg from YAML training config."""
    framework = FrameworkConfig.from_yaml_dict(config)
    robot = framework.robot
    action = framework.action
    sensors = framework.sensors
    task = framework.task

    task_cfg = config.get('task', {})
    reward_cfg = config.get('reward', {})
    policy_cfg = config.get('policy', {})
    control_cfg = config.get('control', {})
    num_envs = int(config.get('num_envs', 64))
    isaac_cfg = config.get('isaac', {})
    device = str(config.get('device', 'cuda:0')).lower()
    use_fabric = bool(isaac_cfg.get('use_fabric', device.startswith('cuda')))
    arena_size_m = float(task_cfg.get('arena_size_m', 16.0))
    max_steps = int(task_cfg.get('max_episode_steps', 500))
    merge_fixed_joints = bool(isaac_cfg.get('merge_fixed_joints', False))

    if 'replicate_physics' in isaac_cfg:
        replicate_physics = bool(isaac_cfg['replicate_physics'])
    elif num_envs >= 16 and use_fabric:
        replicate_physics = True
    else:
        replicate_physics = False

    if not use_fabric and num_envs > 1:
        num_envs = int(isaac_cfg.get('num_envs_cpu_max', 1))

    kind_def = kind_defaults(robot.kind)
    paths = resolve_sensor_paths(robot, merge_fixed_joints=merge_fixed_joints)

    env_cfg = NavrlEnvCfg()
    seed = config.get('seed')
    if seed is not None:
        env_cfg.seed = int(seed)
    env_cfg.scene.num_envs = num_envs
    env_cfg.scene.replicate_physics = replicate_physics
    env_cfg.scene.env_spacing = _resolve_env_spacing(num_envs, arena_size_m, isaac_cfg)
    env_cfg.sim.use_fabric = use_fabric
    env_cfg.sim.device = 'cpu' if device.startswith('cpu') else device
    env_cfg.episode_length_s = float(max_steps) * env_cfg.sim.dt * env_cfg.decimation

    env_cfg.action_space = action.dim
    env_cfg.max_vx = action.max_vx
    env_cfg.max_vy = action.max_vy
    env_cfg.max_w = action.max_w

    stiffness = float(kind_def['actuator_stiffness'])
    damping = float(kind_def['actuator_damping'])
    env_cfg.robot_cfg.prim_path = f'/World/envs/env_.*/{robot.prim_name}'
    env_cfg.robot_cfg.spawn.asset_path = robot.urdf_path
    env_cfg.robot_cfg.spawn.merge_fixed_joints = merge_fixed_joints
    env_cfg.robot_cfg.spawn.force_usd_conversion = bool(
        isaac_cfg.get('force_usd_conversion', not merge_fixed_joints)
    )
    env_cfg.robot_cfg.spawn.joint_drive.gains.stiffness = stiffness
    env_cfg.robot_cfg.spawn.joint_drive.gains.damping = damping
    _is_s10_robot = (
        robot.kind == 'wheeled_legged'
        and 'fl_hipx_joint' in robot.init_joint_pos
    )
    if _is_s10_robot:
        from autonomy_navrl.plugins.locomotion.s10_jit import (
            S10_LEG_JOINT_NAMES,
            S10_WHEEL_JOINT_NAMES,
        )

        # Match Isaac Lab S10 velocity env (train_s10_locomotion / s10_velocity_env_cfg).
        env_cfg.robot_cfg.spawn.joint_drive.gains.stiffness = 0.0
        env_cfg.robot_cfg.spawn.joint_drive.gains.damping = 0.0
        env_cfg.robot_cfg.actuators = {
            'legs': DCMotorCfg(
                joint_names_expr=list(S10_LEG_JOINT_NAMES),
                effort_limit=50.0,
                saturation_effort=50.0,
                velocity_limit=30.0,
                stiffness=80.0,
                damping=2.0,
                friction=0.0,
            ),
            'wheels': ImplicitActuatorCfg(
                joint_names_expr=list(S10_WHEEL_JOINT_NAMES),
                effort_limit_sim=14.0,
                velocity_limit_sim=65.5,
                stiffness=0.0,
                damping=0.5,
                friction=0.0,
            ),
        }
    elif robot.kind == 'wheeled_legged':
        # Match Isaac Lab wheeled velocity env: DC legs + velocity wheels.
        env_cfg.robot_cfg.actuators = {
            'legs': DCMotorCfg(
                joint_names_expr=['^(?!.*(_foot_joint|_wheel_joint)$).*'],
                effort_limit=23.5,
                saturation_effort=23.5,
                velocity_limit=30.0,
                stiffness=25.0,
                damping=0.5,
                friction=0.0,
            ),
            'wheels': ImplicitActuatorCfg(
                joint_names_expr=['.*(_foot_joint|_wheel_joint)$'],
                effort_limit_sim=23.5,
                velocity_limit_sim=30.0,
                stiffness=0.0,
                damping=0.5,
                friction=0.0,
            ),
        }
    else:
        env_cfg.robot_cfg.actuators['all_joints'].stiffness = stiffness
        env_cfg.robot_cfg.actuators['all_joints'].damping = damping
    env_cfg.robot_cfg.init_state.pos = robot.spawn_position

    if robot.init_joint_pos:
        env_cfg._init_joint_pos = dict(robot.init_joint_pos)
        env_cfg.robot_cfg.init_state.joint_pos = robot.init_joint_pos

    env_cfg.tiled_camera = build_tiled_camera_cfg(sensors, paths)
    imu_cfg = build_imu_cfg(sensors, paths)
    env_cfg.imu = imu_cfg

    env_cfg.goal_tolerance_m = float(task_cfg.get('goal_tolerance_m', 0.5))
    pos_tol = task_cfg.get('position_tolerance_m')
    if pos_tol is not None:
        env_cfg.goal_tolerance_m = float(pos_tol)
    env_cfg.arena_size_m = arena_size_m
    env_cfg.goal_distance_m = float(task_cfg.get('goal_distance_m', 8.0))
    env_cfg.max_depth_m = sensors.depth_max_m
    env_cfg.collision_threshold_m = sensors.collision_threshold_m
    env_cfg.state_dim = int(policy_cfg.get('state_dim', 19))
    env_cfg.reward_progress_scale = float(reward_cfg.get('progress_scale', 1.0))
    env_cfg.reward_goal_reached = float(reward_cfg.get('goal_reached', 10.0))
    env_cfg.reward_collision_penalty = float(reward_cfg.get('collision_penalty', -5.0))
    env_cfg.reward_proximity_scale = float(reward_cfg.get('proximity_penalty_scale', -0.5))
    env_cfg.reward_smoothness_scale = float(reward_cfg.get('action_smoothness_scale', -0.01))
    env_cfg.reward_timeout_penalty = float(reward_cfg.get('timeout_penalty', -1.0))
    env_cfg.num_obstacles = int(task_cfg.get('num_obstacles', 0))
    env_cfg.obstacle_size_m = float(task_cfg.get('obstacle_size_m', 0.5))
    env_cfg.obstacle_height_m = float(task_cfg.get('obstacle_height_m', 1.0))
    env_cfg.reward_profile = framework.reward_profile
    env_cfg.task_kind = task.kind
    env_cfg.yaw_tolerance_rad = float(task_cfg.get('yaw_tolerance_rad', 0.15))
    env_cfg.orientation_reward_scale = float(task_cfg.get('orientation_reward_scale', -5.0))
    env_cfg.require_stop_speed_mps = float(task_cfg.get('require_stop_speed_mps', 0.0))
    env_cfg.collision_terminate = bool(task_cfg.get('collision_terminate', True))

    env_cfg._framework = framework
    env_cfg._task_params = task.params
    env_cfg._reward_raw = dict(config.get('reward', {}))
    env_cfg._locomotion_spec = framework.locomotion

    env_cfg._action_smoothing_cfg = build_action_smoothing_cfg(control_cfg)
    env_cfg._goal_curriculum = build_goal_curriculum(task_cfg)
    env_cfg._jdrobot_reward_cfg = build_jdrobot_reward_cfg(reward_cfg)
    from autonomy_navrl.viz.isaac.config import PoseVizConfig

    livestream = int(isaac_cfg.get('livestream', 0))
    env_cfg._viz_cfg = PoseVizConfig.from_dict(framework.viz_raw, livestream=livestream)

    viewer_raw = (framework.viz_raw or {}).get('viewer', {})
    if viewer_raw:
        eye = viewer_raw.get('eye')
        if eye is not None:
            env_cfg.viewer.eye = tuple(float(v) for v in eye[:3])
        lookat = viewer_raw.get('lookat')
        if lookat is not None:
            env_cfg.viewer.lookat = tuple(float(v) for v in lookat[:3])
        resolution = viewer_raw.get('resolution')
        if resolution is not None:
            env_cfg.viewer.resolution = (int(resolution[0]), int(resolution[1]))
        origin_type = viewer_raw.get('origin_type')
        if origin_type is not None:
            env_cfg.viewer.origin_type = str(origin_type)
        env_index = viewer_raw.get('env_index')
        if env_index is not None:
            env_cfg.viewer.env_index = int(env_index)
        asset_name = viewer_raw.get('asset_name')
        if asset_name is not None:
            env_cfg.viewer.asset_name = str(asset_name)

    return env_cfg
