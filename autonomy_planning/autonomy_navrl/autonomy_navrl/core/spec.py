"""Framework configuration specs (robot / task / action / sensors)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from autonomy_navrl.core.locomotion_spec import LocomotionSpec


# Supported robot morphologies (kinematic family).
ROBOT_KINDS = frozenset({'quadruped', 'wheeled_legged', 'humanoid', 'diff_drive'})

# High-level velocity command models.
ACTION_MODELS = frozenset({'holonomic_3d', 'diff_drive_2d'})

# Built-in navigation tasks — prefer task_registry.names() after ensure_plugins().
TASK_KINDS = frozenset({'goal_nav', 'precision_pose', 'go2w_precision_pose', 'waypoint_nav'})


@dataclass(frozen=True)
class RobotSpec:
    """Robot asset and link layout."""

    kind: str
    urdf_path: str
    base_link: str = 'base_link'
    imu_link: str = 'imu_link'
    camera_link: str = 'camera_link'
    camera_optical_frame: str = 'camera_optical_frame'
    spawn_position: tuple[float, float, float] = (0.0, 0.0, 0.5)
    spawn_orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    init_joint_pos: dict[str, float] = field(default_factory=dict)
    camera_offset: tuple[float, float, float] | None = None
    camera_optical_rpy: tuple[float, float, float] | None = None
    imu_offset: tuple[float, float, float] | None = None
    prim_name: str = 'Robot'

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> RobotSpec:
        kind = str(raw.get('kind', 'quadruped'))
        if kind not in ROBOT_KINDS:
            raise ValueError(f'robot.kind must be one of {sorted(ROBOT_KINDS)}, got {kind!r}')
        urdf_path = str(raw.get('urdf_path', '')).strip()
        if not urdf_path:
            raise ValueError('robot.urdf_path is required')
        spawn = raw.get('spawn_position', [0.0, 0.0, 0.5])
        orient = raw.get('spawn_orientation', [0.0, 0.0, 0.0, 1.0])
        init_joints = raw.get('init_joint_pos') or {}
        cam_off = raw.get('camera_offset')
        cam_rpy = raw.get('camera_optical_rpy')
        imu_off = raw.get('imu_offset')
        return cls(
            kind=kind,
            urdf_path=urdf_path,
            base_link=str(raw.get('base_link', 'base_link')),
            imu_link=str(raw.get('imu_link', 'imu_link')),
            camera_link=str(raw.get('camera_link', 'camera_link')),
            camera_optical_frame=str(raw.get('camera_optical_frame', 'camera_optical_frame')),
            spawn_position=tuple(float(v) for v in spawn[:3]),
            spawn_orientation=tuple(float(v) for v in orient[:4]),
            init_joint_pos={str(k): float(v) for k, v in init_joints.items()},
            camera_offset=tuple(float(v) for v in cam_off[:3]) if cam_off else None,
            camera_optical_rpy=tuple(float(v) for v in cam_rpy[:3]) if cam_rpy else None,
            imu_offset=tuple(float(v) for v in imu_off[:3]) if imu_off else None,
            prim_name=str(raw.get('prim_name', 'Robot')),
        )


@dataclass(frozen=True)
class ActionSpec:
    """Normalized action -> robot command mapping."""

    model: str
    dim: int
    max_vx: float = 1.0
    max_vy: float = 0.5
    max_w: float = 1.5

    @classmethod
    def from_dict(cls, raw: dict[str, Any], robot_kind: str) -> ActionSpec:
        from autonomy_navrl.plugins.bootstrap import ensure_plugins
        from autonomy_navrl.plugins.actions import action_registry

        ensure_plugins()

        default_model = 'diff_drive_2d' if robot_kind == 'diff_drive' else 'holonomic_3d'
        model = str(raw.get('action_model', default_model))
        known = set(action_registry.names())
        if model not in known:
            raise ValueError(
                f'control.action_model must be one of {sorted(known)}, got {model!r}'
            )
        dim = 2 if model == 'diff_drive_2d' else int(raw.get('action_dim', 3))
        return cls(
            model=model,
            dim=dim,
            max_vx=float(raw.get('max_vx', 1.0)),
            max_vy=float(raw.get('max_vy', 0.5)),
            max_w=float(raw.get('max_w', 1.5)),
        )


@dataclass(frozen=True)
class SensorSpec:
    """RGB-D + optional IMU / odom feature flags."""

    rgb_width: int = 128
    rgb_height: int = 96
    fov_deg: float = 90.0
    focal_length: float = 24.0
    depth_min_m: float = 0.1
    depth_max_m: float = 5.0
    collision_threshold_m: float = 0.35
    imu_enabled: bool = True
    odom_enabled: bool = True

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> SensorSpec:
        rgb = raw.get('rgb', {})
        depth = raw.get('depth', {})
        imu = raw.get('imu', {})
        odom = raw.get('odom', {})
        return cls(
            rgb_width=int(rgb.get('width', 128)),
            rgb_height=int(rgb.get('height', 96)),
            fov_deg=float(rgb.get('fov_deg', 90.0)),
            focal_length=float(rgb.get('focal_length', 24.0)),
            depth_min_m=float(depth.get('min_range_m', 0.1)),
            depth_max_m=float(depth.get('max_range_m', 5.0)),
            collision_threshold_m=float(depth.get('collision_threshold_m', 0.35)),
            imu_enabled=bool(imu.get('enabled', True)),
            odom_enabled=bool(odom.get('enabled', True)),
        )


@dataclass(frozen=True)
class TaskSpec:
    """Task definition with optional custom params."""

    kind: str
    params: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> TaskSpec:
        from autonomy_navrl.plugins.bootstrap import ensure_plugins
        from autonomy_navrl.plugins.tasks import task_registry

        ensure_plugins()

        kind = str(raw.get('kind') or raw.get('mode', 'goal_nav'))
        alias = {'navigate': 'goal_nav', 'precision_pose': 'precision_pose'}
        kind = alias.get(kind, kind)
        known = set(task_registry.names())
        if kind not in known:
            raise ValueError(f'task.kind must be one of {sorted(known)}, got {kind!r}')
        params = {k: v for k, v in raw.items() if k not in ('kind', 'mode')}
        return cls(kind=kind, params=params)


@dataclass(frozen=True)
class FrameworkConfig:
    """Parsed top-level framework sections from training YAML."""

    robot: RobotSpec
    action: ActionSpec
    sensors: SensorSpec
    task: TaskSpec
    reward_profile: str = 'default'
    locomotion: LocomotionSpec = field(default_factory=LocomotionSpec)
    reward_raw: dict[str, Any] = field(default_factory=dict)
    viz_raw: dict[str, Any] | None = None

    @classmethod
    def from_yaml_dict(cls, config: dict[str, Any]) -> FrameworkConfig:
        from autonomy_navrl.plugins.bootstrap import ensure_plugins
        from autonomy_navrl.plugins.locomotion.base import locomotion_registry
        from autonomy_navrl.plugins.rewards import reward_registry

        ensure_plugins()

        robot_raw = dict(config.get('robot', {}))
        preset = robot_raw.pop('preset', None)
        if preset:
            from autonomy_navrl.plugins.robots import apply_robot_preset

            apply_robot_preset(preset, robot_raw)
        robot = RobotSpec.from_dict(robot_raw)
        action = ActionSpec.from_dict(config.get('control', {}), robot.kind)
        sensors = SensorSpec.from_dict(config.get('sensors', {}))
        task = TaskSpec.from_dict(config.get('task', {}))
        reward_profile = str(config.get('reward', {}).get('profile', 'default'))
        reward_raw = dict(config.get('reward', {}))

        if reward_profile not in reward_registry.names():
            raise ValueError(
                f'reward.profile must be one of {reward_registry.names()}, got {reward_profile!r}'
            )
        locomotion = LocomotionSpec.from_dict(config.get('locomotion'))

        if locomotion.backend not in locomotion_registry.names():
            raise ValueError(
                f'locomotion.backend must be one of {locomotion_registry.names()}, '
                f'got {locomotion.backend!r}'
            )
        viz_raw = config.get('viz')
        return cls(
            robot=robot,
            action=action,
            sensors=sensors,
            task=task,
            reward_profile=reward_profile,
            locomotion=locomotion,
            reward_raw=reward_raw,
            viz_raw=viz_raw,
        )
