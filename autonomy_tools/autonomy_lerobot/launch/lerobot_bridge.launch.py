# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Launch autonomy_lerobot in online or offline mode."""

from pathlib import Path

import yaml
from autonomy_lerobot.data_paths import lerobot_habitat_nav2_root
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from autonomy_lerobot.launch_utils import lerobot_bridge_node

_COMMON_DECLARE_ARG_SPECS = (
    (
        'mode',
        'online',
        'online: bridge ROS topics, offline: convert rosbag/mcap',
    ),
    ('config_file', None, 'online mode only: LeRobot bridge parameter file'),
    ('offline_config_file', None, 'offline mode config file; CLI args override file values'),
    ('dataset_root', '', 'dataset root for online recording or offline output parent'),
)


_OFFLINE_ARG_SPECS = (
    {
        'name': 'bag_path',
        'section': 'input',
        'flag': '--bag',
        'kind': 'path',
        'description': 'single input mcap file',
    },
    {
        'name': 'input_dir',
        'section': 'input',
        'flag': '--input-dir',
        'kind': 'path',
        'description': 'input directory for recursive mcap discovery',
    },
    {
        'name': 'scene_name',
        'section': 'input',
        'flag': '--scene-name',
        'kind': 'path',
        'description': 'explicit scene name for single-bag conversion',
    },
    {
        'name': 'repo_id',
        'section': 'output',
        'flag': '--repo-id',
        'kind': 'required',
        'default': 'local/habitat_nav2',
        'description': 'LeRobot dataset repo id',
    },
    {
        'name': 'robot_type',
        'section': 'input',
        'flag': '--robot-type',
        'kind': 'required',
        'default': 'jdrobot',
        'description': 'dataset robot type',
    },
    {
        'name': 'task',
        'section': 'input',
        'flag': '--task',
        'kind': 'required',
        'default': 'navigate to goal',
        'description': 'task text stored in dataset',
    },
    {
        'name': 'fps',
        'section': 'output',
        'flag': '--fps',
        'kind': 'required',
        'default': '20.0',
        'description': 'target output fps',
    },
    {
        'name': 'episode_seconds',
        'section': 'output',
        'flag': '--episode-seconds',
        'kind': 'required',
        'default': '41.5',
        'description': 'fixed-mode slice length; nav mode uses odom motion boundaries',
    },
    {
        'name': 'episode_split_mode',
        'section': 'output',
        'flag': '--episode-split-mode',
        'kind': 'required',
        'default': 'nav',
        'description': 'nav (autonomy_task-aligned) or fixed time windows',
    },
    {
        'name': 'video_vcodec',
        'section': 'output',
        'flag': '--video-vcodec',
        'kind': 'required',
        'default': 'av1',
        'description': 'video codec for encoded image streams',
    },
    {
        'name': 'overwrite',
        'section': 'output',
        'flag': '--overwrite',
        'kind': 'flag',
        'description': 'overwrite existing output scene',
    },
    {
        'name': 'rgb_topic',
        'section': 'input',
        'flag': '--rgb-topic',
        'kind': 'value',
        'description': 'RGB image topic',
    },
    {
        'name': 'depth_topic',
        'section': 'input',
        'flag': '--depth-topic',
        'kind': 'value',
        'description': 'depth image topic',
    },
    {
        'name': 'camera_info_topic',
        'section': 'input',
        'flag': '--camera-info-topic',
        'kind': 'value',
        'description': 'camera info topic',
    },
    {
        'name': 'odom_topic',
        'section': 'input',
        'flag': '--odom-topic',
        'kind': 'value',
        'description': 'primary odometry topic',
    },
    {
        'name': 'odom_fallback_topic',
        'section': 'input',
        'flag': '--odom-fallback-topic',
        'kind': 'value',
        'description': 'fallback odometry topic',
    },
    {
        'name': 'pointcloud_stride',
        'section': 'pointcloud',
        'flag': '--pointcloud-stride',
        'kind': 'value',
        'description': 'pixel stride for depth-to-pointcloud sampling',
    },
    {
        'name': 'max_pointcloud_points',
        'section': 'pointcloud',
        'flag': '--max-pointcloud-points',
        'kind': 'value',
        'description': 'max saved point count per frame',
    },
    {
        'name': 'depth_min_m',
        'section': 'pointcloud',
        'flag': '--depth-min-m',
        'kind': 'value',
        'description': 'minimum valid depth in meters',
    },
    {
        'name': 'depth_max_m',
        'section': 'pointcloud',
        'flag': '--depth-max-m',
        'kind': 'value',
        'description': 'maximum valid depth in meters',
    },
    {
        'name': 'record_before_sec',
        'section': 'episode',
        'flag': '--record-before-sec',
        'kind': 'value',
        'description': 'pre-nav recording duration (online/offline shared)',
    },
    {
        'name': 'record_after_sec',
        'section': 'episode',
        'flag': '--record-after-sec',
        'kind': 'value',
        'description': 'post-arrival recording duration (online/offline shared)',
    },
    {
        'name': 'max_nav_sec',
        'section': 'episode',
        'flag': '--max-nav-sec',
        'kind': 'value',
        'description': 'hard nav duration cap per episode',
    },
    {
        'name': 'stall_move_m',
        'section': 'episode',
        'flag': '--stall-move-m',
        'kind': 'value',
        'description': 'minimum odom displacement to count as moving',
    },
    {
        'name': 'sam3_enable',
        'section': 'sam3',
        'flag': '--sam3-enable',
        'kind': 'flag',
        'description': 'enable SAM3 semantic segmentation',
    },
    {
        'name': 'sam3_device',
        'section': 'sam3',
        'flag': '--sam3-device',
        'kind': 'value',
        'description': 'SAM3 inference device',
    },
    {
        'name': 'sam3_confidence',
        'section': 'sam3',
        'flag': '--sam3-confidence',
        'kind': 'value',
        'description': 'SAM3 confidence threshold',
    },
    {
        'name': 'sam3_checkpoint',
        'section': 'sam3',
        'flag': '--sam3-checkpoint',
        'kind': 'value',
        'description': 'explicit SAM3 checkpoint path',
    },
    {
        'name': 'sam3_prompts',
        'section': 'sam3',
        'flag': '--sam3-prompts',
        'kind': 'prompts',
        'description': 'comma-separated SAM3 text prompts',
    },
    {
        'name': 'save_segmentation_viz',
        'section': 'sam3',
        'flag': '--save-segmentation-viz',
        'kind': 'flag',
        'description': 'save segmentation visualization artifacts',
    },
    {
        'name': 'skip_stationary_runs',
        'section': 'cleanup',
        'flag': '--skip-stationary-runs',
        'kind': 'flag',
        'description': 'drop nearly stationary runs after conversion',
    },
    {
        'name': 'stationary_pos_threshold',
        'section': 'cleanup',
        'flag': '--stationary-pos-threshold',
        'kind': 'value',
        'description': 'stationary position threshold in meters',
    },
    {
        'name': 'stationary_yaw_threshold',
        'section': 'cleanup',
        'flag': '--stationary-yaw-threshold',
        'kind': 'value',
        'description': 'stationary yaw threshold in radians',
    },
    {
        'name': 'dry_run_clean',
        'section': 'cleanup',
        'flag': '--dry-run-clean',
        'kind': 'flag',
        'description': 'report cleanup decisions without deleting runs',
    },
)


def _offline_declare_arg_specs():
    for spec in _OFFLINE_ARG_SPECS:
        yield (spec['name'], '', spec.get('description'))


def _section_keys(section: str) -> tuple[str, ...]:
    return tuple(spec['name'] for spec in _OFFLINE_ARG_SPECS if spec['section'] == section)


_OFFLINE_SECTION_KEYS = {
    'input': _section_keys('input'),
    'output': ('dataset_root',) + _section_keys('output'),
    'episode': _section_keys('episode'),
    'pointcloud': _section_keys('pointcloud'),
    'sam3': _section_keys('sam3'),
    'cleanup': _section_keys('cleanup'),
}


def _flatten_offline_config(offline: dict[str, object]) -> dict[str, str]:
    flat: dict[str, str] = {}
    for key, value in offline.items():
        if value is None:
            continue
        if key in _OFFLINE_SECTION_KEYS:
            if not isinstance(value, dict):
                raise RuntimeError(f'offline config section `{key}` must be a mapping')
            for nested_key in _OFFLINE_SECTION_KEYS[key]:
                nested_value = value.get(nested_key)
                if nested_value is not None:
                    flat[nested_key] = str(nested_value)
            continue
        flat[str(key)] = str(value)
    return flat


def _load_offline_config(path_str: str) -> dict[str, str]:
    path = Path(path_str).expanduser()
    if not path.is_file():
        raise RuntimeError(f'offline config not found: {path}')
    data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f'offline config must be a mapping: {path}')
    offline = data.get('offline', data)
    if not isinstance(offline, dict):
        raise RuntimeError(f'offline config `offline` must be a mapping: {path}')
    return _flatten_offline_config(offline)


def _launch_or_config(
    context,
    offline_config: dict[str, str],
    name: str,
    default: str = '',
) -> str:
    value = LaunchConfiguration(name).perform(context).strip()
    if value:
        return value
    return offline_config.get(name, default)


def _build_offline_command(context, offline_config: dict[str, str]) -> list[str]:
    dataset_root = _launch_or_config(
        context,
        offline_config,
        'dataset_root',
        str(lerobot_habitat_nav2_root()),
    )
    command = [
        'python3',
        '-m',
        'autonomy_lerobot.bag_to_lerobot',
        '--output-root',
        dataset_root,
    ]
    for spec in _OFFLINE_ARG_SPECS:
        name = spec['name']
        flag = spec['flag']
        kind = spec['kind']
        if kind == 'required':
            command.extend([flag, _launch_or_config(context, offline_config, name, spec['default'])])
            continue
        value = _launch_or_config(context, offline_config, name)
        if kind == 'flag':
            if value.lower() == 'true':
                command.append(flag)
            continue
        if not value:
            continue
        if kind == 'prompts':
            command.extend([flag, *[item for item in value.split(',') if item]])
            continue
        command.extend([flag, value])
    return command


def _declare_launch_arg(name: str, default_value, description: str | None = None):
    kwargs = {'default_value': default_value}
    if description:
        kwargs['description'] = description
    return DeclareLaunchArgument(name, **kwargs)


def _build_mode_action(context, *_args, **_kwargs):
    mode = LaunchConfiguration('mode').perform(context).strip().lower()
    config_file = LaunchConfiguration('config_file')

    if mode == 'online':
        dataset_root = LaunchConfiguration('dataset_root').perform(context)
        return [
            lerobot_bridge_node(
                output='screen',
                parameters=[
                    config_file,
                    {'dataset_root': dataset_root},
                ],
            )
        ]

    if mode != 'offline':
        raise RuntimeError(
            f"unsupported mode={mode!r}; use 'online' or 'offline'"
        )

    offline_config = _load_offline_config(
        LaunchConfiguration('offline_config_file').perform(context).strip()
    )
    return [ExecuteProcess(cmd=_build_offline_command(context, offline_config), output='screen')]


def generate_launch_description() -> LaunchDescription:
    config_file = PathJoinSubstitution([
        FindPackageShare('autonomy_lerobot'),
        'config',
        'lerobot_bridge.yaml',
    ])
    offline_config_file = PathJoinSubstitution([
        FindPackageShare('autonomy_lerobot'),
        'config',
        'lerobot_offline.yaml',
    ])
    actions = []
    declare_arg_specs = _COMMON_DECLARE_ARG_SPECS + tuple(_offline_declare_arg_specs())
    for name, default_value, description in declare_arg_specs:
        resolved_default = default_value
        if name == 'config_file':
            resolved_default = config_file
        elif name == 'offline_config_file':
            resolved_default = offline_config_file
        actions.append(_declare_launch_arg(name, resolved_default, description))
    actions.append(OpaqueFunction(function=_build_mode_action))
    return LaunchDescription(actions)
