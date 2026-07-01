"""Launch autonomy_slam in mono / stereo / RGB-D mode."""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_path(value: str, default: str) -> str:
    return value if value else default


def _ld_library_path() -> str:
    """Prepend runtime lib dirs for fbow/g2o and autonomy transitive deps."""
    parts = []
    if os.path.isdir('/usr/local/lib'):
        parts.append('/usr/local/lib')
    try:
        slam_prefix = get_package_prefix('autonomy_slam')
        for lib_dir in (
            os.path.join(slam_prefix, 'lib'),
            os.path.join(os.path.dirname(slam_prefix), 'autonomy', 'lib'),
        ):
            if os.path.isdir(lib_dir):
                parts.append(lib_dir)
    except Exception:
        pass
    existing = os.environ.get('LD_LIBRARY_PATH', '')
    if existing:
        parts.append(existing)
    return ':'.join(parts)


def _topic_remappings(mode: str, context) -> list:
    if mode == 'mono':
        topic = LaunchConfiguration('image_topic').perform(context)
        return [('camera/image_raw', topic)]
    if mode == 'stereo':
        return [
            ('camera/left/image_raw', LaunchConfiguration('left_topic').perform(context)),
            ('camera/right/image_raw', LaunchConfiguration('right_topic').perform(context)),
        ]
    if mode == 'rgbd':
        return [
            ('camera/color/image_raw', LaunchConfiguration('color_topic').perform(context)),
            ('camera/depth/image_raw', LaunchConfiguration('depth_topic').perform(context)),
        ]
    raise ValueError(f'Unsupported mode: {mode}')


def _slam_arguments(context) -> list:
    pkg_share = get_package_share_directory('autonomy_slam')
    mode = LaunchConfiguration('mode').perform(context)

    vocab_file = _resolve_path(
        LaunchConfiguration('vocab_file').perform(context),
        os.path.join(pkg_share, 'vocab', 'orb_vocab.fbow'),
    )
    atlas_config = _resolve_path(
        LaunchConfiguration('atlas_config').perform(context),
        os.path.join(pkg_share, 'config', 'atlas', f'{mode}.yaml'),
    )

    args = ['--vocab', vocab_file, '--config', atlas_config]

    map_db_in = LaunchConfiguration('map_db_in').perform(context)
    map_db_out = LaunchConfiguration('map_db_out').perform(context)
    eval_log_dir = LaunchConfiguration('eval_log_dir').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    mask = LaunchConfiguration('mask').perform(context)

    if map_db_in:
        args += ['--map-db-in', map_db_in]
    if map_db_out:
        args += ['--map-db-out', map_db_out]
    if eval_log_dir:
        args += ['--eval-log-dir', eval_log_dir]
    if log_level:
        args += ['--log-level', log_level]
    if mask:
        args += ['--mask', mask]
    if LaunchConfiguration('disable_mapping').perform(context) == 'true':
        args.append('--disable-mapping')
    if LaunchConfiguration('temporal_mapping').perform(context) == 'true':
        args.append('--temporal-mapping')
    if mode == 'stereo' and LaunchConfiguration('rectify').perform(context) == 'true':
        args.append('--rectify')

    use_offline = LaunchConfiguration('use_offline').perform(context) == 'true'
    if use_offline:
        bag_path = LaunchConfiguration('bag_path').perform(context)
        if not bag_path:
            raise RuntimeError('use_offline:=true requires bag_path')
        args += ['--bag', bag_path]
        if LaunchConfiguration('no_sleep').perform(context) == 'true':
            args.append('--no-sleep')
        start_offset = LaunchConfiguration('start_offset').perform(context)
        if start_offset and start_offset != '0.0':
            args += ['--start-offset', start_offset]
        storage_id = LaunchConfiguration('bag_storage_id').perform(context)
        if storage_id:
            args += ['--storage-id', storage_id]
        if mode == 'mono':
            args += ['--camera', LaunchConfiguration('image_topic').perform(context)]
        elif mode == 'stereo':
            args += [
                '--left', LaunchConfiguration('left_topic').perform(context),
                '--right', LaunchConfiguration('right_topic').perform(context),
            ]
        elif mode == 'rgbd':
            args += [
                '--color', LaunchConfiguration('color_topic').perform(context),
                '--depth', LaunchConfiguration('depth_topic').perform(context),
            ]

    return args, use_offline


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('autonomy_slam')
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('mono', 'stereo', 'rgbd'):
        raise RuntimeError(f'mode must be mono, stereo, or rgbd (got {mode})')

    slam_args, use_offline = _slam_arguments(context)
    ros_config = _resolve_path(
        LaunchConfiguration('ros_config').perform(context),
        os.path.join(pkg_share, 'config', f'{mode}.yaml'),
    )
    executable = 'run_slam_offline' if use_offline else 'run_slam'

    nodes = [
        Node(
            package='autonomy_slam',
            executable=executable,
            name='run_slam',
            output='screen',
            arguments=slam_args,
            parameters=[ros_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=_topic_remappings(mode, context),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'autonomy_slam.rviz')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
        ),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('LD_LIBRARY_PATH', _ld_library_path()),
        DeclareLaunchArgument(
            'mode', default_value='mono',
            description='SLAM mode: mono, stereo, or rgbd'),
        DeclareLaunchArgument(
            'vocab_file', default_value='',
            description='ORB vocabulary path (default: share/autonomy_slam/vocab/orb_vocab.fbow)'),
        DeclareLaunchArgument(
            'atlas_config', default_value='',
            description='Atlas SLAM YAML (default: share/autonomy_slam/config/atlas/<mode>.yaml)'),
        DeclareLaunchArgument(
            'ros_config', default_value='',
            description='ROS node YAML (default: share/autonomy_slam/config/<mode>.yaml)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('map_db_in', default_value=''),
        DeclareLaunchArgument('map_db_out', default_value=''),
        DeclareLaunchArgument('eval_log_dir', default_value=''),
        DeclareLaunchArgument('mask', default_value=''),
        DeclareLaunchArgument('disable_mapping', default_value='false'),
        DeclareLaunchArgument('temporal_mapping', default_value='false'),
        DeclareLaunchArgument('rectify', default_value='false',
                              description='Stereo: rectify raw images before tracking'),
        # Topics
        DeclareLaunchArgument('image_topic', default_value='camera/image_raw',
                              description='Mono image topic'),
        DeclareLaunchArgument('left_topic', default_value='camera/left/image_raw'),
        DeclareLaunchArgument('right_topic', default_value='camera/right/image_raw'),
        DeclareLaunchArgument('color_topic', default_value='camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='camera/depth/image_raw'),
        # Offline rosbag2
        DeclareLaunchArgument('use_offline', default_value='false',
                              description='Use run_slam_offline with rosbag2'),
        DeclareLaunchArgument('bag_path', default_value=''),
        DeclareLaunchArgument('bag_storage_id', default_value='sqlite3'),
        DeclareLaunchArgument('start_offset', default_value='0.0'),
        DeclareLaunchArgument('no_sleep', default_value='false'),
        OpaqueFunction(function=_launch_setup),
    ])
