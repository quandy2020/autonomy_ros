"""Run monocular SLAM on AIST Living Lab MP4 datasets.

- launch_rviz:=true  → video_publisher + run_slam + rviz2
- launch_rviz:=false → run_video_slam (stella_vslam tutorial CLI, no ROS viz)
https://stella-cv.readthedocs.io/en/latest/simple_tutorial.html
"""

import os
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
if _LAUNCH_DIR not in sys.path:
    sys.path.append(_LAUNCH_DIR)

from aist_paths import (  # noqa: E402
    get_aist_data_root,
    get_aist_video,
    get_default_atlas_config,
    get_default_vocab,
    ld_library_path,
)


def _task_flags(task: str, map_db_in: str, map_db_out: str, default_map: str):
    if task == 'mapping':
        return map_db_out or default_map, '', 'false', 'false'
    if task == 'localization':
        return '', map_db_in or default_map, 'true', 'false'
    if task == 'temporal_localization':
        return '', map_db_in or default_map, 'false', 'true'
    raise RuntimeError(
        f'Unknown task: {task} (use mapping, localization, temporal_localization)')


def _configure(context, *args, **kwargs):
    pkg_share = get_package_share_directory('autonomy_slam')
    pkg_prefix = get_package_prefix('autonomy_slam')
    task = LaunchConfiguration('task').perform(context)
    sequence = LaunchConfiguration('sequence').perform(context)
    data_root = LaunchConfiguration('data_root').perform(context) or get_aist_data_root()
    video_path = LaunchConfiguration('video_path').perform(context) or get_aist_video(sequence)
    launch_rviz = LaunchConfiguration('launch_rviz').perform(context) == 'true'
    logger = get_logger('aist_slam')
    logger.info(f'launch_rviz={launch_rviz}')

    map_db_in = LaunchConfiguration('map_db_in').perform(context)
    map_db_out = LaunchConfiguration('map_db_out').perform(context)
    default_map = os.path.join(data_root, 'aist_living_lab_1_map.msg')
    map_db_out, map_db_in, disable_mapping, temporal_mapping = _task_flags(
        task, map_db_in, map_db_out, default_map)

    atlas_config = LaunchConfiguration('atlas_config').perform(context) or get_default_atlas_config()
    vocab_file = LaunchConfiguration('vocab_file').perform(context) or get_default_vocab()
    frame_skip = LaunchConfiguration('frame_skip').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    image_topic = LaunchConfiguration('image_topic').perform(context)
    eval_log_dir = LaunchConfiguration('eval_log_dir').perform(context)

    actions = [SetEnvironmentVariable('LD_LIBRARY_PATH', ld_library_path())]

    if launch_rviz:
        logger.info('Starting video_publisher + run_slam + rviz2')
        ros_config = LaunchConfiguration('ros_config').perform(context) or os.path.join(
            pkg_share, 'config', 'aist_mono.yaml')
        realtime = LaunchConfiguration('realtime').perform(context)
        if realtime == '':
            realtime = 'false'

        video_node = Node(
            package='autonomy_slam',
            executable='video_publisher.py',
            name='video_publisher',
            output='screen',
            parameters=[{
                'video_path': video_path,
                'topic': image_topic,
                'frame_id': 'camera_frame',
                'fps': float(LaunchConfiguration('fps').perform(context)),
                'frame_skip': int(frame_skip),
                'realtime': realtime == 'true',
                'loop': LaunchConfiguration('loop').perform(context) == 'true',
                'use_backpressure': True,
                'frame_processed_topic': '/run_slam/frame_processed',
            }],
        )

        slam_args = {
            'mode': 'mono',
            'atlas_config': atlas_config,
            'ros_config': ros_config,
            'image_topic': image_topic,
            'launch_rviz': 'true',
            'map_db_in': map_db_in,
            'map_db_out': map_db_out,
            'disable_mapping': disable_mapping,
            'temporal_mapping': temporal_mapping,
            'log_level': log_level,
            'vocab_file': vocab_file,
        }
        if eval_log_dir:
            slam_args['eval_log_dir'] = eval_log_dir

        slam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'slam.launch.py')),
            launch_arguments=slam_args.items(),
        )
        actions.extend([video_node, slam_launch])
        return actions

    logger.info('Starting run_video_slam (no rviz)')
    no_sleep = LaunchConfiguration('no_sleep').perform(context) == 'true'
    cmd = [
        os.path.join(pkg_prefix, 'lib', 'autonomy_slam', 'run_video_slam'),
        '-v', vocab_file,
        '-m', video_path,
        '-c', atlas_config,
        '--frame-skip', frame_skip,
        '--log-level', log_level,
    ]
    if no_sleep:
        cmd.append('--no-sleep')
    if map_db_in:
        cmd += ['--map-db-in', map_db_in]
    if map_db_out:
        cmd += ['--map-db-out', map_db_out]
    if disable_mapping == 'true':
        cmd.append('--disable-mapping')
    if temporal_mapping == 'true':
        cmd.append('--temporal-mapping')
    if eval_log_dir:
        cmd += ['--eval-log-dir', eval_log_dir]

    actions.append(ExecuteProcess(cmd=cmd, output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'task', default_value='mapping',
            description='mapping | localization | temporal_localization'),
        DeclareLaunchArgument(
            'sequence', default_value='aist_living_lab_1',
            description='aist_living_lab_1 or aist_living_lab_2'),
        DeclareLaunchArgument('data_root', default_value=''),
        DeclareLaunchArgument('video_path', default_value=''),
        DeclareLaunchArgument('image_topic', default_value='camera/image_raw'),
        DeclareLaunchArgument('frame_skip', default_value='3'),
        DeclareLaunchArgument('fps', default_value='30.0'),
        DeclareLaunchArgument(
            'launch_rviz', default_value='true',
            description='true: video_publisher + run_slam + rviz2; false: run_video_slam CLI'),
        DeclareLaunchArgument(
            'realtime', default_value='',
            description='Video publish rate when launch_rviz:=true (default: false, process every frame)'),
        DeclareLaunchArgument('loop', default_value='false'),
        DeclareLaunchArgument(
            'no_sleep', default_value='true',
            description='run_video_slam only: process frames as fast as possible'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('vocab_file', default_value=''),
        DeclareLaunchArgument('atlas_config', default_value=''),
        DeclareLaunchArgument('ros_config', default_value=''),
        DeclareLaunchArgument('map_db_in', default_value=''),
        DeclareLaunchArgument('map_db_out', default_value=''),
        DeclareLaunchArgument('eval_log_dir', default_value=''),
        OpaqueFunction(function=_configure),
    ])
