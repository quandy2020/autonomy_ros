# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0


# ros2 launch autonomy_ros test_controller.launch.py \
#   path_shape:=circle \
#   path_center_x:=0.0 path_center_y:=0.0 path_radius:=2.0 \
#   controller_id:=mppi_controller \
#   use_rviz:=true
#
# fake 模式请保持 use_sim_time:=false（默认）；Gazebo 再用 use_sim_time:=true。
#
# Shapes: circle, rectangle, figure_eight (aliases: rect, figure8, eight)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _resolve_core_config_directory() -> str:
    try:
        return os.path.join(get_package_share_directory('autonomy'), 'config')
    except Exception:
        pass
    ros_share = get_package_share_directory('autonomy_ros')
    install_root = os.path.dirname(os.path.dirname(os.path.dirname(ros_share)))
    candidate = os.path.join(install_root, 'autonomy', 'share', 'autonomy', 'config')
    if os.path.isdir(candidate):
        return candidate
    return ''


def generate_launch_description():
    package_share = get_package_share_directory('autonomy_ros')
    default_core_config = _resolve_core_config_directory()
    if not default_core_config:
        raise RuntimeError(
            'Could not resolve autonomy core config directory. '
            'Build and source install/setup.bash, or pass configuration_directory:=<path>.'
        )

    configuration_directory = LaunchConfiguration('configuration_directory')
    simulation_mode = LaunchConfiguration('simulation_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    controller_id = LaunchConfiguration('controller_id')
    goal_checker_id = LaunchConfiguration('goal_checker_id')
    progress_checker_id = LaunchConfiguration('progress_checker_id')
    path_shape = LaunchConfiguration('path_shape')
    path_center_x = LaunchConfiguration('path_center_x')
    path_center_y = LaunchConfiguration('path_center_y')
    path_radius = LaunchConfiguration('path_radius')
    path_width = LaunchConfiguration('path_width')
    path_height = LaunchConfiguration('path_height')
    figure_eight_scale = LaunchConfiguration('figure_eight_scale')
    path_pose_spacing = LaunchConfiguration('path_pose_spacing')
    costmap_margin = LaunchConfiguration('costmap_margin')
    auto_start = LaunchConfiguration('auto_start')
    snap_robot_to_path_start = LaunchConfiguration('snap_robot_to_path_start')
    repeat_path = LaunchConfiguration('repeat_path')
    publish_mppi_trajectories = LaunchConfiguration('publish_mppi_trajectories')
    mppi_trajectory_step = LaunchConfiguration('mppi_trajectory_step')
    mppi_time_step = LaunchConfiguration('mppi_time_step')
    use_rviz = LaunchConfiguration('use_rviz')
    simulator_share = get_package_share_directory('autonomy_simulator')

    return LaunchDescription([
        DeclareLaunchArgument(
            'configuration_directory',
            default_value=default_core_config,
            description='Directory containing control/controller.lua'),
        DeclareLaunchArgument(
            'simulation_mode',
            default_value='fake',
            description="Robot backend: 'gazebo' or 'fake'"),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description=(
                'Use /clock simulation time. Must be false for fake robot '
                '(no /clock publisher); use true with Gazebo.')),
        DeclareLaunchArgument(
            'controller_id',
            default_value='mppi_controller',
            description='Controller plugin id (default: mppi_controller)'),
        DeclareLaunchArgument(
            'goal_checker_id',
            default_value='goal_checker',
            description='Goal checker plugin id'),
        DeclareLaunchArgument(
            'progress_checker_id',
            default_value='progress_checker',
            description='Progress checker plugin id'),
        DeclareLaunchArgument(
            'path_shape',
            default_value='rectangle',
            description='Reference path: circle, rectangle, figure_eight'),
        DeclareLaunchArgument(
            'path_center_x',
            default_value='0.0',
            description='Path center x [m] in odom frame'),
        DeclareLaunchArgument(
            'path_center_y',
            default_value='0.0',
            description='Path center y [m] in odom frame'),
        DeclareLaunchArgument(
            'path_radius',
            default_value='2.0',
            description='Circle radius [m]'),
        DeclareLaunchArgument(
            'path_width',
            default_value='4.0',
            description='Rectangle width [m]'),
        DeclareLaunchArgument(
            'path_height',
            default_value='3.0',
            description='Rectangle height [m]'),
        DeclareLaunchArgument(
            'figure_eight_scale',
            default_value='2.0',
            description='Figure-eight scale a (x=a*sin(t), y=a*sin(t)*cos(t))'),
        DeclareLaunchArgument(
            'path_pose_spacing',
            default_value='0.05',
            description='Max spacing between path poses [m]'),
        DeclareLaunchArgument(
            'costmap_margin',
            default_value='3.0',
            description='Extra margin around path for synthetic costmap [m]'),
        DeclareLaunchArgument(
            'auto_start',
            default_value='true',
            description='Start following generated path on launch'),
        DeclareLaunchArgument(
            'snap_robot_to_path_start',
            default_value='true',
            description='Teleport fake_robot to the first path pose before follow'),
        DeclareLaunchArgument(
            'repeat_path',
            default_value='true',
            description='After reaching the path goal, snap to start and follow again'),
        DeclareLaunchArgument(
            'publish_mppi_trajectories',
            default_value='true',
            description='Publish MPPI candidate/optimal trajectory markers (mppi_controller only)'),
        DeclareLaunchArgument(
            'mppi_trajectory_step',
            default_value='5',
            description='Subsample every N-th MPPI rollout for visualization'),
        DeclareLaunchArgument(
            'mppi_time_step',
            default_value='3',
            description='Subsample every N-th time index on each rollout'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(simulator_share, 'launch', 'simulator.launch.py')),
            launch_arguments={
                'sim_mode': simulation_mode,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        Node(
            package='autonomy_ros',
            executable='test_controller',
            name='test_controller',
            output='screen',
            remappings=[
                ('cmd_vel', '/cmd_vel'),
                ('odom', '/odom'),
            ],
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'configuration_directory': configuration_directory,
                'frame_id': 'odom',
                'controller_id': controller_id,
                'goal_checker_id': goal_checker_id,
                'progress_checker_id': progress_checker_id,
                'path_shape': path_shape,
                'path_center_x': ParameterValue(path_center_x, value_type=float),
                'path_center_y': ParameterValue(path_center_y, value_type=float),
                'path_radius': ParameterValue(path_radius, value_type=float),
                'path_width': ParameterValue(path_width, value_type=float),
                'path_height': ParameterValue(path_height, value_type=float),
                'figure_eight_scale': ParameterValue(figure_eight_scale, value_type=float),
                'path_pose_spacing': ParameterValue(path_pose_spacing, value_type=float),
                'costmap_margin': ParameterValue(costmap_margin, value_type=float),
                'auto_start': ParameterValue(auto_start, value_type=bool),
                'snap_robot_to_path_start': ParameterValue(
                    snap_robot_to_path_start, value_type=bool),
                'repeat_path': ParameterValue(repeat_path, value_type=bool),
                'publish_mppi_trajectories': ParameterValue(
                    publish_mppi_trajectories, value_type=bool),
                'mppi_trajectory_step': ParameterValue(
                    mppi_trajectory_step, value_type=int),
                'mppi_time_step': ParameterValue(mppi_time_step, value_type=int),
            }],
        ),
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(package_share, 'rviz', 'autonomy.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
