# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Habitat bridge + namespaced Nav2 for one robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch_robot(context, *args, **kwargs):
    """Start habitat (optional delay) and Nav2 for one robot namespace."""
    autonomy_ros_share = get_package_share_directory('autonomy_ros')
    simulator_share = get_package_share_directory('autonomy_simulator')

    namespace = LaunchConfiguration('namespace').perform(context)
    use_namespace = LaunchConfiguration('use_namespace').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    autostart = LaunchConfiguration('autostart').perform(context)
    scene_data_path = LaunchConfiguration('scene_data_path').perform(context)
    occupancy_grid_rate_hz = LaunchConfiguration('occupancy_grid_rate_hz').perform(context)
    habitat_delay = float(LaunchConfiguration('habitat_startup_delay').perform(context))
    nav2_delay = float(LaunchConfiguration('nav2_startup_delay').perform(context))
    spawn_index = LaunchConfiguration('spawn_index').perform(context)
    spawn_count = LaunchConfiguration('spawn_count').perform(context)
    spawn_mode = LaunchConfiguration('spawn_mode').perform(context)

    nav2_launch = os.path.join(
        autonomy_ros_share, 'launch', 'nav2_namespaced.launch.py'
    )
    habitat_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulator_share, 'launch', 'habitat.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'scene_data_path': scene_data_path,
            'occupancy_grid_rate_hz': occupancy_grid_rate_hz,
            'use_sim_time': use_sim_time,
            'spawn_index': spawn_index,
            'spawn_count': spawn_count,
            'spawn_mode': spawn_mode,
        }.items(),
    )
    nav2_stack = TimerAction(
        period=nav2_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    'namespace': namespace,
                    'use_namespace': use_namespace,
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'autostart': autostart,
                }.items(),
            ),
        ],
    )

    if habitat_delay > 0.0:
        return [TimerAction(period=habitat_delay, actions=[habitat_launch]), nav2_stack]
    return [habitat_launch, nav2_stack]


def generate_launch_description() -> LaunchDescription:
    autonomy_ros_share = get_package_share_directory('autonomy_ros')

    default_params = os.path.join(
        autonomy_ros_share, 'config', 'nav2_habitat_params.yaml'
    )
    default_scene = '/workspace/autonomy/src/17DRP5sb8fy'

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='ROS namespace for this robot (e.g. robot1)',
        ),
        DeclareLaunchArgument(
            'use_namespace',
            default_value='false',
            description='Place Nav2 nodes under namespace',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Nav2 parameters file',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Habitat bridge uses wall clock; keep false',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Autostart Nav2 lifecycle nodes',
        ),
        DeclareLaunchArgument(
            'nav2_startup_delay',
            default_value='25.0',
            description='Seconds after this robot launch to start Nav2',
        ),
        DeclareLaunchArgument(
            'habitat_startup_delay',
            default_value='0.0',
            description='Seconds before starting Habitat (stagger multi-robot GPU load)',
        ),
        DeclareLaunchArgument(
            'scene_data_path',
            default_value=default_scene,
            description='MP3D scene directory for this robot',
        ),
        DeclareLaunchArgument(
            'occupancy_grid_rate_hz',
            default_value='0.0',
            description='Map publish rate; 0 = latched once',
        ),
        DeclareLaunchArgument(
            'spawn_mode',
            default_value='fixed',
            description='Agent spawn: dispersed | random | fixed',
        ),
        DeclareLaunchArgument(
            'spawn_index',
            default_value='0',
            description='0-based robot index for dispersed spawn slot',
        ),
        DeclareLaunchArgument(
            'spawn_count',
            default_value='1',
            description='Total robots sharing the dispersed spawn layout',
        ),
        OpaqueFunction(function=_launch_robot),
    ])
