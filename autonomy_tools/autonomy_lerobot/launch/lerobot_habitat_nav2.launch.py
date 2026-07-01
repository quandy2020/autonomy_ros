# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Habitat + Nav2 navigation with LeRobot dataset recording."""

import os

from ament_index_python.packages import get_package_share_directory
from autonomy_lerobot.data_paths import lerobot_habitat_nav2_root
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from autonomy_lerobot.launch_utils import lerobot_bridge_node


def generate_launch_description() -> LaunchDescription:
    autonomy_ros_share = get_package_share_directory('autonomy_ros')
    default_nav2_params = os.path.join(
        autonomy_ros_share, 'config', 'nav2_habitat_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=default_nav2_params),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('nav2_startup_delay', default_value='8.0'),
        DeclareLaunchArgument('lerobot_startup_delay', default_value='10.0'),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('autonomy_lerobot'),
                'config',
                'lerobot_bridge.yaml',
            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                autonomy_ros_share, 'launch', 'navigation_nav2.launch.py',
            )),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': LaunchConfiguration('params_file'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'autostart': LaunchConfiguration('autostart'),
                'nav2_startup_delay': LaunchConfiguration('nav2_startup_delay'),
            }.items(),
        ),
        TimerAction(
            period=LaunchConfiguration('lerobot_startup_delay'),
            actions=[
                lerobot_bridge_node(
                    output='screen',
                    parameters=[
                        LaunchConfiguration('config_file'),
                        {'dataset_root': str(lerobot_habitat_nav2_root())},
                    ],
                ),
            ],
        ),
    ])
