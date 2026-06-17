# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Habitat MP3D simulation + Nav2 navigation (map/odom/TF from habitat_bridge)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    autonomy_ros_share = get_package_share_directory('autonomy_ros')
    simulator_share = get_package_share_directory('autonomy_simulator')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    autostart = LaunchConfiguration('autostart')
    nav2_startup_delay = LaunchConfiguration('nav2_startup_delay')

    default_params = os.path.join(
        autonomy_ros_share, 'config', 'nav2_habitat_params.yaml'
    )
    default_rviz = os.path.join(
        nav2_bringup_share, 'rviz', 'nav2_default_view.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Habitat bridge uses wall clock; keep false'),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Nav2 parameters file'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 with Nav2 view'),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Autostart Nav2 lifecycle nodes'),
        DeclareLaunchArgument(
            'nav2_startup_delay',
            default_value='8.0',
            description='Seconds to wait for habitat_bridge TF and /map'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(simulator_share, 'launch', 'habitat.launch.py')
            ),
            launch_arguments={
                'occupancy_grid_rate_hz': '0.0',
            }.items(),
        ),
        TimerAction(
            period=nav2_startup_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            nav2_bringup_share, 'launch', 'navigation_launch.py'
                        )
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'params_file': params_file,
                        'autostart': autostart,
                        'use_composition': 'False',
                    }.items(),
                ),
            ],
        ),
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', default_rviz],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
