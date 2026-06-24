# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Habitat MP3D simulation + Nav2 navigation (map/odom/TF from habitat_bridge).

Single-robot launch (default). For multiple namespaced robots use
``navigation_nav2_multi.launch.py``.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    autonomy_ros_share = get_package_share_directory('autonomy_ros')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    autostart = LaunchConfiguration('autostart')
    nav2_startup_delay = LaunchConfiguration('nav2_startup_delay')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    scene_data_path = LaunchConfiguration('scene_data_path')

    default_params = os.path.join(
        autonomy_ros_share, 'config', 'nav2_habitat_params.yaml'
    )
    default_rviz = os.path.join(
        nav2_bringup_share, 'rviz', 'nav2_default_view.rviz'
    )
    default_scene = '/workspace/autonomy/src/17DRP5sb8fy'

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Habitat bridge uses wall clock; keep false',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Nav2 parameters file',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 with Nav2 view',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Autostart Nav2 lifecycle nodes',
        ),
        DeclareLaunchArgument(
            'nav2_startup_delay',
            default_value='8.0',
            description='Seconds to wait for habitat_bridge TF and /map',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='ROS namespace (empty = global, backward compatible)',
        ),
        DeclareLaunchArgument(
            'use_namespace',
            default_value='false',
            description='Place Habitat + Nav2 under namespace',
        ),
        DeclareLaunchArgument(
            'scene_data_path',
            default_value=default_scene,
            description='MP3D scene directory',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    autonomy_ros_share, 'launch', 'navigation_nav2_robot.launch.py'
                )
            ),
            launch_arguments={
                'namespace': namespace,
                'use_namespace': use_namespace,
                'scene_data_path': scene_data_path,
                'params_file': params_file,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'nav2_startup_delay': nav2_startup_delay,
                'occupancy_grid_rate_hz': '0.0',
            }.items(),
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
