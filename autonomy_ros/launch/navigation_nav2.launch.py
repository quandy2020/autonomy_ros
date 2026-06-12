# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Habitat MP3D simulation + Nav2 navigation (map/odom/TF from habitat_bridge)."""

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
    simulator_share = get_package_share_directory('autonomy_simulator')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    autostart = LaunchConfiguration('autostart')

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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(simulator_share, 'launch', 'fake_robot.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'publish_odom': 'false',
                'publish_tf': 'false',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(simulator_share, 'launch', 'habitat.launch.py')
            ),
            launch_arguments={
                'cmd_vel_topic': 'cmd_vel_stamped',
                'occupancy_grid_rate_hz': '0.0',
            }.items(),
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            remappings=[('cloud_in', '/semantic_pointcloud')],
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': 'base_footprint',
                'transform_tolerance': 0.3,
                'min_height': 0.05,
                'max_height': 1.5,
                'angle_min': -3.14159265,
                'angle_max': 3.14159265,
                'angle_increment': 0.00872665,
                'scan_time': 0.1,
                'range_min': 0.12,
                'range_max': 8.0,
            }],
        ),
        Node(
            package='autonomy_ros',
            executable='cmd_vel_twist_stamped_relay.py',
            name='cmd_vel_twist_stamped_relay',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': 'cmd_vel',
                'output_topic': 'cmd_vel_stamped',
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': autostart,
                'use_composition': 'False',
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
