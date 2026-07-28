# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Autocharge demo entry point for the autonomy_tests metapackage."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    autocharge_demo_pkg = get_package_share_directory('autonomy_autocharge')

    return LaunchDescription([
        DeclareLaunchArgument(
            'charger_x',
            default_value='2.0',
            description='Charger position X in map frame (m)'),
        DeclareLaunchArgument(
            'charger_y',
            default_value='0.0',
            description='Charger position Y in map frame (m)'),
        DeclareLaunchArgument(
            'charger_yaw',
            default_value='0.0',
            description='Charger yaw in map frame (rad)'),
        DeclareLaunchArgument(
            'use_explicit_predock',
            default_value='true',
            description='Use predock_x/y/yaw instead of distance offset'),
        DeclareLaunchArgument(
            'predock_x',
            default_value='1.4',
            description='Predock X in map frame (m)'),
        DeclareLaunchArgument(
            'predock_y',
            default_value='-0.08',
            description='Predock Y in map frame (m)'),
        DeclareLaunchArgument(
            'predock_yaw',
            default_value='0.0',
            description='Predock yaw in map frame (rad)'),
        DeclareLaunchArgument(
            'predock_distance_m',
            default_value='0.8',
            description='Predock offset when use_explicit_predock=false (m)'),
        DeclareLaunchArgument(
            'demo_start_delay',
            default_value='25.0',
            description='Seconds to wait before starting Nav2 + dock demo'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz with autocharge demo view'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(autocharge_demo_pkg, 'launch', 'autocharge_demo.launch.py')
            ),
            launch_arguments={
                'charger_x': LaunchConfiguration('charger_x'),
                'charger_y': LaunchConfiguration('charger_y'),
                'charger_yaw': LaunchConfiguration('charger_yaw'),
                'use_explicit_predock': LaunchConfiguration('use_explicit_predock'),
                'predock_x': LaunchConfiguration('predock_x'),
                'predock_y': LaunchConfiguration('predock_y'),
                'predock_yaw': LaunchConfiguration('predock_yaw'),
                'predock_distance_m': LaunchConfiguration('predock_distance_m'),
                'demo_start_delay': LaunchConfiguration('demo_start_delay'),
                'use_rviz': LaunchConfiguration('use_rviz'),
            }.items(),
        ),
    ])
