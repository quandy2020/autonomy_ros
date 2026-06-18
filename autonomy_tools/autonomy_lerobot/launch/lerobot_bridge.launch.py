# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Launch the LeRobot bridge node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('autonomy_lerobot'),
                'config',
                'lerobot_bridge.yaml',
            ]),
            description='LeRobot bridge parameter file',
        ),
        Node(
            package='autonomy_lerobot',
            executable='lerobot_bridge_node',
            output='screen',
            parameters=[config_file],
        ),
    ])
