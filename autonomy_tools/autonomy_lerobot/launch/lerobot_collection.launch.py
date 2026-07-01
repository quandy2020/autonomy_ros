# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""LeRobot bridge for multi-robot collection (jdrobot / kujiale schema)."""

from autonomy_lerobot.collection_params import jdrobot_collection_parameters
from autonomy_lerobot.data_paths import lerobot_collection_root
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from autonomy_lerobot.launch_utils import lerobot_bridge_node


def generate_launch_description() -> LaunchDescription:
    robot_root = PathJoinSubstitution([
        str(lerobot_collection_root()),
        LaunchConfiguration('robot_id'),
    ])
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('autonomy_lerobot'),
                'config',
                'lerobot_collection.yaml',
            ]),
        ),
        DeclareLaunchArgument('robot_id', default_value='robot1'),
        lerobot_bridge_node(
            name='lerobot_bridge_node',
            namespace=LaunchConfiguration('robot_id'),
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                jdrobot_collection_parameters(),
                {'dataset_root': robot_root},
            ],
        ),
    ])
