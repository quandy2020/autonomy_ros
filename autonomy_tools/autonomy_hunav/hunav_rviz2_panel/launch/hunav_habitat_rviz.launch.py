# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0

"""Habitat /map bridge + RViz with HuNav panel (load YAML + Run HuNav Simulation)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from autonomy_lerobot.data_paths import default_mp3d_scene
from launch_ros.actions import Node

DEFAULT_MP3D_SCENE = str(default_mp3d_scene())


def generate_launch_description():
    simulator_share = get_package_share_directory('autonomy_simulator')
    default_rviz = os.path.join(simulator_share, 'rviz', 'habitat.rviz')

    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument(
            'scene_data_path',
            default_value=DEFAULT_MP3D_SCENE,
            description='MP3D scene directory',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(simulator_share, 'launch', 'habitat.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'scene_data_path': LaunchConfiguration('scene_data_path'),
            }.items(),
        ),
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
