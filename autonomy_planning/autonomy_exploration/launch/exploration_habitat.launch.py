# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0

"""Launch Habitat simulation + exploration_node + RViz2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('autonomy_exploration')
    sim_pkg = get_package_share_directory('autonomy_simulator')

    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_cmd_vel = LaunchConfiguration('enable_cmd_vel')

    declares = [
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 with exploration.rviz',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Habitat uses wall clock; keep false',
        ),
        DeclareLaunchArgument(
            'enable_cmd_vel',
            default_value='true',
            description='Simple P-controller toward exploration waypoint',
        ),
        DeclareLaunchArgument(
            'topdown_enabled',
            default_value='false',
            description='Habitat topdown overview image (extra render pass; default off)',
        ),
        DeclareLaunchArgument(
            'topdown_mode',
            default_value='room',
            description='Habitat topdown mode',
        ),
    ]

    # Scoped group: habitat's use_rviz:=false must not override this launch's
    # use_rviz (otherwise exploration RViz never starts).
    habitat = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sim_pkg, 'launch', 'habitat.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'use_rviz': 'false',
                    'topdown_enabled': LaunchConfiguration('topdown_enabled'),
                    'topdown_mode': LaunchConfiguration('topdown_mode'),
                }.items(),
            ),
        ],
    )

    exploration_params = os.path.join(pkg, 'config', 'exploration_params.yaml')
    exploration_node = Node(
        package='autonomy_exploration',
        executable='exploration_node',
        name='exploration_node',
        output='screen',
        parameters=[
            exploration_params,
            {
                'use_sim_time': use_sim_time,
                'enable_cmd_vel': enable_cmd_vel,
            },
        ],
    )

    rviz_config = os.path.join(pkg, 'rviz', 'exploration.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        *declares,
        habitat,
        exploration_node,
        rviz_node,
    ])
