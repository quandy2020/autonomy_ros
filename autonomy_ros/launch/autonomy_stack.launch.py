# Copyright (C) 2025 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('autonomy_ros')
    params_file = os.path.join(pkg_share, 'configs', 'autonomy_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    enable_autonomy = LaunchConfiguration('enable_autonomy')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Start RViz2')
    declare_enable_autonomy = DeclareLaunchArgument(
        'enable_autonomy', default_value='true',
        description='Start autonomy core (map/plan/control via autonomy_ros::Autonomy)')

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'tb3_simulator.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    autonomy_node = Node(
        package='autonomy_ros',
        executable='autonomy_node',
        name='autonomy_node',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'enable_autonomy': enable_autonomy,
            },
        ],
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'autonomy.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rviz,
        declare_enable_autonomy,
        simulator,
        autonomy_node,
        rviz,
    ])
