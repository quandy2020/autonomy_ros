# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0

"""TurtleBot3 fake robot (no Gazebo): diff-drive odom, joint_states, tf."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('autonomy_simulator')
    params_file = os.path.join(pkg_share, 'param', 'fake_robot_waffle.yaml')
    urdf_file = os.path.join(pkg_share, 'urdf', 'turtlebot3_waffle.urdf')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    with open(urdf_file, encoding='utf-8') as infp:
        robot_description = infp.read()

    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (false for fake robot)')

    fake_robot_node = Node(
        package='autonomy_simulator',
        executable='fake_robot_node',
        name='fake_robot_node',
        namespace=namespace,
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time, 'robot_description': robot_description}
        ],
    )

    return LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        fake_robot_node,
        robot_state_publisher,
    ])
