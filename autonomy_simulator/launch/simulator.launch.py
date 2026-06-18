# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0

"""Launch robot simulation backend: Gazebo, fake diff-drive, or Habitat."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    pkg_share = get_package_share_directory('autonomy_simulator')

    sim_mode = LaunchConfiguration('sim_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_sim_mode = DeclareLaunchArgument(
        'sim_mode',
        default_value='gazebo',
        description="Simulation backend: 'gazebo', 'fake', or 'habitat'")
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock (true for Gazebo; false for fake/habitat)')

    is_gazebo = IfCondition(PythonExpression(["'", sim_mode, "' == 'gazebo'"]))
    is_habitat = IfCondition(PythonExpression(["'", sim_mode, "' == 'habitat'"]))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'tb3_simulator.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=is_gazebo,
    )

    fake_robot_fake = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'fake_robot.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'publish_odom': 'true',
            'publish_tf': 'true',
        }.items(),
        condition=IfCondition(PythonExpression(["'", sim_mode, "' == 'fake'"])),
    )

    habitat = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'habitat.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=is_habitat,
    )

    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        declare_sim_mode,
        declare_use_sim_time,
        gazebo,
        fake_robot_fake,
        habitat,
    ])
