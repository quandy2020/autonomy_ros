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
        description='Use simulation clock (Gazebo only; fake/habitat always use wall clock)',
    )

    is_gazebo = IfCondition(PythonExpression(["'", sim_mode, "' == 'gazebo'"]))
    is_habitat = IfCondition(PythonExpression(["'", sim_mode, "' == 'habitat'"]))

    declare_pedestrians_enabled = DeclareLaunchArgument(
        'pedestrians_enabled',
        default_value='false',
        description='Habitat only: enable dynamic pedestrian obstacles',
    )
    declare_pedestrian_count = DeclareLaunchArgument(
        'pedestrian_count',
        default_value='5',
        description='Habitat only: number of URDF humanoid pedestrians',
    )
    declare_trackvla_root = DeclareLaunchArgument(
        'trackvla_root',
        default_value='',
        description='Habitat only: TrackVLA root for humanoid assets',
    )
    declare_topdown_enabled = DeclareLaunchArgument(
        'topdown_enabled',
        default_value='false',
        description='Habitat only: publish overhead RGB on camera/topdown/image_raw',
    )
    declare_topdown_mode = DeclareLaunchArgument(
        'topdown_mode',
        default_value='room',
        description='Habitat only: room | oblique | overhead | interactive',
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Habitat only: launch rviz2 with habitat.rviz',
    )

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
            'use_sim_time': 'false',
            'publish_odom': 'true',
            'publish_tf': 'true',
        }.items(),
        condition=IfCondition(PythonExpression(["'", sim_mode, "' == 'fake'"])),
    )

    habitat = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'habitat.launch.py')
        ),
        # Habitat bridge uses wall clock; no /clock publisher. use_sim_time=true stalls timers.
        launch_arguments={
            'use_sim_time': 'false',
            'pedestrians_enabled': LaunchConfiguration('pedestrians_enabled'),
            'pedestrian_count': LaunchConfiguration('pedestrian_count'),
            'trackvla_root': LaunchConfiguration('trackvla_root'),
            'topdown_enabled': LaunchConfiguration('topdown_enabled'),
            'topdown_mode': LaunchConfiguration('topdown_mode'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
        condition=is_habitat,
    )

    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        declare_sim_mode,
        declare_use_sim_time,
        declare_pedestrians_enabled,
        declare_pedestrian_count,
        declare_trackvla_root,
        declare_topdown_enabled,
        declare_topdown_mode,
        declare_use_rviz,
        gazebo,
        fake_robot_fake,
        habitat,
    ])
