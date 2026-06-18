# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0

"""Start autonomy_pedestrian for Pure RViz mode (RViz already running via HuNav panel)."""

import os
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _converter_path() -> str:
    prefix = get_package_prefix('hunav_rviz2_panel')
    return os.path.join(prefix, 'lib', 'hunav_rviz2_panel', 'hunav_yaml_to_pedestrian_xml.py')


def _prepare(context, *args, **kwargs):
    scenario_yaml = LaunchConfiguration('scenario_yaml').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    auto_start = LaunchConfiguration('auto_start')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')

    pedestrian_share = get_package_share_directory('autonomy_pedestrian')
    config_file = os.path.join(pedestrian_share, 'config', 'ros2_configuration.yaml')
    yaml_path = Path(scenario_yaml).resolve()
    xml_path = yaml_path.with_suffix('.xml')
    subprocess.check_call([_converter_path(), str(yaml_path), str(xml_path)])

    return [
        Node(
            package='autonomy_pedestrian',
            executable='pedestrian_simulator_node',
            name='pedestrian_simulator',
            output='screen',
            parameters=[
                config_file,
                {
                    'use_sim_time': use_sim_time,
                    'auto_start': auto_start,
                    'pedestrian_simulator.node.scenario': str(xml_path),
                    'pedestrian_simulator.node.scenario_package_name': 'autonomy_pedestrian',
                },
            ],
        ),
        Node(
            package='autonomy_pedestrian',
            executable='pedestrian_visualizer.py',
            name='pedestrian_visualizer',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='autonomy_pedestrian',
            executable='pedestrian_demo.py',
            name='pedestrian_demo',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_x': robot_x},
                {'robot_y': robot_y},
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario_yaml',
            description='Absolute path to HuNav scenario YAML (hunav_loader format)',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('robot_x', default_value='0.0'),
        DeclareLaunchArgument('robot_y', default_value='0.0'),
        OpaqueFunction(function=_prepare),
    ])
