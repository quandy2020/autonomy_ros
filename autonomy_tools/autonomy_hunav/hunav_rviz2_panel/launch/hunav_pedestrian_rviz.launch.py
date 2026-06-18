# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0

"""Run a HuNav scenario YAML in RViz via autonomy_pedestrian or hunav_agent_manager."""

import os
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _converter_path() -> str:
    prefix = get_package_prefix('hunav_rviz2_panel')
    return os.path.join(prefix, 'lib', 'hunav_rviz2_panel', 'hunav_yaml_to_pedestrian_xml.py')


def _prepare_scenario(context, *args, **kwargs):
    scenario_yaml = LaunchConfiguration('scenario_yaml').perform(context)
    backend = LaunchConfiguration('simulation_backend').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    auto_start = LaunchConfiguration('auto_start')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    update_hz = LaunchConfiguration('update_hz')

    actions = []

    if backend == 'pedestrian':
        pedestrian_share = get_package_share_directory('autonomy_pedestrian')
        config_file = os.path.join(pedestrian_share, 'config', 'ros2_configuration.yaml')
        yaml_path = Path(scenario_yaml).resolve()
        xml_path = yaml_path.with_suffix('.xml')
        subprocess.check_call([_converter_path(), str(yaml_path), str(xml_path)])

        actions.extend([
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
                executable='pedestrian_demo.py',
                name='pedestrian_demo',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'robot_x': robot_x},
                    {'robot_y': robot_y},
                ],
            ),
            Node(
                package='autonomy_pedestrian',
                executable='pedestrian_visualizer.py',
                name='pedestrian_visualizer',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ])
    else:
        actions.extend([
            Node(
                package='hunav_agent_manager',
                executable='hunav_loader',
                name='hunav_loader',
                output='screen',
                parameters=[scenario_yaml],
            ),
            Node(
                package='hunav_agent_manager',
                executable='hunav_agent_manager',
                name='hunav_agent_manager',
                output='screen',
            ),
            Node(
                package='hunav_rviz2_panel',
                executable='hunav_sim_driver.py',
                name='hunav_sim_driver',
                output='screen',
                parameters=[
                    {'scenario_yaml': scenario_yaml},
                    {'update_hz': update_hz},
                    {'robot_x': robot_x},
                    {'robot_y': robot_y},
                ],
            ),
            Node(
                package='hunav_rviz2_panel',
                executable='hunav_agents_bridge.py',
                name='hunav_agents_bridge',
                output='screen',
            ),
            Node(
                package='autonomy_pedestrian',
                executable='pedestrian_visualizer.py',
                name='pedestrian_visualizer',
                output='screen',
                parameters=[
                    {'input_topic': '/hunav_agents_bridge/pedestrians'},
                ],
            ),
            Node(
                package='autonomy_pedestrian',
                executable='pedestrian_demo.py',
                name='pedestrian_demo',
                output='screen',
                parameters=[
                    {'robot_x': robot_x},
                    {'robot_y': robot_y},
                ],
            ),
        ])

    actions.append(
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        )
    )
    return actions


def generate_launch_description():
    panel_share = get_package_share_directory('hunav_rviz2_panel')
    default_rviz = os.path.join(panel_share, 'launch', 'hunav_pedestrian.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario_yaml',
            description='Absolute path to HuNav scenario YAML (hunav_loader format)',
        ),
        DeclareLaunchArgument(
            'simulation_backend',
            default_value='pedestrian',
            description='pedestrian: autonomy_pedestrian XML sim; hunav: hunav_agent_manager SFM/BT',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument('robot_x', default_value='0.0'),
        DeclareLaunchArgument('robot_y', default_value='0.0'),
        DeclareLaunchArgument('update_hz', default_value='10.0'),
        OpaqueFunction(function=_prepare_scenario),
    ])
