# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0

"""Start hunav_agent_manager + BT driver for Pure RViz (RViz already running via HuNav panel)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scenario_yaml = LaunchConfiguration('scenario_yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    update_hz = LaunchConfiguration('update_hz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario_yaml',
            description='Absolute path to HuNav scenario YAML (hunav_loader format)',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('robot_x', default_value='0.0'),
        DeclareLaunchArgument('robot_y', default_value='0.0'),
        DeclareLaunchArgument('update_hz', default_value='10.0'),
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
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='hunav_rviz2_panel',
            executable='hunav_sim_driver.py',
            name='hunav_sim_driver',
            output='screen',
            parameters=[
                {'scenario_yaml': scenario_yaml},
                {'use_sim_time': use_sim_time},
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
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='autonomy_pedestrian',
            executable='pedestrian_visualizer.py',
            name='pedestrian_visualizer',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'input_topic': '/hunav_agents_bridge/pedestrians'},
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
    ])
