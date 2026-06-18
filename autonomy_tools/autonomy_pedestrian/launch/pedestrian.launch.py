# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0

"""Launch pedestrian simulator demo (corridor scenario + optional RViz / teleop)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('autonomy_pedestrian')
    config_file = os.path.join(pkg_share, 'config', 'ros2_configuration.yaml')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'pedestrian.rviz')

    pedestrian_scenario = LaunchConfiguration('pedestrian_scenario')
    static = LaunchConfiguration('static')
    use_sim_time = LaunchConfiguration('use_sim_time')
    auto_start = LaunchConfiguration('auto_start')
    run_demo_helpers = LaunchConfiguration('run_demo_helpers')
    use_rviz = LaunchConfiguration('use_rviz')
    use_pedestrian_rviz_plugin = LaunchConfiguration('use_pedestrian_rviz_plugin')
    rviz_config = LaunchConfiguration('rviz_config')
    publish_pedestrian_tf = LaunchConfiguration('publish_pedestrian_tf')
    use_keyboard_teleop = LaunchConfiguration('use_keyboard_teleop')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')

    return LaunchDescription([
        DeclareLaunchArgument(
            'pedestrian_scenario',
            default_value='random_social/8_corridor.xml',
            description='Scenario XML under share/autonomy_pedestrian/scenarios/',
        ),
        DeclareLaunchArgument(
            'static',
            default_value='false',
            description='Freeze pedestrians at spawn pose',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'auto_start',
            default_value='true',
            description='Start simulation loop from config after node is ready',
        ),
        DeclareLaunchArgument(
            'run_demo_helpers',
            default_value='true',
            description='Publish static /robot_state for standalone demo',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz with the bundled pedestrian.rviz config',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Absolute path to RViz config file',
        ),
        DeclareLaunchArgument(
            'use_pedestrian_rviz_plugin',
            default_value='true',
            description='Run visualizer bridge and use pedsim_rviz_plugin TrackedPersons display',
        ),
        DeclareLaunchArgument(
            'publish_pedestrian_tf',
            default_value='false',
            description='Broadcast map -> pedestrian_<id> transforms',
        ),
        DeclareLaunchArgument(
            'use_keyboard_teleop',
            default_value='false',
            description='Use keyboard teleop instead of static robot pose publisher',
        ),
        DeclareLaunchArgument(
            'robot_x',
            default_value='-8.0',
            description='Demo robot x in map frame',
        ),
        DeclareLaunchArgument(
            'robot_y',
            default_value='0.0',
            description='Demo robot y in map frame',
        ),
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
                    'publish_pedestrian_tf': publish_pedestrian_tf,
                    'pedestrian_simulator.node.scenario': pedestrian_scenario,
                    'pedestrian_simulator.static': static,
                    'pedestrian_simulator.node.scenario_package_name': 'autonomy_pedestrian',
                },
            ],
        ),
        Node(
            condition=IfCondition(
                PythonExpression([
                    "'", run_demo_helpers, "' == 'true' and '",
                    use_keyboard_teleop, "' != 'true'",
                ])
            ),
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
            condition=IfCondition(use_pedestrian_rviz_plugin),
            package='autonomy_pedestrian',
            executable='pedestrian_visualizer.py',
            name='pedestrian_visualizer',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            condition=IfCondition(use_keyboard_teleop),
            package='autonomy_pedestrian',
            executable='keyboard_teleop.py',
            name='pedestrian_keyboard_teleop',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_x': robot_x},
                {'robot_y': robot_y},
            ],
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
