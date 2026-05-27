# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_core_config_directory() -> str:
    try:
        return os.path.join(get_package_share_directory('autonomy'), 'config')
    except Exception:
        pass
    ros_share = get_package_share_directory('autonomy_ros')
    install_root = os.path.dirname(os.path.dirname(os.path.dirname(ros_share)))
    candidate = os.path.join(install_root, 'autonomy', 'share', 'autonomy', 'config')
    if os.path.isdir(candidate):
        return candidate
    return ''


def generate_launch_description():
    package_share = get_package_share_directory('autonomy_ros')
    parameters_file = os.path.join(package_share, 'config', 'parameters.yaml')
    default_core_config = _resolve_core_config_directory()
    if not default_core_config:
        raise RuntimeError(
            'Could not resolve autonomy core config directory. '
            'Build and source install/setup.bash, or pass core_config_directory:=<path>.'
        )

    simulation_mode = LaunchConfiguration('simulation_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    core_config_directory = LaunchConfiguration('core_config_directory')

    simulator_share = get_package_share_directory('autonomy_simulator')

    return LaunchDescription([
        DeclareLaunchArgument(
            'simulation_mode',
            default_value='gazebo',
            description="Robot backend: 'gazebo' or 'fake'"),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz2'),
        DeclareLaunchArgument(
            'core_config_directory',
            default_value=default_core_config,
            description='Directory containing autonomy.lua'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(simulator_share, 'launch', 'simulator.launch.py')),
            launch_arguments={
                'sim_mode': simulation_mode,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        Node(
            package='autonomy_ros',
            executable='autonomy_node',
            name='autonomy_node',
            output='screen',
            parameters=[
                parameters_file,
                {
                    'use_sim_time': use_sim_time,
                    'autonomy.config_directory': core_config_directory,
                },
            ],
        ),
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(package_share, 'rviz', 'autonomy.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
