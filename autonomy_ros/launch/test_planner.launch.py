# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0


# ros2 launch autonomy_ros test_planner.launch.py \
#   simulation_mode:=fake \
#   use_sim_time:=true \
#   configuration_directory:=/workspace/autonomy/src/autonomy/config \
#   map_yaml:=/workspace/autonomy/src/autonomy/config/data/map.yaml \
#   planner_id:=theta_star_planner \
#   use_rviz:=false

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
    default_core_config = _resolve_core_config_directory()
    if not default_core_config:
        raise RuntimeError(
            'Could not resolve autonomy core config directory. '
            'Build and source install/setup.bash, or pass configuration_directory:=<path>.'
        )

    configuration_directory = LaunchConfiguration('configuration_directory')
    simulation_mode = LaunchConfiguration('simulation_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map_yaml')
    planner_id = LaunchConfiguration('planner_id')
    use_rviz = LaunchConfiguration('use_rviz')
    simulator_share = get_package_share_directory('autonomy_simulator')

    return LaunchDescription([
        DeclareLaunchArgument(
            'configuration_directory',
            default_value=default_core_config,
            description='Directory containing planner/planner.lua'),
        DeclareLaunchArgument(
            'simulation_mode',
            default_value='fake',
            description="Robot backend: 'gazebo' or 'fake'"),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'map_yaml',
            default_value='data/Sydney_2_1024.yaml',
            description='Map YAML (relative to config dir, or absolute path; empty -> data/map.yaml)'),
        DeclareLaunchArgument(
            'planner_id',
            default_value='',
            description='Planner plugin id (empty -> planner.lua default_planner_id)'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(simulator_share, 'launch', 'simulator.launch.py')),
            launch_arguments={
                'sim_mode': simulation_mode,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_base_link_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='autonomy_ros',
            executable='test_planner',
            name='test_planner',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'configuration_directory': configuration_directory,
                'map_yaml': map_yaml,
                'planner_id': planner_id,
            }],
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
