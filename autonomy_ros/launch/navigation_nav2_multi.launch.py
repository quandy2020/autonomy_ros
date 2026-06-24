# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Launch up to N independent Habitat + Nav2 stacks (default 10 robots).

Each robot runs in its own ROS namespace (``robot1`` … ``robotN``) with isolated
topics, services, actions, and TF (``/robotK/tf``). Nav2 stacks start with a
staggered delay to reduce startup contention.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_SCENE = '/workspace/autonomy/src/17DRP5sb8fy'
DEFAULT_NUM_ROBOTS = 10


def _robot_configs(
    num_robots: int,
    name_prefix: str,
    scene: str,
    delay_base: float,
    delay_step: float,
    habitat_step: float,
) -> list[dict[str, str]]:
    return [
        {
            'name': f'{name_prefix}{index}',
            'scene': scene,
            'nav2_startup_delay': f'{delay_base + (index - 1) * delay_step:.1f}',
            'habitat_startup_delay': f'{(index - 1) * habitat_step:.1f}',
            'spawn_index': str(index - 1),
            'spawn_count': str(num_robots),
        }
        for index in range(1, num_robots + 1)
    ]


def _launch_robot_stacks(context, *args, **kwargs):
    autonomy_ros_share = get_package_share_directory('autonomy_ros')

    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    if num_robots < 1:
        raise RuntimeError(f'num_robots must be >= 1, got {num_robots}')

    name_prefix = LaunchConfiguration('robot_name_prefix').perform(context)
    scene = LaunchConfiguration('scene_data_path').perform(context)
    delay_base = float(LaunchConfiguration('nav2_startup_delay_base').perform(context))
    delay_step = float(LaunchConfiguration('nav2_startup_delay_step').perform(context))
    habitat_step = float(LaunchConfiguration('habitat_startup_delay_step').perform(context))

    params_file = LaunchConfiguration('params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    autostart = LaunchConfiguration('autostart').perform(context)

    robot_launch = os.path.join(
        autonomy_ros_share, 'launch', 'navigation_nav2_robot.launch.py'
    )

    actions = []
    for robot in _robot_configs(
        num_robots, name_prefix, scene, delay_base, delay_step, habitat_step
    ):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_launch),
                launch_arguments={
                    'namespace': robot['name'],
                    'use_namespace': 'true',
                    'scene_data_path': robot['scene'],
                    'params_file': params_file,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'nav2_startup_delay': robot['nav2_startup_delay'],
                    'habitat_startup_delay': robot['habitat_startup_delay'],
                    'occupancy_grid_rate_hz': '0.0',
                    'spawn_index': robot['spawn_index'],
                    'spawn_count': robot['spawn_count'],
                    'spawn_mode': 'fixed',
                }.items(),
            ),
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    autonomy_ros_share = get_package_share_directory('autonomy_ros')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    params_file = LaunchConfiguration('params_file')

    default_params = os.path.join(
        autonomy_ros_share, 'config', 'nav2_habitat_params.yaml'
    )
    default_rviz = os.path.join(
        autonomy_ros_share, 'rviz', 'nav2_multi_habitat.rviz'
    )

    declares = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Nav2 parameters shared by all robots',
        ),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'num_robots',
            default_value=str(DEFAULT_NUM_ROBOTS),
            description='Number of independent robot stacks (robot1 … robotN)',
        ),
        DeclareLaunchArgument(
            'robot_name_prefix',
            default_value='robot',
            description='Namespace prefix; robot i is {prefix}{i}',
        ),
        DeclareLaunchArgument(
            'scene_data_path',
            default_value=DEFAULT_SCENE,
            description='MP3D scene directory for every robot',
        ),
        DeclareLaunchArgument(
            'nav2_startup_delay_base',
            default_value='8.0',
            description='Nav2 start delay for robot1 (seconds; allow Habitat Session load)',
        ),
        DeclareLaunchArgument(
            'nav2_startup_delay_step',
            default_value='5.0',
            description='Extra Nav2 delay per robot index (seconds)',
        ),
        DeclareLaunchArgument(
            'habitat_startup_delay_step',
            default_value='3.0',
            description='Extra Habitat delay per robot index (seconds)',
        ),
    ]

    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        *declares,
        OpaqueFunction(function=_launch_robot_stacks),
        rviz,
    ])
