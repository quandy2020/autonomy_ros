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
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from autonomy_lerobot.data_paths import default_mp3d_scene
from launch_ros.actions import Node

DEFAULT_MP3D_SCENE = str(default_mp3d_scene())
DEFAULT_NUM_ROBOTS = 10


def _robot_configs(
    num_robots: int,
    name_prefix: str,
    scene: str,
    habitat_step: float,
    habitat_load: float,
    habitat_load_penalty: float,
    nav2_gap: float,
    nav2_step: float,
) -> list[dict[str, str]]:
    """Per-robot Nav2 delay: wait for *this* robot's Habitat start + load (+ GPU penalty)."""
    configs = []
    for index in range(1, num_robots + 1):
        i = index - 1
        nav2_delay = (
            i * habitat_step
            + habitat_load
            + i * habitat_load_penalty
            + nav2_gap
            + i * nav2_step
        )
        configs.append({
            'name': f'{name_prefix}{index}',
            'scene': scene,
            'nav2_startup_delay': f'{nav2_delay:.1f}',
            'habitat_startup_delay': f'{i * habitat_step:.1f}',
            'spawn_index': str(i),
            'spawn_count': str(num_robots),
        })
    return configs


def _launch_robot_stacks(context, *args, **kwargs):
    autonomy_ros_share = get_package_share_directory('autonomy_ros')

    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    if num_robots < 1:
        raise RuntimeError(f'num_robots must be >= 1, got {num_robots}')

    name_prefix = LaunchConfiguration('robot_name_prefix').perform(context)
    scene = LaunchConfiguration('scene_data_path').perform(context)
    habitat_step = float(LaunchConfiguration('habitat_startup_delay_step').perform(context))
    habitat_load = float(LaunchConfiguration('habitat_load_sec').perform(context))
    habitat_load_penalty = float(
        LaunchConfiguration('habitat_load_penalty_sec').perform(context))
    nav2_gap = float(LaunchConfiguration('nav2_gap_sec').perform(context))
    nav2_step = float(LaunchConfiguration('nav2_startup_delay_step').perform(context))

    params_file = LaunchConfiguration('params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    autostart = LaunchConfiguration('autostart').perform(context)
    lifecycle_bringup_delay = LaunchConfiguration(
        'lifecycle_bringup_delay').perform(context)

    robot_launch = os.path.join(
        autonomy_ros_share, 'launch', 'navigation_nav2_robot.launch.py'
    )

    actions = []
    for robot in _robot_configs(
        num_robots,
        name_prefix,
        scene,
        habitat_step,
        habitat_load,
        habitat_load_penalty,
        nav2_gap,
        nav2_step,
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
                    'lifecycle_bringup_delay': lifecycle_bringup_delay,
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
            default_value=DEFAULT_MP3D_SCENE,
            description='MP3D scene directory for every robot',
        ),
        DeclareLaunchArgument(
            'habitat_startup_delay_step',
            default_value='3.0',
            description='Extra Habitat delay per robot index (seconds)',
        ),
        DeclareLaunchArgument(
            'habitat_load_sec',
            default_value='8.0',
            description='Expected Habitat Session load time before Nav2 (per robot)',
        ),
        DeclareLaunchArgument(
            'habitat_load_penalty_sec',
            default_value='12.0',
            description=(
                'Extra Nav2 delay per robot index for cumulative GPU load '
                '(robotK waits K×penalty longer after its Habitat start)'
            ),
        ),
        DeclareLaunchArgument(
            'nav2_gap_sec',
            default_value='1.0',
            description='Gap after Habitat load before Nav2 for each robot',
        ),
        DeclareLaunchArgument(
            'nav2_startup_delay_step',
            default_value='12.0',
            description='Extra Nav2 stagger per robot index (seconds)',
        ),
        DeclareLaunchArgument(
            'lifecycle_bringup_delay',
            default_value='8.0',
            description='Per-robot delay before lifecycle_manager autostarts Nav2',
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
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        *declares,
        OpaqueFunction(function=_launch_robot_stacks),
        rviz,
    ])
