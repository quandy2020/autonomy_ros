# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""One-shot launch: multi-robot Habitat + Nav2 + LeRobot + collection coordinator.

Startup order (global timeline, t=0 is launch):
  1. Habitat  — per-robot stagger (robot1 @ 0s, robot2 @ +stagger, …)
  2. Nav2     — after all Habitat instances have had time to load map/TF
  3. LeRobot  — after Habitat load (parallel with Nav2 bringup)
  4. Coordinator (+ optional RViz) — after last Nav2 stack + nav_ready_sec
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from autonomy_lerobot.data_paths import collection_repo_id, default_mp3d_scene, lerobot_collection_root
from autonomy_lerobot.repo_id import per_robot_repo_id


def _phase_delays(
    num_robots: int,
    *,
    habitat_stagger: float,
    habitat_load: float,
    nav2_gap: float,
    nav2_stagger: float,
    lerobot_gap: float,
    nav_ready_sec: float,
) -> dict[str, float]:
    """Compute per-phase start times from robot count and timing knobs."""
    n = max(1, num_robots)
    last_habitat = (n - 1) * habitat_stagger
    nav2_base = last_habitat + habitat_load + nav2_gap
    last_nav2 = nav2_base + (n - 1) * nav2_stagger
    lerobot_delay = last_habitat + habitat_load + lerobot_gap
    # Coordinator assigns Nav2 goals — start after the last stack has time to activate.
    coordinator_delay = last_nav2 + nav_ready_sec
    return {
        'habitat_stagger': habitat_stagger,
        'nav2_base': nav2_base,
        'nav2_stagger': nav2_stagger,
        'lerobot_delay': lerobot_delay,
        'coordinator_delay': coordinator_delay,
        'last_nav2': last_nav2,
    }


def _launch_phased_stack(context, *args, **kwargs):
    autonomy_ros_share = get_package_share_directory('autonomy_ros')
    autonomy_task_share = get_package_share_directory('autonomy_task')

    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    if num_robots < 1:
        raise RuntimeError(f'num_robots must be >= 1, got {num_robots}')

    prefix = LaunchConfiguration('robot_name_prefix').perform(context)
    dataset_root = LaunchConfiguration('dataset_root').perform(context)
    dataset_repo_id = LaunchConfiguration('dataset_repo_id').perform(context)
    graph_topic = LaunchConfiguration('graph_topic').perform(context)
    task_config = LaunchConfiguration('task_config').perform(context)
    lerobot_config = LaunchConfiguration('lerobot_config').perform(context)
    recording_enabled = (
        LaunchConfiguration('recording_enabled').perform(context).lower() == 'true'
    )
    clean_datasets = (
        LaunchConfiguration('clean_datasets_on_start').perform(context).lower() == 'true'
    )
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() == 'true'
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    scene_data_path = LaunchConfiguration('scene_data_path').perform(context)

    habitat_stagger = float(LaunchConfiguration('habitat_stagger_sec').perform(context))
    habitat_load = float(LaunchConfiguration('habitat_load_sec').perform(context))
    nav2_gap = float(LaunchConfiguration('nav2_gap_sec').perform(context))
    nav2_stagger = float(LaunchConfiguration('nav2_stagger_sec').perform(context))
    lerobot_gap = float(LaunchConfiguration('lerobot_gap_sec').perform(context))
    nav_ready_sec = float(LaunchConfiguration('nav_ready_sec').perform(context))

    phases = _phase_delays(
        num_robots,
        habitat_stagger=habitat_stagger,
        habitat_load=habitat_load,
        nav2_gap=nav2_gap,
        nav2_stagger=nav2_stagger,
        lerobot_gap=lerobot_gap,
        nav_ready_sec=nav_ready_sec,
    )

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                autonomy_ros_share, 'launch', 'navigation_nav2_multi.launch.py',
            )),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_rviz': 'false',
                'num_robots': str(num_robots),
                'robot_name_prefix': prefix,
                'params_file': params_file,
                'scene_data_path': scene_data_path,
                'nav2_startup_delay_base': f'{phases["nav2_base"]:.1f}',
                'nav2_startup_delay_step': f'{phases["nav2_stagger"]:.1f}',
                'habitat_startup_delay_step': f'{phases["habitat_stagger"]:.1f}',
            }.items(),
        ),
    ]

    lerobot_stagger = float(LaunchConfiguration('lerobot_stagger_sec').perform(context))
    lerobot_nodes = []
    if recording_enabled:
        for index in range(1, num_robots + 1):
            name = f'{prefix}{index}'
            lerobot_nodes.append(
                TimerAction(
                    period=phases['lerobot_delay'] + (index - 1) * lerobot_stagger,
                    actions=[
                        Node(
                            package='autonomy_lerobot',
                            executable='lerobot_bridge_node',
                            name='lerobot_bridge_node',
                            namespace=name,
                            output='screen',
                            parameters=[
                                lerobot_config,
                                {
                                    'dataset_root': f'{dataset_root}/{name}',
                                    'dataset_repo_id': per_robot_repo_id(
                                        dataset_repo_id, name),
                                    'overwrite_dataset': clean_datasets,
                                },
                            ],
                        ),
                    ],
                ),
            )
    if lerobot_nodes:
        actions.extend(lerobot_nodes)

    # First Nav2 stack may need (n-1)*stagger + nav_ready_sec to activate; add slack.
    server_wait_sec = (
        nav_ready_sec
        + max(90.0, (num_robots - 1) * nav2_stagger + 60.0)
    )
    coordinator_nodes = [
        Node(
            package='autonomy_task',
            executable='collection_coordinator_node',
            name='collection_coordinator',
            output='screen',
            parameters=[{
                'config_file': task_config,
                'num_robots': num_robots,
                'graph_topic': graph_topic,
                'dataset_root': dataset_root,
                'dataset_repo_id': dataset_repo_id,
                'server_wait_sec': server_wait_sec,
            }],
        ),
    ]
    if use_rviz:
        default_rviz = os.path.join(
            autonomy_ros_share, 'rviz', 'nav2_multi_habitat.rviz')
        use_sim_time_bool = use_sim_time.lower() == 'true'
        coordinator_nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', default_rviz],
                parameters=[{'use_sim_time': use_sim_time_bool}],
            ),
        )
    actions.append(
        TimerAction(period=phases['coordinator_delay'], actions=coordinator_nodes))

    return actions


def generate_launch_description() -> LaunchDescription:
    autonomy_ros_share = get_package_share_directory('autonomy_ros')
    autonomy_task_share = get_package_share_directory('autonomy_task')

    default_nav2_params = os.path.join(
        autonomy_ros_share, 'config', 'nav2_habitat_params.yaml')
    default_task_config = os.path.join(
        autonomy_task_share, 'config', 'collection_task.yaml')
    default_scene = str(default_mp3d_scene())
    default_dataset_root = str(lerobot_collection_root())

    timing_declares = [
        DeclareLaunchArgument(
            'habitat_stagger_sec',
            default_value='3.0',
            description='Delay between each robot Habitat start (phase 1)',
        ),
        DeclareLaunchArgument(
            'habitat_load_sec',
            default_value='4.0',
            description='Expected Habitat Session load time before Nav2 (phase 1→2)',
        ),
        DeclareLaunchArgument(
            'nav2_gap_sec',
            default_value='1.0',
            description='Extra gap after Habitat before first Nav2 (phase 1→2)',
        ),
        DeclareLaunchArgument(
            'nav2_stagger_sec',
            default_value='8.0',
            description='Delay between each robot Nav2 start (phase 2)',
        ),
        DeclareLaunchArgument(
            'lerobot_gap_sec',
            default_value='1.0',
            description='Gap after Habitat before first LeRobot bridge (phase 3)',
        ),
        DeclareLaunchArgument(
            'lerobot_stagger_sec',
            default_value='4.0',
            description='Delay between each robot LeRobot bridge start (reduces DDS load)',
        ),
        DeclareLaunchArgument(
            'nav_ready_sec',
            default_value='25.0',
            description='Extra delay after last Nav2 start before coordinator assigns goals',
        ),
    ]

    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('num_robots', default_value='3'),
        DeclareLaunchArgument('robot_name_prefix', default_value='robot'),
        DeclareLaunchArgument('params_file', default_value=default_nav2_params),
        DeclareLaunchArgument('task_config', default_value=default_task_config),
        DeclareLaunchArgument('scene_data_path', default_value=default_scene),
        DeclareLaunchArgument(
            'graph_topic',
            default_value='',
            description='Override shared graph topic (empty = per-robot habitat/graph)',
        ),
        DeclareLaunchArgument('dataset_root', default_value=default_dataset_root),
        DeclareLaunchArgument(
            'dataset_repo_id', default_value=collection_repo_id()),
        DeclareLaunchArgument('recording_enabled', default_value='true'),
        DeclareLaunchArgument(
            'clean_datasets_on_start',
            default_value='false',
            description='When true, remove per-robot dataset dirs before LeRobot bridge starts',
        ),
        DeclareLaunchArgument(
            'lerobot_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('autonomy_lerobot'),
                'config',
                'lerobot_collection.yaml',
            ]),
        ),
        *timing_declares,
        OpaqueFunction(function=_launch_phased_stack),
    ])
