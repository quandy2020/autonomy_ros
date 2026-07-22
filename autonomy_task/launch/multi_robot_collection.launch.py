# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""One-shot launch: multi-robot Habitat + Nav2 + LeRobot + collection coordinator.

Startup order (global timeline, t=0 is launch):
  1. Habitat  — per-robot stagger (robot1 @ 0s, robot2 @ +stagger, …)
  2. Nav2     — after all Habitat instances have had time to load map/TF
  3. LeRobot  — after all Nav2 stacks have started (reduces lifecycle contention)
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
from launch.substitutions import FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from autonomy_task.launch_utils import collection_coordinator_node


def _lerobot_bridge_node(**kwargs) -> Node:
    """Start LeRobot bridge via python3 -m (avoids stale/broken ament scripts in install)."""
    return Node(
        executable=FindExecutable(name='python3'),
        arguments=['-m', 'autonomy_lerobot.node'],
        respawn=True,
        respawn_delay=3.0,
        **kwargs,
    )

from autonomy_lerobot.collection_params import jdrobot_collection_parameters
from autonomy_lerobot.data_paths import collection_repo_id, default_mp3d_scene, lerobot_collection_root
from autonomy_lerobot.repo_id import per_robot_repo_id


def _phase_delays(
    num_robots: int,
    *,
    habitat_stagger: float,
    habitat_load: float,
    habitat_load_penalty: float,
    nav2_gap: float,
    nav2_stagger: float,
    lerobot_gap: float,
    nav_ready_sec: float,
) -> dict[str, float]:
    """Compute per-phase start times from robot count and timing knobs."""
    n = max(1, num_robots)
    last_habitat = (n - 1) * habitat_stagger
    last_nav2 = (
        last_habitat
        + habitat_load
        + (n - 1) * habitat_load_penalty
        + nav2_gap
        + (n - 1) * nav2_stagger
    )
    # Start LeRobot after all Nav2 stacks have launched (avoids DDS/CPU contention during bringup).
    lerobot_delay = last_nav2 + lerobot_gap
    # Coordinator assigns Nav2 goals — start after the last stack has time to activate.
    coordinator_delay = last_nav2 + nav_ready_sec
    return {
        'habitat_stagger': habitat_stagger,
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
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    rviz_robot = LaunchConfiguration('rviz_robot').perform(context)
    rviz_delay_sec = float(LaunchConfiguration('rviz_delay_sec').perform(context))

    habitat_stagger = float(LaunchConfiguration('habitat_stagger_sec').perform(context))
    habitat_load = float(LaunchConfiguration('habitat_load_sec').perform(context))
    habitat_load_penalty = float(
        LaunchConfiguration('habitat_load_penalty_sec').perform(context))
    nav2_gap = float(LaunchConfiguration('nav2_gap_sec').perform(context))
    nav2_stagger = float(LaunchConfiguration('nav2_stagger_sec').perform(context))
    lerobot_gap = float(LaunchConfiguration('lerobot_gap_sec').perform(context))
    nav_ready_sec = float(LaunchConfiguration('nav_ready_sec').perform(context))

    phases = _phase_delays(
        num_robots,
        habitat_stagger=habitat_stagger,
        habitat_load=habitat_load,
        habitat_load_penalty=habitat_load_penalty,
        nav2_gap=nav2_gap,
        nav2_stagger=nav2_stagger,
        lerobot_gap=lerobot_gap,
        nav_ready_sec=nav_ready_sec,
    )

    pedestrians_enabled = LaunchConfiguration('pedestrians_enabled').perform(context)
    pedestrian_count = LaunchConfiguration('pedestrian_count').perform(context)
    human_agent_count = LaunchConfiguration('human_agent_count').perform(context)
    pedestrian_linear_speed = LaunchConfiguration('pedestrian_linear_speed').perform(context)
    pedestrian_goal_count = LaunchConfiguration('pedestrian_goal_count').perform(context)
    humanoid_avatar = LaunchConfiguration('humanoid_avatar').perform(context)
    humanoid_avatars = LaunchConfiguration('humanoid_avatars').perform(context)
    dynamic_actor_kind = LaunchConfiguration('dynamic_actor_kind').perform(context)
    enable_social_layer = (
        'true'
        if LaunchConfiguration('pedestrians_enabled').perform(context).lower() == 'true'
        else 'false'
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
                'habitat_startup_delay_step': f'{phases["habitat_stagger"]:.1f}',
                'habitat_load_sec': f'{habitat_load:.1f}',
                'habitat_load_penalty_sec': f'{habitat_load_penalty:.1f}',
                'nav2_gap_sec': f'{nav2_gap:.1f}',
                'nav2_startup_delay_step': f'{phases["nav2_stagger"]:.1f}',
                'spawn_mode': 'dispersed',
                'enable_social_layer': enable_social_layer,
                'pedestrians_enabled': pedestrians_enabled,
                'pedestrian_count': pedestrian_count,
                'human_agent_count': human_agent_count,
                'pedestrian_linear_speed': pedestrian_linear_speed,
                'pedestrian_goal_count': pedestrian_goal_count,
                'humanoid_avatar': humanoid_avatar,
                'humanoid_avatars': humanoid_avatars,
                'dynamic_actor_kind': dynamic_actor_kind,
            }.items(),
        ),
    ]

    lerobot_stagger = float(LaunchConfiguration('lerobot_stagger_sec').perform(context))
    last_lerobot = phases['lerobot_delay'] + max(0, num_robots - 1) * lerobot_stagger
    coordinator_delay = max(phases['last_nav2'] + nav_ready_sec, last_lerobot + 15.0)
    phases['coordinator_delay'] = coordinator_delay

    lerobot_nodes = []
    if recording_enabled:
        for index in range(1, num_robots + 1):
            name = f'{prefix}{index}'
            lerobot_nodes.append(
                TimerAction(
                    period=phases['lerobot_delay'] + (index - 1) * lerobot_stagger,
                    actions=[
                        _lerobot_bridge_node(
                            name='lerobot_bridge_node',
                            namespace=name,
                            output='screen',
                            parameters=[
                                lerobot_config,
                                jdrobot_collection_parameters(
                                    dataset_root=f'{dataset_root}/{name}',
                                    dataset_repo_id=per_robot_repo_id(
                                        dataset_repo_id, name),
                                    overwrite_dataset=clean_datasets,
                                ),
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
        collection_coordinator_node(
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
        use_sim_time_bool = use_sim_time.lower() == 'true'
        tf_ns = rviz_robot.strip('/') or f'{prefix}1'
        actions.append(
            TimerAction(
                period=max(phases['last_nav2'] + rviz_delay_sec, 5.0),
                actions=[
                    Node(
                        package='rviz2',
                        executable='rviz2',
                        name='rviz2',
                        output='screen',
                        arguments=['-d', rviz_config],
                        parameters=[{'use_sim_time': use_sim_time_bool}],
                        remappings=[
                            ('/tf', f'/{tf_ns}/tf'),
                            ('/tf_static', f'/{tf_ns}/tf_static'),
                        ],
                    ),
                ],
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
            default_value='8.0',
            description='Expected Habitat Session load time before Nav2 (phase 1→2)',
        ),
        DeclareLaunchArgument(
            'habitat_load_penalty_sec',
            default_value='12.0',
            description='Extra Nav2 delay per robot index for cumulative GPU load',
        ),
        DeclareLaunchArgument(
            'nav2_gap_sec',
            default_value='1.0',
            description='Extra gap after Habitat before first Nav2 (phase 1→2)',
        ),
        DeclareLaunchArgument(
            'nav2_stagger_sec',
            default_value='12.0',
            description='Delay between each robot Nav2 start (phase 2)',
        ),
        DeclareLaunchArgument(
            'lerobot_gap_sec',
            default_value='15.0',
            description='Gap after last Nav2 start before first LeRobot bridge (phase 3)',
        ),
        DeclareLaunchArgument(
            'lerobot_stagger_sec',
            default_value='6.0',
            description='Delay between each robot LeRobot bridge start (reduces DDS load)',
        ),
        DeclareLaunchArgument(
            'nav_ready_sec',
            default_value='45.0',
            description='Extra delay after last Nav2 start before coordinator assigns goals',
        ),
    ]

    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                autonomy_ros_share, 'rviz', 'nav2_multi_habitat_collection.rviz'),
            description='RViz layout (map, pedestrians, costmaps, collection waypoints)',
        ),
        DeclareLaunchArgument(
            'rviz_robot',
            default_value='robot1',
            description='Namespace for RViz TF remap (e.g. robot1 -> /robot1/tf)',
        ),
        DeclareLaunchArgument(
            'rviz_delay_sec',
            default_value='20.0',
            description='Start RViz this many seconds after last Nav2 stack launch',
        ),
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
            default_value='true',
            description='When true, remove per-robot dataset dirs before LeRobot bridge starts (required when switching to jdrobot/kujiale schema)',
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
        DeclareLaunchArgument(
            'pedestrians_enabled',
            default_value='false',
            description='Enable dynamic humanoid pedestrians in Habitat-Sim',
        ),
        DeclareLaunchArgument(
            'pedestrian_count',
            default_value='5',
            description='Number of dynamic humanoid agents',
        ),
        DeclareLaunchArgument(
            'human_agent_count',
            default_value='0',
            description='Explicit humanoid agent count (overrides pedestrian_count when >0)',
        ),
        DeclareLaunchArgument(
            'pedestrian_linear_speed',
            default_value='0.5',
            description='Humanoid walking speed (m/s); spawn uses 0.8–1.2× → ~0.4–0.6 m/s',
        ),
        DeclareLaunchArgument(
            'pedestrian_goal_count',
            default_value='4',
            description='Number of waypoints per pedestrian cycle',
        ),
        DeclareLaunchArgument(
            'humanoid_avatar',
            default_value='female_2',
            description='Fallback humanoid avatar name',
        ),
        DeclareLaunchArgument(
            'humanoid_avatars',
            default_value='',
            description='Comma-separated avatar names; empty = auto-discover all',
        ),
        DeclareLaunchArgument(
            'dynamic_actor_kind',
            default_value='humanoid',
            description='Dynamic actor renderer: humanoid | robot',
        ),
        OpaqueFunction(function=_launch_phased_stack),
    ])
