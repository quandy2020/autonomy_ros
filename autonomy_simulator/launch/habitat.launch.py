# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0

"""Habitat-Sim MP3D bridge: agent pose + camera topics."""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from autonomy_lerobot.data_paths import default_mp3d_scene
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_MP3D_SCENE = str(default_mp3d_scene())


def _simulator_script(name: str) -> str:
    """Resolve installed script path, with source-tree fallback for symlink-install."""
    prefix = get_package_prefix('autonomy_simulator')
    installed = os.path.join(prefix, 'lib', 'autonomy_simulator', name)
    if os.path.isfile(installed):
        return installed
    ws_root = os.path.abspath(os.path.join(prefix, '..', '..'))
    source = os.path.join(
        ws_root, 'src', 'autonomy_ros', 'autonomy_simulator', 'scripts', name,
    )
    if os.path.isfile(source):
        return source
    raise RuntimeError(
        f'Missing autonomy_simulator script {name!r}; '
        'run: colcon build --packages-select autonomy_simulator --symlink-install',
    )


def generate_launch_description():
    pkg = get_package_share_directory('autonomy_simulator')
    params = os.path.join(pkg, 'param', 'habitat.yaml')
    dataset_config = os.path.join(pkg, 'configs', 'mp3d.scene_dataset_config.json')

    ns = LaunchConfiguration('namespace')
    scene = LaunchConfiguration('scene_data_path')
    cmd_vel = LaunchConfiguration('cmd_vel_topic')
    map_hz = LaunchConfiguration('occupancy_grid_rate_hz')
    semantic_ply = LaunchConfiguration('semantic_ply_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    spawn_mode = LaunchConfiguration('spawn_mode')
    spawn_index = LaunchConfiguration('spawn_index')
    spawn_count = LaunchConfiguration('spawn_count')
    pedestrians_enabled = LaunchConfiguration('pedestrians_enabled')
    pedestrian_count = LaunchConfiguration('pedestrian_count')
    trackvla_root = LaunchConfiguration('trackvla_root')
    dynamic_actor_kind = LaunchConfiguration('dynamic_actor_kind')
    humanoid_avatar = LaunchConfiguration('humanoid_avatar')
    humanoid_avatars = LaunchConfiguration('humanoid_avatars')
    human_agent_count = LaunchConfiguration('human_agent_count')
    robot_agent_count = LaunchConfiguration('robot_agent_count')
    robot_asset_root = LaunchConfiguration('robot_asset_root')
    robot_asset_type = LaunchConfiguration('robot_asset_type')
    robot_asset_types = LaunchConfiguration('robot_asset_types')
    robot_asset_counts = LaunchConfiguration('robot_asset_counts')
    robot_radius_overrides = LaunchConfiguration('robot_radius_overrides')
    robot_height_overrides = LaunchConfiguration('robot_height_overrides')
    robot_semantic_id_overrides = LaunchConfiguration('robot_semantic_id_overrides')
    topdown_enabled = LaunchConfiguration('topdown_enabled')
    topdown_mode = LaunchConfiguration('topdown_mode')
    use_rviz = LaunchConfiguration('use_rviz')

    with open(os.path.join(pkg, 'urdf', 'habitat.urdf'), encoding='utf-8') as f:
        urdf = f.read()

    # Keep TF on namespaced topics when namespace is set (multi-robot isolation).
    tf_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    declares = [
        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
        DeclareLaunchArgument(
            'scene_data_path',
            default_value=DEFAULT_MP3D_SCENE,
            description='MP3D scene directory containing .glb assets',
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='cmd_vel',
            description='cmd_vel subscription (geometry_msgs/Twist, Nav2 compatible)',
        ),
        DeclareLaunchArgument(
            'occupancy_grid_rate_hz',
            default_value='1.0',
            description='Map publish rate; 0 = latched once',
        ),
        DeclareLaunchArgument(
            'semantic_ply_path',
            default_value='',
            description='Override semantic PLY; empty = {scene_dir}/{scene_id}_semantic.ply',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'spawn_mode',
            default_value='fixed',
            description='Agent spawn: dispersed | random | fixed',
        ),
        DeclareLaunchArgument(
            'spawn_index',
            default_value='0',
            description='0-based robot index for dispersed spawn slot',
        ),
        DeclareLaunchArgument(
            'spawn_count',
            default_value='1',
            description='Total robots sharing the dispersed spawn layout',
        ),
        DeclareLaunchArgument(
            'pedestrians_enabled',
            default_value='false',
            description='Enable evt_bench-style dynamic pedestrians on navmesh',
        ),
        DeclareLaunchArgument(
            'pedestrian_count',
            default_value='5',
            description='Number of dynamic pedestrian obstacles (URDF humanoid mesh)',
        ),
        DeclareLaunchArgument(
            'trackvla_root',
            default_value='',
            description='TrackVLA repo root for humanoid assets (default: TRACKVLA_ROOT env or sibling TrackVLA/)',
        ),
        DeclareLaunchArgument(
            'dynamic_actor_kind',
            default_value='humanoid',
            description='Dynamic actor renderer: humanoid | robot',
        ),
        DeclareLaunchArgument(
            'humanoid_avatar',
            default_value='female_2',
            description='Fallback humanoid avatar name',
        ),
        DeclareLaunchArgument(
            'humanoid_avatars',
            default_value='',
            description='Comma-separated humanoid avatar names; empty = auto-discover all',
        ),
        DeclareLaunchArgument(
            'human_agent_count',
            default_value='0',
            description='Number of dynamic humanoid agents',
        ),
        DeclareLaunchArgument(
            'robot_agent_count',
            default_value='0',
            description='Number of dynamic robot agents',
        ),
        DeclareLaunchArgument(
            'robot_asset_root',
            default_value='',
            description='Root directory containing robot URDF assets; empty = autonomy_simulator/urdf',
        ),
        DeclareLaunchArgument(
            'robot_asset_type',
            default_value='turtlebot3_waffle',
            description='Fallback robot asset type (.urdf stem)',
        ),
        DeclareLaunchArgument(
            'robot_asset_types',
            default_value='',
            description='Comma-separated robot asset types; empty = auto-discover all',
        ),
        DeclareLaunchArgument(
            'robot_asset_counts',
            default_value='',
            description='Comma-separated exact robot counts, e.g. spot=2,jackal=1',
        ),
        DeclareLaunchArgument(
            'robot_radius_overrides',
            default_value='',
            description='Comma-separated robot radius overrides, e.g. jackal=0.32,husky=0.45',
        ),
        DeclareLaunchArgument(
            'robot_height_overrides',
            default_value='',
            description='Comma-separated robot height overrides, e.g. jackal=0.40,stretch=1.25',
        ),
        DeclareLaunchArgument(
            'robot_semantic_id_overrides',
            default_value='',
            description='Comma-separated robot semantic id overrides, e.g. jackal=333,husky=334',
        ),
        DeclareLaunchArgument(
            'topdown_enabled',
            default_value='false',
            description='Publish overview RGB on camera/topdown/image_raw',
        ),
        DeclareLaunchArgument(
            'topdown_mode',
            default_value='room',
            description='room | oblique | overhead | interactive (RViz 6-DOF marker)',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch rviz2 with rviz/habitat.rviz (3D orbit + scene layers)',
        ),
    ]

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=ns,
        output='screen',
        remappings=tf_remappings,
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': urdf}],
    )

    habitat_odom_tf = Node(
        executable='/opt/venv/bin/python3',
        arguments=[_simulator_script('habitat_odom_tf_node.py')],
        name='habitat_odom_tf',
        namespace=ns,
        output='screen',
        remappings=tf_remappings,
        parameters=[
            params,
            {
                'use_sim_time': use_sim_time,
                'scene_data_path': scene,
                'package_scene_dataset_config': dataset_config,
            },
        ],
    )

    habitat_node = Node(
        package='autonomy_simulator',
        executable='habitat_node.py',
        name='habitat_node',
        namespace=ns,
        output='screen',
        remappings=tf_remappings,
        parameters=[
            params,
            {
                'use_sim_time': use_sim_time,
                'scene_data_path': scene,
                'package_scene_dataset_config': dataset_config,
                'cmd_vel_topic': cmd_vel,
                'occupancy_grid_rate_hz': map_hz,
                'semantic_ply_path': semantic_ply,
                'spawn_mode': spawn_mode,
                'spawn_index': spawn_index,
                'spawn_count': spawn_count,
                'pedestrians_enabled': pedestrians_enabled,
                'pedestrian_count': pedestrian_count,
                'trackvla_root': trackvla_root,
                'dynamic_actor_kind': dynamic_actor_kind,
                'humanoid_avatar': humanoid_avatar,
                'humanoid_avatars': humanoid_avatars,
                'human_agent_count': human_agent_count,
                'robot_agent_count': robot_agent_count,
                'robot_asset_root': robot_asset_root,
                'robot_asset_type': robot_asset_type,
                'robot_asset_types': robot_asset_types,
                'robot_asset_counts': robot_asset_counts,
                'robot_radius_overrides': robot_radius_overrides,
                'robot_height_overrides': robot_height_overrides,
                'robot_semantic_id_overrides': robot_semantic_id_overrides,
                'topdown_enabled': topdown_enabled,
                'topdown_mode': topdown_mode,
            },
        ],
    )

    rviz_config = os.path.join(pkg, 'rviz', 'habitat.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=ns,
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        # Docker /dev/shm: avoid Fast DDS SHM port lock failures (fastrtps_port*)
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        *declares,
        robot_state_publisher,
        habitat_odom_tf,
        habitat_node,
        rviz_node,
    ])
