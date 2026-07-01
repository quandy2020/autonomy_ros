# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0

"""Habitat-Sim MP3D bridge: agent pose + camera topics."""

import os

from ament_index_python.packages import get_package_share_directory
from autonomy_lerobot.data_paths import default_mp3d_scene
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_MP3D_SCENE = str(default_mp3d_scene())


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
            description='Override PLY path; empty = pointcloud.ply then *_semantic.ply',
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
            },
        ],
    )

    return LaunchDescription([
        # Docker /dev/shm: avoid Fast DDS SHM port lock failures (fastrtps_port*)
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        *declares,
        robot_state_publisher,
        habitat_node,
    ])
