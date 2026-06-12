# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0

"""Habitat-Sim MP3D bridge: agent pose + camera topics."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('autonomy_simulator')
    params_file = os.path.join(pkg_share, 'param', 'habitat_bridge.yaml')
    mp3d_dataset_config = os.path.join(
        pkg_share, 'configs', 'mp3d.scene_dataset_config.json'
    )
    nvidia_egl_vendor = os.path.join(pkg_share, 'configs', '10_nvidia.json')

    namespace = LaunchConfiguration('namespace')
    scene_data_path = LaunchConfiguration('scene_data_path')
    scene_id = LaunchConfiguration('scene_id')
    mp3d_root = LaunchConfiguration('mp3d_root')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    occupancy_grid_rate_hz = LaunchConfiguration('occupancy_grid_rate_hz')

    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace')
    declare_scene_data_path = DeclareLaunchArgument(
        'scene_data_path',
        default_value='/workspace/autonomy/src/17DRP5sb8fy',
        description='MP3D scene directory containing .glb assets')
    declare_scene_id = DeclareLaunchArgument(
        'scene_id',
        default_value='17DRP5sb8fy',
        description='MP3D scene id (ignored when scene_data_path is set)')
    declare_mp3d_root = DeclareLaunchArgument(
        'mp3d_root',
        default_value='/workspace/autonomy/src',
        description='MP3D dataset root (ignored when scene_data_path is set)')
    declare_cmd_vel_topic = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='cmd_vel',
        description='cmd_vel subscription (TwistStamped); use cmd_vel_stamped with Nav2')
    declare_occupancy_grid_rate_hz = DeclareLaunchArgument(
        'occupancy_grid_rate_hz',
        default_value='1.0',
        description='Map publish rate; 0 = latched once (Nav2)')

    habitat_bridge = Node(
        package='autonomy_simulator',
        executable='habitat_bridge_node.py',
        name='habitat_bridge_node',
        namespace=namespace,
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': False,
                'scene_data_path': scene_data_path,
                'scene_id': scene_id,
                'mp3d_root': mp3d_root,
                'package_scene_dataset_config': mp3d_dataset_config,
                'cmd_vel_topic': cmd_vel_topic,
                'occupancy_grid_rate_hz': occupancy_grid_rate_hz,
            },
        ],
    )

    return LaunchDescription([
        declare_namespace,
        declare_scene_data_path,
        declare_scene_id,
        declare_mp3d_root,
        declare_cmd_vel_topic,
        declare_occupancy_grid_rate_hz,
        SetEnvironmentVariable('CUDA_VISIBLE_DEVICES', '0'),
        SetEnvironmentVariable('NVIDIA_DRIVER_CAPABILITIES', 'all'),
        SetEnvironmentVariable(
            '__EGL_VENDOR_LIBRARY_FILENAMES', nvidia_egl_vendor),
        habitat_bridge,
    ])
