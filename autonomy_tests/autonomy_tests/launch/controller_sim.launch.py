# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Controller sim entry point for the autonomy_tests metapackage."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    controller_pkg = get_package_share_directory('autonomy_controller')
    tests_pkg = get_package_share_directory('autonomy_tests')
    default_rviz = os.path.join(tests_pkg, 'rviz', 'controller_sim.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'controller_id',
            default_value='',
            description='Optional override for controller_id in YAML'),
        DeclareLaunchArgument(
            'path_shape',
            default_value='',
            description='Optional override for path_shape in YAML'),
        DeclareLaunchArgument(
            'config_preset',
            default_value='',
            description='Optional YAML preset under autonomy_controller/config/presets/'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz (ReferencePath on /controller_sim/reference_path)'),
        DeclareLaunchArgument(
            'frame_id',
            default_value='',
            description='Optional override for frame_id in YAML'),
        DeclareLaunchArgument(
            'base_frame',
            default_value='',
            description='Optional override for base_frame in YAML'),
        DeclareLaunchArgument(
            'static_count',
            default_value='',
            description='Optional override for static_count in YAML'),
        DeclareLaunchArgument(
            'dynamic_count',
            default_value='',
            description='Optional override for dynamic_count in YAML'),
        DeclareLaunchArgument(
            'mppi_viz_max_candidates',
            default_value='',
            description='Override mppi_viz_max_candidates (empty = use YAML)'),
        DeclareLaunchArgument(
            'mppi_viz_line_width',
            default_value='',
            description='Override mppi_viz_line_width (empty = use YAML)'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(controller_pkg, 'launch', 'controller_sim.launch.py')
            ),
            launch_arguments={
                'controller_id': LaunchConfiguration('controller_id'),
                'path_shape': LaunchConfiguration('path_shape'),
                'config_preset': LaunchConfiguration('config_preset'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'rviz_config': default_rviz,
                'frame_id': LaunchConfiguration('frame_id'),
                'base_frame': LaunchConfiguration('base_frame'),
                'static_count': LaunchConfiguration('static_count'),
                'dynamic_count': LaunchConfiguration('dynamic_count'),
                'mppi_viz_max_candidates': LaunchConfiguration('mppi_viz_max_candidates'),
                'mppi_viz_line_width': LaunchConfiguration('mppi_viz_line_width'),
            }.items(),
        ),
    ])
