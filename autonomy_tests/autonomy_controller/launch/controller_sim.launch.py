# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Controller sim: fake_robot + InteractiveMarker obstacles → PointCloud2 + follow."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('autonomy_controller')
    sim_pkg = get_package_share_directory('autonomy_simulator')
    rviz_arg = LaunchConfiguration('rviz_config').perform(context).strip()
    default_rviz = os.path.join(pkg, 'rviz', 'controller_sim.rviz')
    rviz_config = rviz_arg if rviz_arg else default_rviz

    config_dir = os.path.join(pkg, 'config')
    controller_params = [os.path.join(config_dir, 'controller_sim.yaml')]
    # Launch args below are optional overrides; defaults live in YAML.
    obstacle_params = [os.path.join(config_dir, 'obstacle_marker.yaml')]

    preset = LaunchConfiguration('config_preset').perform(context).strip()
    if preset:
        preset_path = os.path.join(config_dir, 'presets', f'{preset}.yaml')
        if not os.path.isfile(preset_path):
            raise RuntimeError(f'Unknown config_preset: {preset} (missing {preset_path})')
        controller_params.append(preset_path)

    frame_id_arg = LaunchConfiguration('frame_id').perform(context).strip()
    base_frame_arg = LaunchConfiguration('base_frame').perform(context).strip()
    controller_id_arg = LaunchConfiguration('controller_id').perform(context).strip()
    path_shape_arg = LaunchConfiguration('path_shape').perform(context).strip()
    static_count_arg = LaunchConfiguration('static_count').perform(context).strip()
    dynamic_count_arg = LaunchConfiguration('dynamic_count').perform(context).strip()

    fake_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg, 'launch', 'fake_robot.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'publish_odom': 'true',
            'publish_tf': 'true',
            'start_fake_robot_node': 'true',
        }.items(),
    )

    obstacle_marker = Node(
        package='autonomy_controller',
        executable='obstacle_marker_node.py',
        name='obstacle_marker_node',
        output='screen',
        parameters=obstacle_params + [{
            k: v for k, v in {
                'frame_id': frame_id_arg,
                'static_count': int(static_count_arg) if static_count_arg else None,
                'dynamic_count': int(dynamic_count_arg) if dynamic_count_arg else None,
            }.items() if v is not None and v != ''
        }],
    )

    controller_overrides = {
        k: v for k, v in {
            'frame_id': frame_id_arg,
            'base_frame': base_frame_arg,
            'controller_id': controller_id_arg,
            'path_shape': path_shape_arg,
        }.items() if v
    }
    mppi_max = LaunchConfiguration('mppi_viz_max_candidates').perform(context).strip()
    mppi_width = LaunchConfiguration('mppi_viz_line_width').perform(context).strip()
    if mppi_max:
        controller_overrides['mppi_viz_max_candidates'] = int(mppi_max)
    if mppi_width:
        controller_overrides['mppi_viz_line_width'] = float(mppi_width)

    controller_sim = Node(
        package='autonomy_controller',
        executable='controller_sim_node',
        name='controller_sim_node',
        output='screen',
        parameters=controller_params + [controller_overrides],
    )

    rviz = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return [fake_robot, obstacle_marker, controller_sim, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'controller_id',
            default_value='',
            description='Optional override for controller_id in YAML: mppi | rpp | graceful'),
        DeclareLaunchArgument(
            'path_shape',
            default_value='',
            description='Optional override for path_shape in YAML'),
        DeclareLaunchArgument(
            'config_preset',
            default_value='',
            description='Optional YAML preset under config/presets/ '
                        '(e.g. goal_navigation, mppi_performance)'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz with InteractiveMarkers + costmap'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='',
            description='Optional RViz config path (empty = package default)'),
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
        OpaqueFunction(function=_launch_setup),
    ])
