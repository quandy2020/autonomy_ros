# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Planner sim: fake_robot + InteractiveMarker obstacles → PointCloud2 + plan."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('autonomy_planner')
    sim_pkg = get_package_share_directory('autonomy_simulator')
    rviz_arg = LaunchConfiguration('rviz_config').perform(context).strip()
    default_rviz = os.path.join(pkg, 'rviz', 'planner_sim.rviz')
    rviz_config = rviz_arg if rviz_arg else default_rviz

    config_dir = os.path.join(pkg, 'config')
    planner_params = [os.path.join(config_dir, 'planner_sim.yaml')]
    obstacle_params = [os.path.join(config_dir, 'obstacle_marker.yaml')]

    frame_id_arg = LaunchConfiguration('frame_id').perform(context).strip()
    base_frame_arg = LaunchConfiguration('base_frame').perform(context).strip()
    planner_id_arg = LaunchConfiguration('planner_id').perform(context).strip()
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
        package='autonomy_planner',
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

    planner_overrides = {
        k: v for k, v in {
            'frame_id': frame_id_arg,
            'base_frame': base_frame_arg,
            'planner_id': planner_id_arg,
        }.items() if v
    }

    planner_sim = Node(
        package='autonomy_planner',
        executable='planner_sim_node',
        name='planner_sim_node',
        output='screen',
        parameters=planner_params + [planner_overrides],
    )

    rviz = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return [fake_robot, obstacle_marker, planner_sim, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'planner_id',
            default_value='',
            description='Optional override: navfn_planner | dijkstra_planner | '
                        'theta_star_planner'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz with InteractiveMarkers + costmap + plan'),
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
        OpaqueFunction(function=_launch_setup),
    ])
