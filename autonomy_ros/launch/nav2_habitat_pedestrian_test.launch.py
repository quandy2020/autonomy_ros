# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Habitat + Nav2 (MPPI) + dynamic pedestrians — single-robot smoke test.

Starts Habitat humanoid pedestrians and Nav2 (MPPI + social costmap layer).
TrackedPersons are bridged to pedsim_msgs/People for MPPI CostCritic.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from autonomy_lerobot.data_paths import default_mp3d_scene
from launch_ros.actions import Node

DEFAULT_MP3D_SCENE = str(default_mp3d_scene())


def generate_launch_description() -> LaunchDescription:
    autonomy_ros_share = get_package_share_directory('autonomy_ros')

    default_params = os.path.join(
        autonomy_ros_share, 'config', 'nav2_habitat_params.yaml'
    )
    default_rviz = os.path.join(
        autonomy_ros_share, 'rviz', 'habitat_nav2.rviz'
    )
    default_scene = DEFAULT_MP3D_SCENE

    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    scene_data_path = LaunchConfiguration('scene_data_path')
    pedestrian_count = LaunchConfiguration('pedestrian_count')
    nav2_startup_delay = LaunchConfiguration('nav2_startup_delay')

    # Scoped group: inner navigation_nav2_robot declares use_rviz (default false);
    # must not override this launch's use_rviz or outer RViz never starts.
    robot_stack = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        autonomy_ros_share, 'launch', 'navigation_nav2_robot.launch.py'
                    )
                ),
                launch_arguments={
                    'namespace': '',
                    'use_namespace': 'false',
                    'scene_data_path': scene_data_path,
                    'params_file': default_params,
                    'use_sim_time': 'false',
                    'autostart': 'true',
                    'nav2_startup_delay': nav2_startup_delay,
                    'habitat_startup_delay': '0.0',
                    'lifecycle_bringup_delay': '8.0',
                    'occupancy_grid_rate_hz': '0.0',
                    'pedestrians_enabled': 'true',
                    'pedestrian_count': pedestrian_count,
                    'human_agent_count': '0',
                    'pedestrian_linear_speed': '0.5',
                    'pedestrian_goal_count': '4',
                    'spawn_mode': 'dispersed',
                    'dynamic_actor_kind': 'humanoid',
                    'enable_social_layer': 'true',
                    'use_rviz': 'false',
                }.items(),
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'scene_data_path',
            default_value=default_scene,
            description='MP3D scene directory',
        ),
        DeclareLaunchArgument(
            'pedestrian_count',
            default_value='5',
            description='Number of dynamic humanoid pedestrians',
        ),
        DeclareLaunchArgument(
            'nav2_startup_delay',
            default_value='25.0',
            description='Wait for Habitat map/TF before Nav2 lifecycle autostart',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='RViz config (must include nav2_rviz_plugins Navigation 2 + GoalTool)',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Open Habitat + Nav2 RViz (Navigation 2 panel + map/pedestrians)',
        ),
        LogInfo(msg=['RViz config: ', rviz_config]),
        robot_stack,
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': False}],
        ),
    ])
