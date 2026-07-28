# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Habitat bridge + namespaced Nav2 for one robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from autonomy_lerobot.data_paths import default_mp3d_scene
from launch.substitutions import LaunchConfiguration

DEFAULT_MP3D_SCENE = str(default_mp3d_scene())


def _launch_robot(context, *args, **kwargs):
    """Start habitat (optional delay) and Nav2 for one robot namespace."""
    autonomy_ros_share = get_package_share_directory('autonomy_ros')
    simulator_share = get_package_share_directory('autonomy_simulator')

    namespace = LaunchConfiguration('namespace').perform(context)
    use_namespace = LaunchConfiguration('use_namespace').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    autostart = LaunchConfiguration('autostart').perform(context)
    use_respawn = LaunchConfiguration('use_respawn').perform(context)
    lifecycle_bringup_delay = LaunchConfiguration(
        'lifecycle_bringup_delay').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    scene_data_path = LaunchConfiguration('scene_data_path').perform(context)
    occupancy_grid_rate_hz = LaunchConfiguration('occupancy_grid_rate_hz').perform(context)
    habitat_delay = float(LaunchConfiguration('habitat_startup_delay').perform(context))
    nav2_delay = float(LaunchConfiguration('nav2_startup_delay').perform(context))
    spawn_index = LaunchConfiguration('spawn_index').perform(context)
    spawn_count = LaunchConfiguration('spawn_count').perform(context)
    spawn_mode = LaunchConfiguration('spawn_mode').perform(context)
    spawn_x = LaunchConfiguration('spawn_x').perform(context)
    spawn_y = LaunchConfiguration('spawn_y').perform(context)
    spawn_yaw = LaunchConfiguration('spawn_yaw').perform(context)
    pedestrians_enabled = LaunchConfiguration('pedestrians_enabled').perform(context)
    pedestrian_count = LaunchConfiguration('pedestrian_count').perform(context)
    human_agent_count = LaunchConfiguration('human_agent_count').perform(context)
    pedestrian_linear_speed = LaunchConfiguration('pedestrian_linear_speed').perform(context)
    pedestrian_goal_count = LaunchConfiguration('pedestrian_goal_count').perform(context)
    humanoid_avatar = LaunchConfiguration('humanoid_avatar').perform(context)
    humanoid_avatars = LaunchConfiguration('humanoid_avatars').perform(context)
    dynamic_actor_kind = LaunchConfiguration('dynamic_actor_kind').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    enable_social_layer = LaunchConfiguration('enable_social_layer').perform(context)

    nav2_launch = os.path.join(
        autonomy_ros_share, 'launch', 'nav2_namespaced.launch.py'
    )
    habitat_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulator_share, 'launch', 'habitat.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'scene_data_path': scene_data_path,
            'occupancy_grid_rate_hz': occupancy_grid_rate_hz,
            'use_sim_time': use_sim_time,
            'spawn_index': spawn_index,
            'spawn_count': spawn_count,
            'spawn_mode': spawn_mode,
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_yaw': spawn_yaw,
            'pedestrians_enabled': pedestrians_enabled,
            'pedestrian_count': pedestrian_count,
            'human_agent_count': human_agent_count,
            'pedestrian_linear_speed': pedestrian_linear_speed,
            'pedestrian_goal_count': pedestrian_goal_count,
            'humanoid_avatar': humanoid_avatar,
            'humanoid_avatars': humanoid_avatars,
            'dynamic_actor_kind': dynamic_actor_kind,
            'use_rviz': use_rviz,
        }.items(),
    )
    nav2_stack = TimerAction(
        period=nav2_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    'namespace': namespace,
                    'use_namespace': use_namespace,
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'autostart': autostart,
                    'use_respawn': use_respawn,
                    'lifecycle_bringup_delay': lifecycle_bringup_delay,
                    'log_level': log_level,
                    'enable_social_layer': enable_social_layer,
                }.items(),
            ),
        ],
    )

    if habitat_delay > 0.0:
        return [TimerAction(period=habitat_delay, actions=[habitat_launch]), nav2_stack]
    return [habitat_launch, nav2_stack]


def generate_launch_description() -> LaunchDescription:
    autonomy_ros_share = get_package_share_directory('autonomy_ros')

    default_params = os.path.join(
        autonomy_ros_share, 'config', 'nav2_habitat_params.yaml'
    )
    default_scene = DEFAULT_MP3D_SCENE

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='ROS namespace for this robot (e.g. robot1)',
        ),
        DeclareLaunchArgument(
            'use_namespace',
            default_value='false',
            description='Place Nav2 nodes under namespace',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Nav2 parameters file',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Habitat bridge uses wall clock; keep false',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Autostart Nav2 lifecycle nodes',
        ),
        DeclareLaunchArgument(
            'use_respawn',
            default_value='true',
            description='Respawn Nav2 nodes on crash',
        ),
        DeclareLaunchArgument(
            'lifecycle_bringup_delay',
            default_value='8.0',
            description='Delay before lifecycle_manager autostarts Nav2',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Nav2 node log level',
        ),
        DeclareLaunchArgument(
            'nav2_startup_delay',
            default_value='25.0',
            description='Seconds after this robot launch to start Nav2',
        ),
        DeclareLaunchArgument(
            'habitat_startup_delay',
            default_value='0.0',
            description='Seconds before starting Habitat (stagger multi-robot GPU load)',
        ),
        DeclareLaunchArgument(
            'scene_data_path',
            default_value=default_scene,
            description='MP3D scene directory for this robot',
        ),
        DeclareLaunchArgument(
            'occupancy_grid_rate_hz',
            default_value='0.0',
            description='Map publish rate; 0 = latched once',
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
            'spawn_x',
            default_value='0.0',
            description='Fixed spawn map X (m) when spawn_mode=fixed',
        ),
        DeclareLaunchArgument(
            'spawn_y',
            default_value='0.0',
            description='Fixed spawn map Y (m) when spawn_mode=fixed',
        ),
        DeclareLaunchArgument(
            'spawn_yaw',
            default_value='0.0',
            description='Fixed spawn yaw (rad) when spawn_mode=fixed',
        ),
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
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch habitat.launch RViz (prefer outer launch RViz when false)',
        ),
        DeclareLaunchArgument(
            'enable_social_layer',
            default_value='false',
            description='Bridge TrackedPersons to pedsim_msgs for Nav2 SocialLayer',
        ),
        OpaqueFunction(function=_launch_robot),
    ])
