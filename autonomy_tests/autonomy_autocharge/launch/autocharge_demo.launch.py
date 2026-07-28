# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Habitat + Nav2 predock navigation + autocharge IR docking demo."""

import os

from ament_index_python.packages import get_package_share_directory
from autonomy_lerobot.data_paths import default_mp3d_scene
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _pose_overrides(context) -> dict:
    raw = {
        'charger_x': LaunchConfiguration('charger_x').perform(context),
        'charger_y': LaunchConfiguration('charger_y').perform(context),
        'charger_yaw': LaunchConfiguration('charger_yaw').perform(context),
        'use_explicit_predock': LaunchConfiguration('use_explicit_predock').perform(context),
        'predock_x': LaunchConfiguration('predock_x').perform(context),
        'predock_y': LaunchConfiguration('predock_y').perform(context),
        'predock_yaw': LaunchConfiguration('predock_yaw').perform(context),
        'predock_distance_m': LaunchConfiguration('predock_distance_m').perform(context),
    }
    overrides: dict = {}
    for key, value in raw.items():
        if not str(value).strip():
            continue
        if key == 'use_explicit_predock':
            overrides[key] = str(value).strip().lower() in ('true', '1', 'yes')
            continue
        fv = float(value)
        overrides[key] = fv
        if key.startswith('charger_'):
            overrides['dock_' + key[len('charger_'):]] = fv
    return overrides


def _launch_setup(context, *args, **kwargs):
    demo_pkg = get_package_share_directory('autonomy_autocharge')
    ros_pkg = get_package_share_directory('autonomy_ros')

    demo_cfg = os.path.join(demo_pkg, 'config', 'autocharge_demo.yaml')
    dock_cfg = os.path.join(demo_pkg, 'config', 'docking_demo.yaml')
    rviz_cfg = os.path.join(demo_pkg, 'rviz', 'autocharge_demo.rviz')
    overrides = _pose_overrides(context)

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_pkg, 'launch', 'navigation_nav2.launch.py')
        ),
        launch_arguments={
            'use_rviz': 'false',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart'),
            'nav2_startup_delay': LaunchConfiguration('nav2_startup_delay'),
            'namespace': LaunchConfiguration('namespace'),
            'use_namespace': LaunchConfiguration('use_namespace'),
            'scene_data_path': LaunchConfiguration('scene_data_path'),
            'pedestrians_enabled': LaunchConfiguration('pedestrians_enabled'),
            'pedestrian_count': LaunchConfiguration('pedestrian_count'),
            'spawn_mode': LaunchConfiguration('spawn_mode'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
        }.items(),
    )

    station_ir = Node(
        package='autonomy_autocharge',
        executable='station_ir_sim',
        name='station_ir_sim',
        output='screen',
        parameters=[demo_cfg, overrides],
    )

    charger_markers = Node(
        package='autonomy_autocharge',
        executable='charger_markers_node',
        name='charger_markers_node',
        output='screen',
        parameters=[demo_cfg, overrides],
    )

    docking_node = Node(
        package='autocharge',
        executable='docking_node',
        name='docking_node',
        output='screen',
        parameters=[dock_cfg, overrides],
    )

    demo_overrides = dict(overrides)
    demo_overrides['start_delay_s'] = float(
        LaunchConfiguration('demo_start_delay').perform(context)
    )
    for key in (
        'origin_x',
        'origin_y',
        'origin_yaw',
        'post_dock_hold_s',
    ):
        demo_overrides[key] = float(LaunchConfiguration(key).perform(context))
    demo_overrides['run_return_mission'] = (
        LaunchConfiguration('run_return_mission').perform(context).strip().lower()
        in ('true', '1', 'yes')
    )
    demo_node = Node(
        package='autonomy_autocharge',
        executable='autocharge_demo_node',
        name='autocharge_demo_node',
        output='screen',
        parameters=[demo_cfg, demo_overrides],
    )

    actions = [nav2_launch, station_ir, charger_markers, docking_node, demo_node]

    use_rviz = LaunchConfiguration('use_rviz').perform(context).strip().lower()
    if use_rviz in ('true', '1', 'yes'):
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_cfg],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    demo_pkg = get_package_share_directory('autonomy_autocharge')
    default_params = os.path.join(demo_pkg, 'config', 'nav2_autocharge_demo_params.yaml')
    default_scene = str(default_mp3d_scene())

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('nav2_startup_delay', default_value='8.0'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_namespace', default_value='false'),
        DeclareLaunchArgument('scene_data_path', default_value=default_scene),
        DeclareLaunchArgument('pedestrians_enabled', default_value='false'),
        DeclareLaunchArgument('pedestrian_count', default_value='5'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'demo_start_delay',
            default_value='25.0',
            description='Seconds after launch before autocharge demo starts',
        ),
        DeclareLaunchArgument(
            'charger_x',
            default_value='',
            description='Charger X (m); empty = config/autocharge_demo.yaml',
        ),
        DeclareLaunchArgument(
            'charger_y',
            default_value='',
            description='Charger Y (m); empty = config/autocharge_demo.yaml',
        ),
        DeclareLaunchArgument(
            'charger_yaw',
            default_value='',
            description='Charger yaw (rad); empty = config/autocharge_demo.yaml',
        ),
        DeclareLaunchArgument(
            'use_explicit_predock',
            default_value='',
            description='Use predock_x/y/yaw; empty = yaml default',
        ),
        DeclareLaunchArgument(
            'predock_x',
            default_value='',
            description='Predock X (m); empty = yaml default',
        ),
        DeclareLaunchArgument(
            'predock_y',
            default_value='',
            description='Predock Y (m); empty = yaml default',
        ),
        DeclareLaunchArgument(
            'predock_yaw',
            default_value='',
            description='Predock yaw (rad); empty = yaml default',
        ),
        DeclareLaunchArgument(
            'predock_distance_m',
            default_value='',
            description='Predock offset (m) when use_explicit_predock=false; empty = yaml',
        ),
        DeclareLaunchArgument(
            'origin_x',
            default_value='0.0',
            description='Return navigation goal X in map frame (m)',
        ),
        DeclareLaunchArgument(
            'origin_y',
            default_value='0.0',
            description='Return navigation goal Y in map frame (m)',
        ),
        DeclareLaunchArgument(
            'origin_yaw',
            default_value='0.0',
            description='Return navigation goal yaw in map frame (rad)',
        ),
        DeclareLaunchArgument(
            'post_dock_hold_s',
            default_value='3.0',
            description='Seconds to hold after charge before leaving to predock',
        ),
        DeclareLaunchArgument(
            'run_return_mission',
            default_value='true',
            description='After dock: leave to predock then navigate to origin',
        ),
        DeclareLaunchArgument(
            'spawn_mode',
            default_value='fixed',
            description='Habitat robot spawn mode: fixed | dispersed | random',
        ),
        DeclareLaunchArgument(
            'spawn_x',
            default_value='-9.0',
            description='Robot initial map X (m)',
        ),
        DeclareLaunchArgument(
            'spawn_y',
            default_value='0.0',
            description='Robot initial map Y (m)',
        ),
        DeclareLaunchArgument(
            'spawn_yaw',
            default_value='0.0',
            description='Robot initial yaw (rad)',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
