# Copyright (C) 2025 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_autonomy_config_directory() -> str:
    """autonomy uses cmake build_type; config lives beside autonomy_ros in install/."""
    try:
        return os.path.join(get_package_share_directory('autonomy'), 'config')
    except Exception:
        pass
    ros_share = get_package_share_directory('autonomy_ros')
    install_root = os.path.dirname(os.path.dirname(os.path.dirname(ros_share)))
    candidate = os.path.join(install_root, 'autonomy', 'share', 'autonomy', 'config')
    if os.path.isdir(candidate):
        return candidate
    return ''


def generate_launch_description():
    pkg_share = get_package_share_directory('autonomy_ros')
    params_file = os.path.join(pkg_share, 'config', 'autonomy_params.yaml')
    default_autonomy_config_dir = _resolve_autonomy_config_directory()
    if not default_autonomy_config_dir:
        raise RuntimeError(
            'Could not resolve autonomy config directory. '
            'Build and source install/setup.bash (colcon build --packages-select autonomy autonomy_ros), '
            'or pass autonomy_config_directory:=<path/to/autonomy/config>.'
        )

    sim_mode = LaunchConfiguration('sim_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_sim_mode = DeclareLaunchArgument(
        'sim_mode',
        default_value='gazebo',
        description="Robot backend: 'gazebo' (Gazebo Sim) or 'fake' (diff-drive, no Gazebo)")
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Start RViz2')
    declare_autonomy_config_directory = DeclareLaunchArgument(
        'autonomy_config_directory',
        default_value=default_autonomy_config_dir,
        description='Directory containing autonomy.lua (resolved from install layout by default)',
    )

    autonomy_config_dir = LaunchConfiguration('autonomy_config_directory')

    sim_share = get_package_share_directory('autonomy_simulator')
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'simulator.launch.py')
        ),
        launch_arguments={
            'sim_mode': sim_mode,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    autonomy_node = Node(
        package='autonomy_ros',
        executable='autonomy_node',
        name='autonomy_node',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'autonomy.config_directory': autonomy_config_dir,
            },
        ],
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'autonomy.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_sim_mode,
        declare_use_sim_time,
        declare_use_rviz,
        declare_autonomy_config_directory,
        simulator,
        autonomy_node,
        rviz,
    ])
