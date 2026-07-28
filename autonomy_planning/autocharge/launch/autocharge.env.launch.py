"""Launch simulation + auto-docking (two nodes) with optional RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory('autocharge')
    rviz_cfg = os.path.join(pkg, 'rviz', 'autocharge.rviz')
    sim_cfg = os.path.join(pkg, 'config', 'sim.yaml')
    dock_cfg = os.path.join(pkg, 'config', 'docking.yaml')

    use_rviz = LaunchConfiguration('use_rviz')
    declare_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualization',
    )

    env_sim = Node(
        package='autocharge',
        executable='env_sim',
        name='env_sim',
        parameters=[sim_cfg] if os.path.isfile(sim_cfg) else [],
        output='screen',
    )
    docking_node = Node(
        package='autocharge',
        executable='docking_node',
        name='docking_node',
        parameters=[dock_cfg] if os.path.isfile(dock_cfg) else [],
        output='screen',
    )
    rviz_args = ['-d', rviz_cfg] if os.path.isfile(rviz_cfg) else []
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=rviz_args,
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_rviz,
        env_sim,
        docking_node,
        rviz,
    ])
