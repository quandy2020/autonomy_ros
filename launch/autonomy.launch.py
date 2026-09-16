# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Launch process-isolated autonomy_ros bridge (+ optional RViz).

Start the autonomy process separately; this node only bridges via autolink.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('autonomy_ros')
    parameters_file = os.path.join(package_share, 'config', 'parameters.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        Node(
            package='autonomy_ros',
            executable='autonomy_ros_node',
            name='autonomy_ros',
            output='screen',
            parameters=[
                parameters_file,
                {'use_sim_time': use_sim_time},
            ],
        ),
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(package_share, 'rviz', 'autonomy.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
