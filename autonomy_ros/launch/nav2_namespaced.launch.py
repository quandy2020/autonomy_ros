# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Nav2 navigation stack under a ROS namespace (bringup_launch pattern)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, SetRemap


def generate_launch_description() -> LaunchDescription:
    autonomy_ros_share = get_package_share_directory('autonomy_ros')

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    lifecycle_bringup_delay = LaunchConfiguration('lifecycle_bringup_delay')
    log_level = LaunchConfiguration('log_level')
    enable_social_layer = LaunchConfiguration('enable_social_layer')
    tracked_persons_topic = LaunchConfiguration('tracked_persons_topic')
    people_topic = LaunchConfiguration('people_topic')

    default_params = os.path.join(
        autonomy_ros_share, 'config', 'nav2_habitat_params.yaml'
    )

    declares = [
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace (e.g. robot1)',
        ),
        DeclareLaunchArgument(
            'use_namespace',
            default_value='true',
            description='Apply PushRosNamespace to the Nav2 stack',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Nav2 parameters YAML',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'use_respawn',
            default_value='true',
            description='Respawn Nav2 nodes on crash (recommended for multi-robot)',
        ),
        DeclareLaunchArgument(
            'lifecycle_bringup_delay',
            default_value='8.0',
            description='Delay before lifecycle_manager autostarts the Nav2 stack',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Nav2 node log level',
        ),
        DeclareLaunchArgument(
            'enable_social_layer',
            default_value='false',
            description='Bridge TrackedPersons to pedsim_msgs for SocialLayer',
        ),
        DeclareLaunchArgument(
            'tracked_persons_topic',
            default_value='pedestrian_visualizer/tracked_persons',
            description='Habitat TrackedPersons topic (relative to namespace)',
        ),
        DeclareLaunchArgument(
            'people_topic',
            default_value='local_costmap/people',
            description='pedsim_msgs/People topic for SocialLayer (relative to robot ns)',
        ),
    ]

    nav2_stack = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace,
        ),
        # Habitat publishes TF on namespaced topics (/robotK/tf); Nav2 defaults to /tf.
        SetRemap('/tf', 'tf'),
        SetRemap('/tf_static', 'tf_static'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(autonomy_ros_share, 'launch', 'nav2_navigation.launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': autostart,
                'use_composition': 'False',
                'use_respawn': use_respawn,
                'lifecycle_bringup_delay': lifecycle_bringup_delay,
                'log_level': log_level,
                'enable_social_layer': enable_social_layer,
                'tracked_persons_topic': tracked_persons_topic,
                'people_topic': people_topic,
            }.items(),
        ),
    ])

    return LaunchDescription([*declares, nav2_stack])
