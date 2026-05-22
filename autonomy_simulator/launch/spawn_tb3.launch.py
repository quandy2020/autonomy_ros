# Copyright (C) 2023 Open Source Robotics Foundation
# Copyright (C) 2024 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def _unpause_world(context, *args, **kwargs):
    return [
        ExecuteProcess(
            cmd=[
                'ign', 'service', '-s', '/world/default/control',
                '--reqtype', 'ignition.msgs.WorldControl',
                '--reptype', 'ignition.msgs.Boolean',
                '--timeout', '5000',
                '--req', 'pause: false',
            ],
            output='screen',
        ),
        LogInfo(msg='Unpaused Gazebo world default'),
    ]


def _remove_robot_sdf(context, robot_sdf_out, *args, **kwargs):
    if os.path.isfile(robot_sdf_out):
        os.remove(robot_sdf_out)
    return []


def generate_launch_description():
    bringup_dir = get_package_share_directory('autonomy_simulator')
    meshes_dir = os.path.join(bringup_dir, 'models', 'turtlebot3_model', 'meshes')

    namespace = LaunchConfiguration('namespace')
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')
    pose = {
        'x': LaunchConfiguration('x_pose', default='-2.00'),
        'y': LaunchConfiguration('y_pose', default='-0.50'),
        'z': LaunchConfiguration('z_pose', default='0.01'),
        'R': LaunchConfiguration('roll', default='0.00'),
        'P': LaunchConfiguration('pitch', default='0.00'),
        'Y': LaunchConfiguration('yaw', default='0.00'),
    }

    robot_sdf_out = os.path.join(
        tempfile.gettempdir(), 'autonomy_tb3_robot.sdf'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace',
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='turtlebot3_waffle',
        description='name of the robot',
    )

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(bringup_dir, 'urdf', 'gz_waffle.sdf.xacro'),
        description='Full path to robot SDF xacro for Gazebo spawn',
    )

    # Write SDF to disk; ros_gz create -string truncates/breaks large XML on argv.
    generate_robot_sdf = ExecuteProcess(
        cmd=[
            'xacro',
            '-o',
            robot_sdf_out,
            ['namespace:=', namespace],
            ['robot_name:=', robot_name],
            ['meshes_dir:=', meshes_dir],
            robot_sdf,
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        parameters=[
            {
                'config_file': os.path.join(
                    bringup_dir, 'configs', 'turtlebot3_waffle_bridge.yaml'
                ),
                'expand_gz_topic_names': False,
                'use_sim_time': True,
                'qos_overrides./cmd_vel.subscription.reliability': 'reliable',
            }
        ],
        output='screen',
    )

    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        namespace=namespace,
        arguments=[
            '-world',
            'default',
            '-name',
            robot_name,
            '-file',
            robot_sdf_out,
            '-x',
            pose['x'],
            '-y',
            pose['y'],
            '-z',
            pose['z'],
            '-R',
            pose['R'],
            '-P',
            pose['P'],
            '-Y',
            pose['Y'],
        ],
    )

    remove_temp_robot_sdf = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[
                OpaqueFunction(
                    function=_remove_robot_sdf,
                    kwargs={'robot_sdf_out': robot_sdf_out},
                )
            ]
        )
    )

    models_path = os.path.join(bringup_dir, 'models')
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', models_path
    )
    set_env_vars_resources2 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        str(Path(bringup_dir).parent.resolve()),
    )
    set_ign_resource_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH', f'{bringup_dir}:{models_path}'
    )

    # xacro -> spawn -> unpause -> bridge
    after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_model,
            on_exit=[
                OpaqueFunction(function=_unpause_world),
                TimerAction(period=1.0, actions=[bridge]),
            ],
        )
    )

    after_robot_sdf = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=generate_robot_sdf,
            on_exit=[spawn_model, after_spawn],
        )
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(set_env_vars_resources)
    ld.add_action(set_env_vars_resources2)
    ld.add_action(set_ign_resource_path)
    ld.add_action(remove_temp_robot_sdf)
    ld.add_action(generate_robot_sdf)
    ld.add_action(after_robot_sdf)

    return ld
