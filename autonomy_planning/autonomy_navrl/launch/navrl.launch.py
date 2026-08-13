from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('autonomy_navrl')
    default_config = PathJoinSubstitution([pkg_share, 'config', 'deploy.yaml'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'navrl.rviz'])

    config = LaunchConfiguration('config')
    checkpoint = LaunchConfiguration('checkpoint')
    device = LaunchConfiguration('device')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('checkpoint', default_value='navrl_latest.pt'),
        DeclareLaunchArgument('device', default_value='cuda:0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='autonomy_navrl',
            executable='navrl_node',
            name='navrl_node',
            output='screen',
            parameters=[
                ParameterFile(config, allow_substs=True),
                {
                    'checkpoint': checkpoint,
                    'device': device,
                    'use_sim_time': use_sim_time,
                },
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
        ),
    ])
