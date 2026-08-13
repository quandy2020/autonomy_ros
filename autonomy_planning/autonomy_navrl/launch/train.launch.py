from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('autonomy_navrl')
    default_config = PathJoinSubstitution([pkg_share, 'config', 's10.yaml'])

    config = LaunchConfiguration('config')
    backend = LaunchConfiguration('backend')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('backend', default_value='mock'),
        ExecuteProcess(
            cmd=[
                'train_navrl',
                '--config', config,
                '--backend', backend,
            ],
            output='screen',
        ),
    ])
