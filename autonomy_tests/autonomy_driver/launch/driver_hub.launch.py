from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("autonomy_driver")
    config_arg = DeclareLaunchArgument(
        "config",
        default_value="driver_hub_mock.yaml",
        description=(
            "Config under share/autonomy_driver/config/: "
            "driver_hub_mock.yaml | driver_hub_ros.yaml | "
            "driver_hub_hardware.yaml | driver_hub_realsense.yaml"
        ),
    )
    config_path = os.path.join(
        pkg_share, "config", LaunchConfiguration("config"))

    return LaunchDescription([
        config_arg,
        Node(
            package="autonomy_driver",
            executable="driver_hub_node",
            name="driver_hub_node",
            output="screen",
            parameters=[config_path],
        ),
    ])
