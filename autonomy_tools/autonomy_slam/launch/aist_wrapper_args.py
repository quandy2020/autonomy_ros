"""Shared launch arguments forwarded from aist_* wrapper launches to aist_slam.launch.py."""

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def declare_common_arguments():
    return [
        DeclareLaunchArgument(
            'launch_rviz', default_value='true',
            description='true: video_publisher + run_slam + rviz2; false: run_video_slam'),
        DeclareLaunchArgument(
            'realtime', default_value='true',
            description='Pace video at fps (default: 30 Hz on camera/image_raw)'),
        DeclareLaunchArgument('frame_skip', default_value='3'),
        DeclareLaunchArgument(
            'no_sleep', default_value='true',
            description='run_video_slam only: process frames as fast as possible'),
    ]


def forward_common_arguments():
    return {
        'launch_rviz': LaunchConfiguration('launch_rviz'),
        'realtime': LaunchConfiguration('realtime'),
        'frame_skip': LaunchConfiguration('frame_skip'),
        'no_sleep': LaunchConfiguration('no_sleep'),
    }
