"""AIST Living Lab — localization on sequence 2 with map from sequence 1."""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
if _LAUNCH_DIR not in sys.path:
    sys.path.append(_LAUNCH_DIR)

from aist_wrapper_args import declare_common_arguments, forward_common_arguments  # noqa: E402


def generate_launch_description():
    pkg_share = get_package_share_directory('autonomy_slam')
    launch_args = {
        'task': 'localization',
        'sequence': 'aist_living_lab_2',
        **forward_common_arguments(),
    }
    return LaunchDescription([
        *declare_common_arguments(),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'aist_slam.launch.py')),
            launch_arguments=launch_args.items(),
        ),
    ])
