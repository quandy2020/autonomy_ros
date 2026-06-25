import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'autonomy_lerobot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[],
    zip_safe=True,
    maintainer='q',
    maintainer_email='duyongquan3@jd.com',
    description='ROS2 bridge for Hugging Face LeRobot',
    license='Apache-2.0',
    options={
        'build_scripts': {
            'executable': '/usr/bin/env python3',
        },
    },
    scripts=[
        'scripts/lerobot_bridge_node',
        'scripts/encode_dataset_videos.py',
        'scripts/cleanup_lerobot_dataset.py',
    ],
)
