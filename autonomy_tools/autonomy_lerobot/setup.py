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
        (os.path.join('share', package_name, 'model'), glob('model/*')),
    ],
    install_requires=[
        'setuptools',
        'mcap',
        'mcap-ros2-support',
        'pyarrow',
        'av',
        'opencv-python-headless',
    ],
    zip_safe=True,
    maintainer='q',
    maintainer_email='quandy2020@126.com',
    description='ROS2 bridge for Hugging Face LeRobot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bag_to_lerobot = autonomy_lerobot.bag_to_lerobot:main',
            'lerobot_offline_pipeline = autonomy_lerobot.offline_pipeline:main',
            'cleanup_lerobot_dataset = autonomy_lerobot.dataset_cleanup:main',
        ],
    },
    scripts=[
        'scripts/encode_dataset_videos.py',
        'scripts/lerobot_dataset_viz',
    ],
)
