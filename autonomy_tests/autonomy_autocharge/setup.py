"""Build configuration for autonomy_autocharge demo package."""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'autonomy_autocharge'
share_dir = os.path.join('share', package_name)

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join(share_dir, 'launch'),
            glob(os.path.join('launch', '*.launch.py')),
        ),
        (
            os.path.join(share_dir, 'config'),
            glob(os.path.join('config', '*.yaml')),
        ),
        (
            os.path.join(share_dir, 'rviz'),
            glob(os.path.join('rviz', '*.rviz')),
        ),
        (
            os.path.join(share_dir, 'scripts'),
            glob(os.path.join('scripts', '*.sh')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='quandy',
    maintainer_email='quandy2020@126.com',
    description='Nav2 predock + autocharge action demo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autocharge_demo_node = autonomy_autocharge.autocharge_demo_node:main',
            'charger_markers_node = autonomy_autocharge.charger_markers_node:main',
            'station_ir_sim = autonomy_autocharge.station_ir_sim:main',
        ],
    },
)
