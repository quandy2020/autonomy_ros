"""Build configuration for the ``autocharge`` ROS 2 Python package."""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'autocharge'
share_dir = os.path.join('share', package_name)

if os.environ.get("CONDA_PREFIX"):
    raise RuntimeError(
        "Conda environment is not supported in this workspace. "
        "Run `conda deactivate` before building."
    )

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='quandy',
    maintainer_email='quandy@todo.todo',
    description='Automatic docking / charging behaviors',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'env_sim = autocharge.sim.env_sim:main',
            'docking_node = autocharge.docking.node:main',
            'docking_tester = autocharge.docking_tester:main',
        ],
    },
)
