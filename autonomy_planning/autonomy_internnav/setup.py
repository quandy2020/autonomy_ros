import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'autonomy_internnav'

data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]
launch_files = glob('launch/*.launch.py')
if launch_files:
    data_files.append(
        (os.path.join('share', package_name, 'launch'), launch_files))
config_files = glob('config/*')
if config_files:
    data_files.append(
        (os.path.join('share', package_name, 'config'), config_files))
rviz_files = glob('rviz/*')
if rviz_files:
    data_files.append(
        (os.path.join('share', package_name, 'rviz'), rviz_files))
# Large checkpoints stay in source weights/ (gitignored); do not install via setuptools.
weights_readme = os.path.join('weights', 'README.md')
if os.path.isfile(weights_readme):
    data_files.append(
        (os.path.join('share', package_name, 'weights'), [weights_readme]))

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='quandy',
    maintainer_email='duyongquan3@jd.com',
    description='ROS 2 NavDP inference bridge and training for point-goal navigation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'internnav_node = autonomy_internnav.node:main',
            'train_internnav = autonomy_internnav.train.cli:main',
            'train_navdp = autonomy_internnav.train.cli:main',
        ],
    },
)
