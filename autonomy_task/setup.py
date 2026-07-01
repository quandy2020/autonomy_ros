import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'autonomy_task'

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

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='q',
    maintainer_email='duyongquan3@jd.com',
    description='Multi-robot navigation and data collection orchestration',
    license='Apache-2.0',
    scripts=[
        'scripts/collection_coordinator_node',
        'scripts/collection_stats',
    ],
)
