import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'autonomy_navrl'


def _collect_data_files() -> list[tuple[str, list[str]]]:
    """Collect installable data files; skip missing paths (stale build symlinks)."""
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
    launch_files = [p for p in glob('launch/*.launch.py') if os.path.isfile(p)]
    if launch_files:
        data_files.append((os.path.join('share', package_name, 'launch'), launch_files))

    config_files = [p for p in glob('config/**/*.yaml', recursive=True) if os.path.isfile(p)]
    if config_files:
        config_by_dir: dict[str, list[str]] = {}
        for cfg in config_files:
            rel = os.path.relpath(cfg, 'config')
            dest = os.path.join('share', package_name, 'config', os.path.dirname(rel))
            config_by_dir.setdefault(dest, []).append(cfg)
        for dest, files in config_by_dir.items():
            data_files.append((dest, files))

    rviz_files = [p for p in glob('rviz/*') if os.path.isfile(p)]
    if rviz_files:
        data_files.append((os.path.join('share', package_name, 'rviz'), rviz_files))

    urdf_readme = os.path.join('urdf', 'README.md')
    if os.path.isfile(urdf_readme):
        data_files.append((os.path.join('share', package_name, 'urdf'), [urdf_readme]))

    weights_readme = os.path.join('weights', 'README.md')
    if os.path.isfile(weights_readme):
        data_files.append((os.path.join('share', package_name, 'weights'), [weights_readme]))

    return data_files


data_files = _collect_data_files()

setup(
    name=package_name,
    version='0.4.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='quandy',
    maintainer_email='quandy2020@126.com',
    description=(
        'Isaac Lab modular RL training and ROS 2 deployment for visual navigation'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'navrl_node = autonomy_navrl.deploy.node:main',
            'train_navrl = autonomy_navrl.train.cli:main',
        ],
    },
)
