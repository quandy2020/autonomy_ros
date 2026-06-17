#
# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

"""ROS 2 parameters for the Habitat bridge."""

from __future__ import annotations

import os
from dataclasses import MISSING, dataclass

from rclpy.node import Node


@dataclass(frozen=True)
class Config:
    """Immutable bridge configuration loaded from ROS parameters."""

    # Required: no defaults; set via param/habitat.yaml or launch.
    scene_data_path: str
    scene_id: str
    mp3d_root: str
    scene_dataset_config: str
    package_scene_dataset_config: str

    # Habitat camera sensor specs (defaults match param/habitat.yaml).
    image_width: int = 640
    image_height: int = 480
    camera_horizontal_fov_deg: float = 90.0
    sensor_height: float = 0.6  # Habitat agent sensor Y offset.

    agent_pose_topic: str = 'habitat/agent_pose'
    set_agent_pose_topic: str = 'habitat/set_agent_pose'
    agent_pose_frame: str = 'odom'

    # Frame names must match urdf/habitat.urdf and image header.frame_id.
    map_frame: str = 'map'
    odom_frame: str = 'odom'
    base_footprint_frame: str = 'base_footprint'
    base_link_height: float = 0.01
    rgb_camera_frame: str = 'camera_optical_frame'
    depth_camera_frame: str = 'camera_optical_frame'
    semantic_camera_frame: str = 'camera_optical_frame'

    odom_topic: str = 'odom'
    cmd_vel_topic: str = 'cmd_vel'
    cmd_vel_timeout: float = 1.0

    rgb_topic: str = 'camera/rgb/image_raw'
    rgb_camera_info_topic: str = 'camera/rgb/camera_info'
    depth_topic: str = 'camera/depth/image_raw'
    semantic_topic: str = 'camera/semantic/image_raw'

    semantic_ply_path: str = ''  # Empty: use {scene_dir}/{scene_id}_semantic.ply.
    semantic_pointcloud_topic: str = 'semantic_pointcloud'
    semantic_pointcloud_frame: str = 'map'
    semantic_pointcloud_downsample: int = 1
    semantic_pointcloud_rate_hz: float = 1.0  # 0 = latched once; <0 = disabled.

    occupancy_grid_topic: str = 'map'
    occupancy_grid_frame: str = 'map'
    occupancy_grid_resolution: float = 0.05
    occupancy_grid_z_min: float = 0.0  # PLY Z-up floor slice (m).
    occupancy_grid_z_max: float = 0.5
    occupancy_grid_downsample: int = 1
    occupancy_grid_rate_hz: float = 1.0  # 0 = latched once; <0 = disabled.
    update_rate_hz: float = 30.0  # Sim step + camera/odom publish rate.

    def dataset_config(self) -> str:
        if self.scene_dataset_config:
            return self.scene_dataset_config
        return os.path.join(self.mp3d_root, 'mp3d.scene_dataset_config.json')

    def scene_asset(self) -> str:
        # Habitat-Sim scene_id relative to mp3d.scene_dataset_config.json.
        return f'{self.scene_id}/{self.scene_id}.glb'

    def scene_dir(self) -> str:
        if self.scene_data_path:
            return self.scene_data_path.rstrip('/')
        return os.path.join(self.mp3d_root, self.scene_id)


def load(node: Node) -> Config:
    fields = Config.__dataclass_fields__

    # Params from --params-file are overrides; they apply when each param is declared.
    to_declare = [
        (name, '' if field.default is MISSING else field.default)
        for name, field in fields.items()
        if not node.has_parameter(name)
    ]
    if to_declare:
        node.declare_parameters('', to_declare)

    values = {name: node.get_parameter(name).value for name in fields}
    path = str(values['scene_data_path']).rstrip('/')
    if path:
        # scene_data_path overrides scene_id and mp3d_root from launch/yaml.
        values['scene_id'] = os.path.basename(path)
        values['mp3d_root'] = os.path.dirname(path)
        values['scene_data_path'] = path
    elif not str(values['scene_id']).strip() or not str(values['mp3d_root']).strip():
        raise RuntimeError(
            'Missing scene path: set scene_data_path or both scene_id and mp3d_root '
            'in param/habitat.yaml or launch'
        )

    if not str(values['package_scene_dataset_config']).strip():
        raise RuntimeError(
            'Missing package_scene_dataset_config (set in launch or param/habitat.yaml)'
        )

    return Config(**values)
