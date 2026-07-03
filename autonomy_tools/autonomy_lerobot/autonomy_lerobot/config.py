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

"""ROS 2 parameters for the LeRobot bridge."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from autonomy_lerobot.data_paths import lerobot_root
from autonomy_lerobot.repo_id import sanitize_repo_id

if TYPE_CHECKING:
    from rclpy.node import Node

DATASET_FORMAT_HABITAT = 'habitat_nav2'
DATASET_FORMAT_JDROBOT = 'jdrobot'


@dataclass(frozen=True)
class Config:
    """Immutable LeRobot bridge configuration."""

    # Habitat camera / state (autonomy_simulator/param/habitat.yaml).
    rgb_topic: str = 'camera/rgb/image_raw'
    depth_topic: str = 'camera/depth/image_raw'
    semantic_topic: str = 'camera/semantic/image_raw'
    camera_info_topic: str = 'camera/rgb/camera_info'
    odom_topic: str = 'odom'
    agent_pose_topic: str = 'habitat/agent_pose'
    cmd_vel_topic: str = 'cmd_vel'
    map_topic: str = 'map'
    pointcloud_topic: str = 'semantic_pointcloud'

    # Nav2 planning (nav2_bringup defaults).
    global_plan_topic: str = 'plan'
    # MPPI (nav2_habitat_params.yaml) publishes transformed_global_plan, not local_plan.
    local_plan_topic: str = 'transformed_global_plan'
    global_costmap_topic: str = 'global_costmap/costmap'
    local_costmap_topic: str = 'local_costmap/costmap'

    use_depth: bool = False
    record_depth: bool = False
    record_semantic: bool = True
    record_map: bool = True
    record_pointcloud: bool = True
    record_camera_info: bool = True
    record_nav2: bool = True
    # True: back-project depth+camera_info (robot view); False: subscribe pointcloud_topic.
    pointcloud_from_depth: bool = True
    pointcloud_stride: int = 4

    image_width: int = 640
    image_height: int = 480
    max_path_waypoints: int = 512
    max_pointcloud_points: int = 4096

    dataset_repo_id: str = 'local/habitat_nav2'
    dataset_root: str = ''
    record_fps: float = 10.0
    task: str = 'navigate to goal'

    # Dataset schema: habitat_nav2 (navigation) or jdrobot (kujiale-compatible).
    dataset_format: str = DATASET_FORMAT_HABITAT
    robot_type: str = ''
    depth_min_m: float = 0.0
    depth_max_m: float = 10.0

    # LeRobot video encoding (RGB/semantic -> .mp4 under videos/).
    video_vcodec: str = 'h264'
    streaming_encoding: bool = True
    parallel_video_encoding: bool = True
    overwrite_dataset: bool = False

    # Robot reset (map→base TF resync via habitat/set_agent_pose).
    set_agent_pose_topic: str = 'habitat/set_agent_pose'
    map_frame: str = 'map'
    base_frame: str = 'base_footprint'

    @property
    def image_shape(self) -> tuple[int, int, int]:
        return (self.image_height, self.image_width, 3)

    @property
    def is_jdrobot(self) -> bool:
        return self.dataset_format == DATASET_FORMAT_JDROBOT

    @property
    def effective_robot_type(self) -> str:
        if self.robot_type.strip():
            return self.robot_type.strip()
        return 'jdrobot' if self.is_jdrobot else 'habitat_diffdrive'


_FLOAT_FIELDS = frozenset({'record_fps', 'depth_min_m', 'depth_max_m'})
_BOOL_FIELDS = frozenset({
    'use_depth', 'record_depth', 'record_semantic', 'record_map', 'record_pointcloud',
    'record_camera_info', 'record_nav2', 'overwrite_dataset', 'pointcloud_from_depth',
})
_INT_FIELDS = frozenset({
    'image_width', 'image_height', 'max_path_waypoints', 'max_pointcloud_points',
    'pointcloud_stride',
})


def load(node: 'Node') -> Config:
    """Declare and read bridge parameters from the ROS node."""
    fields = Config.__dataclass_fields__
    to_declare = [
        (name, field.default)
        for name, field in fields.items()
        if not node.has_parameter(name)
    ]
    if to_declare:
        node.declare_parameters('', to_declare)

    values = {name: node.get_parameter(name).value for name in fields}
    for name in _BOOL_FIELDS:
        values[name] = bool(values[name])
    for name in _INT_FIELDS:
        values[name] = int(values[name])
    for name in _FLOAT_FIELDS:
        values[name] = float(values[name])
    values['dataset_repo_id'] = sanitize_repo_id(str(values['dataset_repo_id']))
    values['dataset_format'] = str(values['dataset_format']).strip()
    values['robot_type'] = str(values['robot_type']).strip()
    if values['dataset_format'] not in (DATASET_FORMAT_HABITAT, DATASET_FORMAT_JDROBOT):
        raise ValueError(
            f'unsupported dataset_format={values["dataset_format"]!r}; '
            f'use {DATASET_FORMAT_HABITAT!r} or {DATASET_FORMAT_JDROBOT!r}')
    if not str(values['dataset_root']).strip():
        values['dataset_root'] = str(lerobot_root())
    return Config(**values)
