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

from rclpy.node import Node


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
    local_plan_topic: str = 'local_plan'
    global_costmap_topic: str = 'global_costmap/costmap'
    local_costmap_topic: str = 'local_costmap/costmap'

    use_depth: bool = False
    record_semantic: bool = True
    record_map: bool = True
    record_pointcloud: bool = True
    record_camera_info: bool = True
    record_nav2: bool = True

    image_width: int = 640
    image_height: int = 480
    max_path_waypoints: int = 512
    max_pointcloud_points: int = 4096

    dataset_repo_id: str = 'local/habitat_nav2'
    dataset_root: str = '~/.cache/lerobot/habitat_nav2'
    record_fps: float = 10.0
    task: str = 'navigate to goal'

    # LeRobot video encoding (RGB/semantic -> .mp4 under videos/).
    video_vcodec: str = 'h264'
    streaming_encoding: bool = True
    parallel_video_encoding: bool = True

    @property
    def image_shape(self) -> tuple[int, int, int]:
        return (self.image_height, self.image_width, 3)


_BOOL_FIELDS = frozenset({
    'use_depth', 'record_semantic', 'record_map', 'record_pointcloud',
    'record_camera_info', 'record_nav2',
})
_INT_FIELDS = frozenset({
    'image_width', 'image_height', 'max_path_waypoints', 'max_pointcloud_points',
})


def load(node: Node) -> Config:
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
    values['record_fps'] = float(values['record_fps'])
    return Config(**values)
