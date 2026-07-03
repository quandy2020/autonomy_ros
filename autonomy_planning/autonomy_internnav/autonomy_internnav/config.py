# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ROS 2 parameters for InternNav bridge nodes."""

from __future__ import annotations

from dataclasses import dataclass, fields
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from rclpy.node import Node


@dataclass(frozen=True)
class Config:
    """InternNav bridge configuration loaded from ROS parameters."""

    policy: str = 'navdp'
    goal_type: str = 'point'
    checkpoint: str = ''
    device: str = 'cuda:0'

    model_config: str = ''
    robot_config: str = ''
    data_config: str = ''
    m2f_checkpoint: str = ''
    m2f_config: str = ''

    rgb_topic: str = 'camera/rgb/image_raw'
    depth_topic: str = 'camera/depth/image_raw'
    camera_info_topic: str = 'camera/rgb/camera_info'
    odom_topic: str = 'odom'
    goal_topic: str = 'goal_pose'
    image_goal_topic: str = 'image_goal'
    pixel_goal_topic: str = 'pixel_goal'
    cmd_vel_topic: str = 'cmd_vel'
    path_topic: str = 'navdp/plan'
    markers_topic: str = 'navdp/markers'
    overlay_topic: str = 'navdp/overlay'
    overlay_rate_hz: float = 20.0
    overlay_decouple: bool = True
    overlay_show_all_samples: bool = True
    overlay_max_samples: int = 16

    base_frame: str = 'base_link'
    map_frame: str = 'map'
    markers_frame: str = ''
    marker_lifetime_sec: float = 0.0
    markers_show_all_samples: bool = True
    markers_use_latest_tf: bool = True
    path_frame: str = ''
    inference_seed: int = -1

    image_size: int = 224
    memory_size: int = 8
    predict_size: int = 24
    temporal_depth: int = 16
    heads: int = 8
    token_dim: int = 384
    stop_threshold: float = -3.0
    batch_size: int = 1
    sample_num: int = 16

    inference_rate_hz: float = 10.0
    linear_speed: float = 0.3
    angular_speed: float = 0.5
    lookahead_index: int = 3
    depth_encoding: str = '32fc1'


def _param(node: Node, name: str, default: Any) -> Any:
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    return node.get_parameter(name).value


def load(node: Node) -> Config:
    """Load configuration from ROS parameters declared on ``node``."""
    defaults = Config()
    values = {
        field.name: _param(node, field.name, getattr(defaults, field.name))
        for field in fields(Config)
    }
    return Config(**values)
