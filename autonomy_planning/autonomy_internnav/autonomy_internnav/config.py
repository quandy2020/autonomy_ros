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

"""ROS 2 parameters for NavDP inference."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from rclpy.node import Node


@dataclass(frozen=True)
class Config:
    """InternNav bridge configuration."""

    policy: str = 'navdp'
    goal_type: str = 'point'  # point | image | pixel | point_image | nogoal
    checkpoint: str = ''
    device: str = 'cuda:0'

    # Optional baseline config overrides (empty = use packaged defaults)
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

    base_frame: str = 'base_link'
    map_frame: str = 'map'
    markers_frame: str = ''  # empty → base_frame（机体系，避免 map 系下 TF/重采样抖动）
    marker_lifetime_sec: float = 0.5
    path_frame: str = ''  # empty → base_frame

    image_size: int = 224
    memory_size: int = 8
    predict_size: int = 24
    temporal_depth: int = 16
    heads: int = 8
    token_dim: int = 384
    stop_threshold: float = -3.0
    batch_size: int = 1
    sample_num: int = 16

    inference_rate_hz: float = 5.0
    linear_speed: float = 0.3
    angular_speed: float = 0.5
    lookahead_index: int = 3
    # depth encoding: 32fc1 (meters), 16uc1 (mm), 16uc1_10k (NavDP server /10000)
    depth_encoding: str = '32fc1'


def _param(node: Node, name: str, default: Any) -> Any:
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    return node.get_parameter(name).value


def load(node: Node) -> Config:
    """Load configuration from ROS parameters."""
    defaults = Config()
    return Config(
        policy=_param(node, 'policy', defaults.policy),
        goal_type=_param(node, 'goal_type', defaults.goal_type),
        checkpoint=_param(node, 'checkpoint', defaults.checkpoint),
        device=_param(node, 'device', defaults.device),
        model_config=_param(node, 'model_config', defaults.model_config),
        robot_config=_param(node, 'robot_config', defaults.robot_config),
        data_config=_param(node, 'data_config', defaults.data_config),
        m2f_checkpoint=_param(node, 'm2f_checkpoint', defaults.m2f_checkpoint),
        m2f_config=_param(node, 'm2f_config', defaults.m2f_config),
        rgb_topic=_param(node, 'rgb_topic', defaults.rgb_topic),
        depth_topic=_param(node, 'depth_topic', defaults.depth_topic),
        camera_info_topic=_param(node, 'camera_info_topic', defaults.camera_info_topic),
        odom_topic=_param(node, 'odom_topic', defaults.odom_topic),
        goal_topic=_param(node, 'goal_topic', defaults.goal_topic),
        image_goal_topic=_param(node, 'image_goal_topic', defaults.image_goal_topic),
        pixel_goal_topic=_param(node, 'pixel_goal_topic', defaults.pixel_goal_topic),
        cmd_vel_topic=_param(node, 'cmd_vel_topic', defaults.cmd_vel_topic),
        path_topic=_param(node, 'path_topic', defaults.path_topic),
        markers_topic=_param(node, 'markers_topic', defaults.markers_topic),
        overlay_topic=_param(node, 'overlay_topic', defaults.overlay_topic),
        base_frame=_param(node, 'base_frame', defaults.base_frame),
        map_frame=_param(node, 'map_frame', defaults.map_frame),
        markers_frame=_param(node, 'markers_frame', defaults.markers_frame),
        marker_lifetime_sec=_param(node, 'marker_lifetime_sec', defaults.marker_lifetime_sec),
        path_frame=_param(node, 'path_frame', defaults.path_frame),
        image_size=_param(node, 'image_size', defaults.image_size),
        memory_size=_param(node, 'memory_size', defaults.memory_size),
        predict_size=_param(node, 'predict_size', defaults.predict_size),
        temporal_depth=_param(node, 'temporal_depth', defaults.temporal_depth),
        heads=_param(node, 'heads', defaults.heads),
        token_dim=_param(node, 'token_dim', defaults.token_dim),
        stop_threshold=_param(node, 'stop_threshold', defaults.stop_threshold),
        batch_size=_param(node, 'batch_size', defaults.batch_size),
        sample_num=_param(node, 'sample_num', defaults.sample_num),
        inference_rate_hz=_param(node, 'inference_rate_hz', defaults.inference_rate_hz),
        linear_speed=_param(node, 'linear_speed', defaults.linear_speed),
        angular_speed=_param(node, 'angular_speed', defaults.angular_speed),
        lookahead_index=_param(node, 'lookahead_index', defaults.lookahead_index),
        depth_encoding=_param(node, 'depth_encoding', defaults.depth_encoding),
    )
