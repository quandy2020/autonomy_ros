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

"""Publish Habitat RGB, depth, and semantic camera images to ROS 2."""

from __future__ import annotations

import array
import math
from typing import Any

import numpy as np
import rclpy
from habitat_sim.utils.viz_utils import semantic_to_rgb
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import CameraInfo, Image

from habitat.config import Config

# Habitat tutorial sensor uuids (TUTORIALS/scripts/sim_utils.py).
_SENSOR_KEYS = {
    'rgb': ('color_sensor', 'rgb'),
    'depth': ('depth_sensor', 'depth'),
    'semantic': ('semantic_sensor', 'semantic'),
}


def _squeeze2d(arr: np.ndarray) -> np.ndarray:
    return arr.squeeze(-1) if arr.ndim == 3 else arr.squeeze()


class CameraPublisher:
    """Publish rgb, depth, and semantic images (habitat-sim tutorial style)."""

    def __init__(self, node: Node, cfg: Config) -> None:
        qos = qos_profile_system_default
        self._cfg = cfg
        self._logger = node.get_logger()
        self._semantic_warned = False
        self._pubs = {
            'rgb': node.create_publisher(Image, cfg.rgb_topic, qos),
            'depth': node.create_publisher(Image, cfg.depth_topic, qos),
            'semantic': node.create_publisher(Image, cfg.semantic_topic, qos),
        }
        self._info_pub = node.create_publisher(
            CameraInfo, cfg.rgb_camera_info_topic, qos
        )
        self._keys: dict[str, str | None] = {name: None for name in _SENSOR_KEYS}

    def publish(self, stamp: rclpy.time.Time, obs: dict[str, Any]) -> None:
        self._pub_rgb(stamp, obs)
        self._pub_depth(stamp, obs)
        self._pub_sem(stamp, obs)

    def _find_key(self, obs: dict[str, Any], name: str) -> str | None:
        if self._keys[name]:
            return self._keys[name]
        for key in _SENSOR_KEYS[name]:
            if key in obs:
                self._keys[name] = key
                return key
            match = next((k for k in obs if k == key or k.endswith(key)), None)
            if match:
                self._keys[name] = match
                return match
        return None

    def _pub_camera_info(self, stamp: rclpy.time.Time) -> None:
        cfg = self._cfg
        w, h = cfg.image_width, cfg.image_height
        fx = w / (2.0 * math.tan(math.radians(cfg.camera_horizontal_fov_deg / 2.0)))
        cx, cy = w / 2.0, h / 2.0
        info = CameraInfo()
        info.header.stamp = stamp.to_msg()
        info.header.frame_id = cfg.rgb_camera_frame
        info.width = w
        info.height = h
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx, 0.0, fx, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fx, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._info_pub.publish(info)

    def _send(
        self,
        stamp: rclpy.time.Time,
        frame_id: str,
        image: np.ndarray,
        encoding: str,
        name: str,
    ) -> None:
        arr = np.ascontiguousarray(image)
        msg = Image()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = frame_id
        msg.encoding = encoding
        msg.is_bigendian = False
        if arr.ndim == 2:
            msg.height, msg.width = arr.shape
            msg.step = msg.width * arr.dtype.itemsize
        else:
            msg.height, msg.width = arr.shape[:2]
            msg.step = msg.width * int(np.dtype(arr.dtype).itemsize * arr.shape[2])
        # array.array('B', ...) avoids per-byte validation in sensor_msgs Image.data.
        msg.data = array.array('B', arr.tobytes())
        self._pubs[name].publish(msg)

    def _pub_rgb(self, stamp: rclpy.time.Time, obs: dict[str, Any]) -> None:
        key = self._find_key(obs, 'rgb')
        if key is None:
            return

        self._pub_camera_info(stamp)
        rgba = np.asarray(obs[key])
        rgb = np.ascontiguousarray(rgba[:, :, :3], dtype=np.uint8)
        self._send(stamp, self._cfg.rgb_camera_frame, rgb, 'rgb8', 'rgb')

    def _pub_depth(self, stamp: rclpy.time.Time, obs: dict[str, Any]) -> None:
        key = self._find_key(obs, 'depth')
        if key is None:
            return

        depth = np.ascontiguousarray(
            _squeeze2d(np.asarray(obs[key], dtype=np.float32))
        )
        self._send(stamp, self._cfg.depth_camera_frame, depth, '32FC1', 'depth')

    def _pub_sem(self, stamp: rclpy.time.Time, obs: dict[str, Any]) -> None:
        key = self._find_key(obs, 'semantic')
        if key is None:
            return

        ids = np.ascontiguousarray(_squeeze2d(np.asarray(obs[key])), dtype=np.uint32)
        if not self._semantic_warned and ids.max() == 0:
            self._logger.warning(
                'Semantic buffer is all zeros; scene may lack semantic annotations.'
            )
            self._semantic_warned = True

        colored = np.asarray(semantic_to_rgb(ids))[:, :, :3].astype(np.uint8)
        self._send(
            stamp, self._cfg.semantic_camera_frame, colored, 'rgb8', 'semantic'
        )
