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

"""Room bounding box marker for RViz orbit framing (Focus Camera)."""

from __future__ import annotations

from typing import Any

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker

from habitat.config import Config
from habitat.sim import Session


class SceneBoundsPublisher:
    """Latched marker at scene center plus wireframe room bounds."""

    def __init__(self, node: rclpy.node.Node, cfg: Config, session: Session, logger: Any) -> None:
        self._session = session
        self._frame = cfg.map_frame
        self._logger = logger
        self._published = False
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = node.create_publisher(Marker, 'habitat/room_bounds', qos)

    def publish_once(self, stamp: rclpy.time.Time) -> None:
        if self._published:
            return
        lo_x, lo_z, hi_x, hi_z = self._session.room_bounds_xz()
        cx = (lo_x + hi_x) * 0.5
        cz = (lo_z + hi_z) * 0.5
        floor_y = self._session.floor_height
        span = max(hi_x - lo_x, hi_z - lo_z)
        orbit_dist = max(span * 0.65, 8.0)

        corners = [
            (lo_x, floor_y, lo_z),
            (hi_x, floor_y, lo_z),
            (hi_x, floor_y, hi_z),
            (lo_x, floor_y, hi_z),
        ]
        edges = (
            (0, 1), (1, 2), (2, 3), (3, 0),
        )

        bbox = Marker()
        bbox.header.stamp = stamp.to_msg()
        bbox.header.frame_id = self._frame
        bbox.ns = 'habitat_room'
        bbox.id = 0
        bbox.type = Marker.LINE_LIST
        bbox.action = Marker.ADD
        bbox.scale.x = 0.06
        bbox.color.r = 0.2
        bbox.color.g = 0.85
        bbox.color.b = 0.35
        bbox.color.a = 0.9
        for i, j in edges:
            for pt in (corners[i], corners[j]):
                bbox.points.append(_pt(pt))

        center = Marker()
        center.header = bbox.header
        center.ns = 'habitat_room'
        center.id = 1
        center.type = Marker.SPHERE
        center.action = Marker.ADD
        center.pose.position.x = cx
        center.pose.position.y = floor_y + 0.05
        center.pose.position.z = cz
        center.scale.x = center.scale.y = center.scale.z = 0.35
        center.color.r = 1.0
        center.color.g = 0.85
        center.color.b = 0.1
        center.color.a = 0.85

        self._pub.publish(bbox)
        self._pub.publish(center)
        self._published = True
        self._logger.info(
            f'[habitat] RViz orbit hint: focal=({cx:.1f}, {floor_y:.1f}, {cz:.1f}), '
            f'suggested Distance≈{orbit_dist:.1f} m '
            f'(Views panel → Orbit; left-drag rotate, scroll zoom)',
        )


def _pt(xyz: tuple[float, float, float]) -> Any:
    from geometry_msgs.msg import Point

    p = Point()
    p.x, p.y, p.z = xyz
    return p
