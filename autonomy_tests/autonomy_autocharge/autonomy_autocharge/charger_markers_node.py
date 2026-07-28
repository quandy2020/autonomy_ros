#!/usr/bin/env python3
# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Publish charger / predock markers in the map frame for RViz."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from autonomy_autocharge.geometry_utils import resolve_predock_pose, yaw_to_quaternion


class ChargerMarkersNode(Node):
    """Visualize charging station and predock goal on /autocharge_demo/markers."""

    def __init__(self) -> None:
        super().__init__('charger_markers_node')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('marker_topic', '/autocharge_demo/markers')
        self.declare_parameter('charger_x', 2.0)
        self.declare_parameter('charger_y', 0.0)
        self.declare_parameter('charger_yaw', 0.0)
        self.declare_parameter('use_explicit_predock', True)
        self.declare_parameter('predock_x', 1.4)
        self.declare_parameter('predock_y', -0.08)
        self.declare_parameter('predock_yaw', 0.0)
        self.declare_parameter('predock_distance_m', 0.8)
        self.declare_parameter('publish_hz', 2.0)

        self._map_frame = str(self.get_parameter('map_frame').value)
        topic = str(self.get_parameter('marker_topic').value)
        self._pub = self.create_publisher(MarkerArray, topic, 10)
        hz = max(0.2, float(self.get_parameter('publish_hz').value))
        self.create_timer(1.0 / hz, self._publish)

    def _publish(self) -> None:
        cx = float(self.get_parameter('charger_x').value)
        cy = float(self.get_parameter('charger_y').value)
        cyaw = float(self.get_parameter('charger_yaw').value)
        px, py, pyaw = resolve_predock_pose(
            use_explicit_predock=bool(self.get_parameter('use_explicit_predock').value),
            predock_x=float(self.get_parameter('predock_x').value),
            predock_y=float(self.get_parameter('predock_y').value),
            predock_yaw=float(self.get_parameter('predock_yaw').value),
            charger_x=cx,
            charger_y=cy,
            charger_yaw=cyaw,
            predock_distance_m=float(self.get_parameter('predock_distance_m').value),
        )

        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        arr.markers.append(self._charger_marker(stamp, cx, cy, cyaw))
        arr.markers.append(self._predock_marker(stamp, px, py, pyaw))
        arr.markers.append(self._line_marker(stamp, px, py, cx, cy))
        self._pub.publish(arr)

    def _hdr(self, stamp, marker_id: int, ns: str) -> Marker:
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = self._map_frame
        m.ns = ns
        m.id = marker_id
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.lifetime.sec = 0
        m.lifetime.nanosec = 500_000_000
        return m

    @staticmethod
    def _color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
        c = ColorRGBA()
        c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
        return c

    def _charger_marker(self, stamp, x: float, y: float, yaw: float) -> Marker:
        m = self._hdr(stamp, 0, 'charger')
        m.type = Marker.ARROW
        m.pose = Pose(
            position=Point(x=x, y=y, z=0.05),
            orientation=yaw_to_quaternion(yaw),
        )
        m.scale.x = 0.45
        m.scale.y = 0.08
        m.scale.z = 0.08
        m.color = self._color(0.95, 0.35, 0.15, 0.95)
        return m

    def _predock_marker(self, stamp, x: float, y: float, yaw: float) -> Marker:
        m = self._hdr(stamp, 1, 'predock')
        m.type = Marker.ARROW
        m.pose = Pose(
            position=Point(x=x, y=y, z=0.05),
            orientation=yaw_to_quaternion(yaw),
        )
        m.scale.x = 0.35
        m.scale.y = 0.06
        m.scale.z = 0.06
        m.color = self._color(0.2, 0.75, 1.0, 0.95)
        return m

    def _line_marker(
        self,
        stamp,
        px: float,
        py: float,
        cx: float,
        cy: float,
    ) -> Marker:
        m = self._hdr(stamp, 2, 'predock_path')
        m.type = Marker.LINE_STRIP
        m.scale.x = 0.03
        m.color = self._color(0.2, 0.75, 1.0, 0.5)
        p0 = Point(x=px, y=py, z=0.02)
        p1 = Point(x=cx, y=cy, z=0.02)
        m.points = [p0, p1]
        return m


def main() -> None:
    rclpy.init()
    node = ChargerMarkersNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
