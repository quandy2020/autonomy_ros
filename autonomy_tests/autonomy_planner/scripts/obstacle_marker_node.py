#!/usr/bin/env python3
#
# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""InteractiveMarker static/dynamic obstacles → PointCloud2 for planner sim."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, Quaternion
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)


@dataclass
class Obstacle:
    name: str
    x: float
    y: float
    radius: float
    height: float
    dynamic: bool = False
    orbit_cx: float = 0.0
    orbit_cy: float = 0.0
    orbit_r: float = 0.0
    orbit_omega: float = 0.0
    orbit_phase: float = 0.0
    color: tuple[float, float, float, float] = (1.0, 0.2, 0.1, 0.85)


@dataclass
class ObstacleWorld:
    obstacles: list[Obstacle] = field(default_factory=list)

    def sample_cloud(self, resolution: float = 0.05) -> np.ndarray:
        pts: list[list[float]] = []
        for obs in self.obstacles:
            n_theta = max(8, int(2.0 * math.pi * obs.radius / resolution))
            n_z = max(2, int(obs.height / resolution))
            for iz in range(n_z + 1):
                z = iz * obs.height / max(n_z, 1)
                for it in range(n_theta):
                    th = 2.0 * math.pi * it / n_theta
                    pts.append(
                        [
                            obs.x + obs.radius * math.cos(th),
                            obs.y + obs.radius * math.sin(th),
                            z,
                        ]
                    )
            n_r = max(1, int(obs.radius / resolution))
            for ir in range(n_r + 1):
                r = ir * obs.radius / max(n_r, 1)
                n_t = max(1, int(2.0 * math.pi * max(r, resolution) / resolution))
                for it in range(n_t):
                    th = 2.0 * math.pi * it / n_t
                    pts.append([obs.x + r * math.cos(th), obs.y + r * math.sin(th), 0.05])
        if not pts:
            return np.zeros((0, 3), dtype=np.float32)
        return np.asarray(pts, dtype=np.float32)


def _pose(x: float, y: float, z: float = 0.0) -> Pose:
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
    return p


def _cloud_msg(frame: str, stamp, xyz: np.ndarray) -> PointCloud2:
    msg = PointCloud2()
    msg.header = Header(stamp=stamp, frame_id=frame)
    msg.height = 1
    msg.width = int(xyz.shape[0])
    msg.is_dense = True
    msg.is_bigendian = False
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.data = xyz.astype(np.float32).tobytes()
    return msg


class ObstacleMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__('obstacle_marker_node')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('cloud_topic', 'planner_sim/obstacle_cloud')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('sample_resolution', 0.08)
        self.declare_parameter('static_count', 3)
        self.declare_parameter('dynamic_count', 1)

        self._frame = self.get_parameter('frame_id').value
        self._res = float(self.get_parameter('sample_resolution').value)
        rate = float(self.get_parameter('publish_rate_hz').value)

        self._world = ObstacleWorld()
        self._seed_obstacles(
            int(self.get_parameter('static_count').value),
            int(self.get_parameter('dynamic_count').value),
        )

        self._cloud_pub = self.create_publisher(
            PointCloud2, self.get_parameter('cloud_topic').value, 10
        )
        self._server = InteractiveMarkerServer(self, 'planner_obstacles')
        for obs in self._world.obstacles:
            self._insert_marker(obs)
        self._server.applyChanges()

        self._t0 = self.get_clock().now()
        self.create_timer(1.0 / max(rate, 1.0), self._on_timer)
        self.get_logger().info(
            f'Obstacle markers ready: {len(self._world.obstacles)} '
            f'(static+dynamic) → {self.get_parameter("cloud_topic").value}'
        )

    def _seed_obstacles(self, n_static: int, n_dynamic: int) -> None:
        static_xy = [(2.5, 1.0), (-2.0, 2.0), (1.5, -2.5), (-2.5, -1.5), (3.0, -1.0)]
        for i in range(max(0, n_static)):
            x, y = static_xy[i % len(static_xy)]
            self._world.obstacles.append(
                Obstacle(
                    name=f'static_{i}',
                    x=x,
                    y=y,
                    radius=0.35,
                    height=1.2,
                    dynamic=False,
                    color=(0.9, 0.25, 0.1, 0.9),
                )
            )
        for i in range(max(0, n_dynamic)):
            phase = i * (2.0 * math.pi / max(n_dynamic, 1))
            self._world.obstacles.append(
                Obstacle(
                    name=f'dynamic_{i}',
                    x=3.5 * math.cos(phase),
                    y=3.5 * math.sin(phase),
                    radius=0.28,
                    height=1.0,
                    dynamic=True,
                    orbit_cx=0.0,
                    orbit_cy=0.0,
                    orbit_r=3.5 + 0.15 * i,
                    orbit_omega=0.25 * (1.0 if i % 2 == 0 else -1.0),
                    orbit_phase=phase,
                    color=(0.1, 0.55, 1.0, 0.9),
                )
            )

    def _cylinder_marker(self, obs: Obstacle) -> Marker:
        m = Marker()
        m.type = Marker.CYLINDER
        m.scale.x = 2.0 * obs.radius
        m.scale.y = 2.0 * obs.radius
        m.scale.z = obs.height
        m.color.r, m.color.g, m.color.b, m.color.a = obs.color
        m.pose.position.z = 0.5 * obs.height
        m.pose.orientation.w = 1.0
        return m

    def _insert_marker(self, obs: Obstacle) -> None:
        im = InteractiveMarker()
        im.header.frame_id = self._frame
        im.name = obs.name
        if obs.dynamic:
            im.description = f'DYNAMIC {obs.name} (drag cylinder to move orbit center)'
        else:
            im.description = f'STATIC {obs.name} (RViz Interact: drag cylinder in XY)'
        im.scale = max(0.6, 2.2 * obs.radius)
        im.pose = _pose(obs.x, obs.y)

        body = InteractiveMarkerControl()
        body.name = 'move_xy'
        body.always_visible = True
        body.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        body.orientation = Quaternion(w=1.0, x=0.0, y=1.0, z=0.0)
        body.markers.append(self._cylinder_marker(obs))
        im.controls.append(body)

        self._server.insert(im, feedback_callback=self._on_feedback)

    def _find(self, name: str) -> Obstacle | None:
        for obs in self._world.obstacles:
            if obs.name == name:
                return obs
        return None

    def _on_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        if feedback.event_type not in (
            InteractiveMarkerFeedback.POSE_UPDATE,
            InteractiveMarkerFeedback.MOUSE_UP,
        ):
            return
        obs = self._find(feedback.marker_name)
        if obs is None:
            return
        obs.x = float(feedback.pose.position.x)
        obs.y = float(feedback.pose.position.y)
        if obs.dynamic:
            obs.orbit_cx = obs.x
            obs.orbit_cy = obs.y
            obs.orbit_phase = 0.0
            self._t0 = self.get_clock().now()
        else:
            self._server.setPose(obs.name, _pose(obs.x, obs.y))
            self._server.applyChanges()
            self._publish_cloud()

    def _publish_cloud(self) -> None:
        xyz = self._world.sample_cloud(self._res)
        self._cloud_pub.publish(
            _cloud_msg(self._frame, self.get_clock().now().to_msg(), xyz)
        )

    def _on_timer(self) -> None:
        t = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
        dynamic_moved = False
        for obs in self._world.obstacles:
            if not obs.dynamic:
                continue
            dynamic_moved = True
            ang = obs.orbit_phase + obs.orbit_omega * t
            obs.x = obs.orbit_cx + obs.orbit_r * math.cos(ang)
            obs.y = obs.orbit_cy + obs.orbit_r * math.sin(ang)
            self._server.setPose(obs.name, _pose(obs.x, obs.y))
        if dynamic_moved:
            self._server.applyChanges()
        self._publish_cloud()


def main() -> None:
    rclpy.init()
    node = ObstacleMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
