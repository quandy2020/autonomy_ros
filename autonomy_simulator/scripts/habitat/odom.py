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

"""Odometry and TF: map → odom → base_footprint (aligned with urdf/habitat.urdf)."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from habitat.config import Config
from habitat.sim import Session


class OdomPublisher:
    """Publish map→odom→base_footprint TF and /odom."""

    def __init__(self, node: Node, cfg: Config) -> None:
        qos = qos_profile_system_default
        self._cfg = cfg
        self._tf = TransformBroadcaster(node)
        self._static_tf = StaticTransformBroadcaster(node)
        self._odom_pub = node.create_publisher(Odometry, cfg.odom_topic, qos)
        self._pub_map_to_odom_static()

    def publish(
        self,
        stamp: rclpy.time.Time,
        session: Session,
        timed_out: bool = False,
    ) -> None:
        lin, ang = (0.0, 0.0) if timed_out else session.velocity()
        x, y, z, yaw_val = session.odom_pose()
        self._pub_odom_to_base(stamp, x, y, z, yaw_val)
        self._pub_odom(stamp, x, y, z, yaw_val, lin, ang)

    def _pub_map_to_odom_static(self) -> None:
        """map and odom share the same origin (identity, latched for early RViz/Nav2)."""
        msg = TransformStamped()
        msg.header.stamp = rclpy.time.Time().to_msg()
        msg.header.frame_id = self._cfg.map_frame
        msg.child_frame_id = self._cfg.odom_frame
        msg.transform.rotation.w = 1.0
        self._static_tf.sendTransform(msg)

    def _pub_odom_to_base(
        self,
        stamp: rclpy.time.Time,
        x: float,
        y: float,
        z: float,
        yaw_val: float,
    ) -> None:
        half = yaw_val * 0.5
        msg = TransformStamped()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self._cfg.odom_frame
        msg.child_frame_id = self._cfg.base_footprint_frame
        msg.transform.translation.x = float(x)
        msg.transform.translation.y = float(y)
        msg.transform.translation.z = float(z)
        msg.transform.rotation.z = math.sin(half)
        msg.transform.rotation.w = math.cos(half)
        self._tf.sendTransform(msg)

    def _pub_odom(
        self,
        stamp: rclpy.time.Time,
        x: float,
        y: float,
        z: float,
        yaw_val: float,
        lin: float,
        ang: float,
    ) -> None:
        half = yaw_val * 0.5
        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self._cfg.odom_frame
        msg.child_frame_id = self._cfg.base_footprint_frame
        msg.pose.pose.position = Point(x=x, y=y, z=z)
        msg.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(half), w=math.cos(half)
        )
        msg.twist.twist.linear.x = float(lin)
        msg.twist.twist.angular.z = float(ang)
        self._odom_pub.publish(msg)
