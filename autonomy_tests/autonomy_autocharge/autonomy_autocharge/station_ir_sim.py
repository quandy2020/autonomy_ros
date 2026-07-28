#!/usr/bin/env python3
# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""TF-based IR / charge simulation for Habitat + Nav2 (no cmd_vel integration)."""

from __future__ import annotations

import math
from typing import Dict, List, Tuple

import rclpy
from geometry_msgs.msg import Point, TransformStamped
from jdbot_interfaces.msg import DockIR
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from autocharge.common.ir_sensor import IRSensorArray, Pose2D, wrap_pi
from autocharge.sim.env_sim import EnvSimNode


class StationIrSimNode(Node):
    """Publish /ir/dock and /dock/charge_connected from map-frame charger pose + TF."""

    def __init__(self) -> None:
        super().__init__('station_ir_sim')
        self._declare_parameters()
        self._load_config()
        self._build_ir_model()
        self._emitters = self._load_emitters()
        self._charge_connected = False
        self._charge_contact_elapsed_s = 0.0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._ir_pub = self.create_publisher(DockIR, self._ir_topic, 10)
        self._charge_pub = self.create_publisher(Bool, self._charge_topic, 10)
        self._marker_pub = self.create_publisher(MarkerArray, self._marker_topic, 10)

        hz = max(1.0, float(self.get_parameter('update_hz').value))
        self.create_timer(1.0 / hz, self._tick)

    def _declare_parameters(self) -> None:
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('dock_frame', 'dock')
        self.declare_parameter('publish_dock_tf', True)
        self.declare_parameter('charger_x', 2.0)
        self.declare_parameter('charger_y', 0.0)
        self.declare_parameter('charger_yaw', math.pi)
        self.declare_parameter('update_hz', 20.0)
        self.declare_parameter('marker_topic', '/env_sim/markers')
        self.declare_parameter('charge_topic', '/dock/charge_connected')
        self.declare_parameter('ir_sensors.topic', '/ir/dock')
        self.declare_parameter('ir_sensors.publish', True)
        self.declare_parameter('ir_sensors.swap_channels', True)
        self.declare_parameter('ir_sensors.use_dock_emitters', True)
        self.declare_parameter('station_visualize', True)
        self.declare_parameter('charge_success_dist_m', 0.038)
        self.declare_parameter('charge_success_yaw_tol_deg', 8.0)
        self.declare_parameter('charge_hold_s', 0.20)
        self.declare_parameter('robot_length_m', 0.30)
        self.declare_parameter('robot_width_m', 0.40)
        # Receivers / emitters aligned with autocharge config/sim.yaml
        self.declare_parameter('ir_sensors.receivers.left.x_offset', 0.16)
        self.declare_parameter('ir_sensors.receivers.left.y_offset', -0.00545)
        self.declare_parameter('ir_sensors.receivers.left.fov', 15.0)
        self.declare_parameter('ir_sensors.receivers.left.yaw', -7.5)
        self.declare_parameter('ir_sensors.receivers.left.visible', True)
        self.declare_parameter('ir_sensors.receivers.left.mode', 'normal')
        self.declare_parameter('ir_sensors.receivers.left.color', 'cyan')
        self.declare_parameter('ir_sensors.receivers.left.max_length_m', 1.5)
        self.declare_parameter('ir_sensors.receivers.left.alpha', 0.4)
        self.declare_parameter('ir_sensors.receivers.left.fill_alpha', 0.20)
        self.declare_parameter('ir_sensors.receivers.right.x_offset', 0.16)
        self.declare_parameter('ir_sensors.receivers.right.y_offset', 0.00545)
        self.declare_parameter('ir_sensors.receivers.right.fov', 15.0)
        self.declare_parameter('ir_sensors.receivers.right.yaw', 7.5)
        self.declare_parameter('ir_sensors.receivers.right.visible', True)
        self.declare_parameter('ir_sensors.receivers.right.mode', 'normal')
        self.declare_parameter('ir_sensors.receivers.right.color', 'cyan')
        self.declare_parameter('ir_sensors.receivers.right.max_length_m', 1.5)
        self.declare_parameter('ir_sensors.receivers.right.alpha', 0.4)
        self.declare_parameter('ir_sensors.receivers.right.fill_alpha', 0.20)
        for name, yaw, y, color in (
            ('far_left', 20.0, 0.01425, 'red'),
            ('left', 5.0, 0.006, 'cyan'),
            ('center', 0.0, 0.0, 'green'),
            ('right', -5.0, -0.006, 'magenta'),
            ('far_right', -20.0, -0.01425, 'red'),
        ):
            base = f'ir_sensors.emitters.{name}'
            self.declare_parameter(f'{base}.x', 0.0)
            self.declare_parameter(f'{base}.y', y)
            self.declare_parameter(f'{base}.yaw', yaw)
            self.declare_parameter(f'{base}.fov', 30.0 if name in ('far_left', 'far_right') else 15.0)
            self.declare_parameter(f'{base}.visible', True)
            self.declare_parameter(
                f'{base}.mode',
                'trapezoid' if name == 'center' else 'normal',
            )
            self.declare_parameter(f'{base}.color', color)
            self.declare_parameter(f'{base}.alpha', 0.3)
            self.declare_parameter(f'{base}.max_length_m', 1.5)
            self.declare_parameter(f'{base}.trap_length_m', 0.03)
            self.declare_parameter(f'{base}.trap_width_m', 0.002)
        self.declare_parameter('bit_l1', 0x01)
        self.declare_parameter('bit_l2', 0x02)
        self.declare_parameter('bit_c', 0x04)
        self.declare_parameter('bit_r1', 0x08)
        self.declare_parameter('bit_r2', 0x10)

    def _load_config(self) -> None:
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._dock_frame = str(self.get_parameter('dock_frame').value)
        self._publish_dock_tf = bool(self.get_parameter('publish_dock_tf').value)
        self._station_x = float(self.get_parameter('charger_x').value)
        self._station_y = float(self.get_parameter('charger_y').value)
        self._station_yaw = float(self.get_parameter('charger_yaw').value)
        self._marker_topic = str(self.get_parameter('marker_topic').value)
        self._charge_topic = str(self.get_parameter('charge_topic').value)
        self._ir_topic = str(self.get_parameter('ir_sensors.topic').value)
        self._ir_publish = bool(self.get_parameter('ir_sensors.publish').value)
        self._ir_swap_channels = bool(self.get_parameter('ir_sensors.swap_channels').value)
        self._use_dock_emitters = bool(self.get_parameter('ir_sensors.use_dock_emitters').value)
        self._station_visualize = bool(self.get_parameter('station_visualize').value)
        self._charge_success_dist_m = float(self.get_parameter('charge_success_dist_m').value)
        self._charge_success_yaw_tol_rad = math.radians(
            float(self.get_parameter('charge_success_yaw_tol_deg').value)
        )
        self._charge_hold_s = float(self.get_parameter('charge_hold_s').value)
        self._vis_left = bool(self.get_parameter('ir_sensors.receivers.left.visible').value)
        self._vis_right = bool(self.get_parameter('ir_sensors.receivers.right.visible').value)

    def _build_ir_model(self) -> None:
        p = self.get_parameter
        robot_len = float(p('robot_length_m').value)
        robot_w = float(p('robot_width_m').value)
        left_x = float(p('ir_sensors.receivers.left.x_offset').value)
        left_y = float(p('ir_sensors.receivers.left.y_offset').value)
        right_x = float(p('ir_sensors.receivers.right.x_offset').value)
        right_y = float(p('ir_sensors.receivers.right.y_offset').value)
        self._ir_model = IRSensorArray(
            robot_length_m=robot_len,
            robot_width_m=robot_w,
            use_explicit_pair_pose=True,
            left_x_m=left_x,
            left_y_m=left_y,
            left_yaw_deg=float(p('ir_sensors.receivers.left.yaw').value),
            left_fov_deg=float(p('ir_sensors.receivers.left.fov').value),
            left_mode=str(p('ir_sensors.receivers.left.mode').value),
            right_x_m=right_x,
            right_y_m=right_y,
            right_yaw_deg=float(p('ir_sensors.receivers.right.yaw').value),
            right_fov_deg=float(p('ir_sensors.receivers.right.fov').value),
            right_mode=str(p('ir_sensors.receivers.right.mode').value),
            left_range_m=float(p('ir_sensors.receivers.left.max_length_m').value),
            right_range_m=float(p('ir_sensors.receivers.right.max_length_m').value),
            bit_left=int(p('bit_l1').value),
            bit_center=int(p('bit_c').value),
            bit_right=int(p('bit_r1').value),
        )
        self._bit_l1 = int(p('bit_l1').value)
        self._bit_l2 = int(p('bit_l2').value)
        self._bit_c = int(p('bit_c').value)
        self._bit_r1 = int(p('bit_r1').value)
        self._bit_r2 = int(p('bit_r2').value)
        self._name_to_token = {
            'far_left': self._bit_l1,
            'left': self._bit_l2,
            'center': self._bit_c,
            'right': self._bit_r1,
            'far_right': self._bit_r2,
        }

    def _load_emitters(self) -> List[Dict[str, float]]:
        out: List[Dict[str, float]] = []
        for name in ('far_left', 'left', 'center', 'right', 'far_right'):
            base = f'ir_sensors.emitters.{name}'
            out.append(
                {
                    'name': name,
                    'x': float(self.get_parameter(f'{base}.x').value),
                    'y': float(self.get_parameter(f'{base}.y').value),
                    'yaw': math.radians(float(self.get_parameter(f'{base}.yaw').value)),
                    'fov': max(0.1, float(self.get_parameter(f'{base}.fov').value)),
                    'visible': bool(self.get_parameter(f'{base}.visible').value),
                    'mode': str(self.get_parameter(f'{base}.mode').value).strip().lower(),
                    'max_length_m': max(
                        0.05, float(self.get_parameter(f'{base}.max_length_m').value)
                    ),
                    'trap_length_m': max(
                        0.0, float(self.get_parameter(f'{base}.trap_length_m').value)
                    ),
                    'trap_width_m': max(
                        0.0, float(self.get_parameter(f'{base}.trap_width_m').value)
                    ),
                }
            )
        return out

    def _lookup_robot_pose(self) -> Pose2D | None:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                rclpy.time.Time(),
            )
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return Pose2D(x=float(t.x), y=float(t.y), yaw=float(yaw))

    def _tick(self) -> None:
        pose = self._lookup_robot_pose()
        if pose is None:
            return
        stamp = self.get_clock().now()
        if self._publish_dock_tf:
            self._publish_dock_tf_msg(stamp.to_msg())
        if self._ir_publish:
            self._publish_ir(pose, stamp)
        self._update_charge_contact(pose, 1.0 / max(1.0, float(self.get_parameter('update_hz').value)))
        if self._station_visualize:
            self._publish_markers(pose, stamp)

    def _publish_dock_tf_msg(self, stamp_msg) -> None:
        half = 0.5 * self._station_yaw
        tfm = TransformStamped()
        tfm.header.stamp = stamp_msg
        tfm.header.frame_id = self._map_frame
        tfm.child_frame_id = self._dock_frame
        tfm.transform.translation.x = self._station_x
        tfm.transform.translation.y = self._station_y
        tfm.transform.translation.z = 0.0
        tfm.transform.rotation.z = math.sin(half)
        tfm.transform.rotation.w = math.cos(half)
        self._tf_broadcaster.sendTransform(tfm)

    def _emitters_world(self) -> List[Dict[str, float]]:
        if not self._use_dock_emitters:
            return []
        tx = self._station_x
        ty = self._station_y
        dock_yaw = self._station_yaw
        c = math.cos(dock_yaw)
        s = math.sin(dock_yaw)
        out: List[Dict[str, float]] = []
        for emitter in self._emitters:
            if not bool(emitter['visible']):
                continue
            ex = tx + c * emitter['x'] - s * emitter['y']
            ey = ty + s * emitter['x'] + c * emitter['y']
            out.append(
                {
                    **emitter,
                    'wx': ex,
                    'wy': ey,
                    'wyaw': dock_yaw + emitter['yaw'],
                }
            )
        return out

    def _publish_ir(self, robot_pose: Pose2D, stamp) -> None:
        left_xy, right_xy = self._ir_model.sensor_positions_world(robot_pose)
        emitters_w = self._emitters_world()
        left_hits = self._hits_for_receiver(
            left_xy, self._ir_model.left_sensor.yaw, self._ir_model.left_sensor, emitters_w, robot_pose.yaw
        )
        right_hits = self._hits_for_receiver(
            right_xy, self._ir_model.right_sensor.yaw, self._ir_model.right_sensor, emitters_w, robot_pose.yaw
        )
        ltok = self._token_from_hits(left_hits)
        rtok = self._token_from_hits(right_hits)
        msg = DockIR()
        msg.header.stamp = stamp.to_msg()
        if self._ir_swap_channels:
            msg.left = int(rtok) & 0xFF
            msg.right = int(ltok) & 0xFF
        else:
            msg.left = int(ltok) & 0xFF
            msg.right = int(rtok) & 0xFF
        msg.code = 0
        msg.message = 'ok' if (ltok or rtok) else 'no_hit'
        self._ir_pub.publish(msg)

    def _hits_for_receiver(
        self,
        receiver_xy: Tuple[float, float],
        receiver_yaw_rel: float,
        receiver,
        emitters_w: List[Dict[str, float]],
        robot_yaw: float,
    ) -> List[Dict[str, float]]:
        hits: List[Dict[str, float]] = []
        rx, ry = receiver_xy
        r_heading = robot_yaw + float(receiver_yaw_rel)
        for emitter in emitters_w:
            in_recv, dist = EnvSimNode._point_in_receiver_fov(
                rx,
                ry,
                r_heading,
                receiver,
                float(emitter['wx']),
                float(emitter['wy']),
            )
            if not in_recv:
                continue
            if not EnvSimNode._point_in_emitter_fov(rx, ry, emitter):
                continue
            hit = dict(emitter)
            hit['dist'] = dist
            hits.append(hit)
        return hits

    def _token_from_hits(self, hits: List[Dict[str, float]]) -> int:
        if not hits:
            return 0
        token = 0
        for hit in hits:
            name = str(hit.get('name', ''))
            if name in self._name_to_token:
                token |= int(self._name_to_token[name]) & 0xFF
        return int(token) & 0xFF

    def _robot_front_ir_world(self, pose: Pose2D) -> Tuple[float, float, float]:
        left_xy, right_xy = self._ir_model.sensor_positions_world(pose)
        center_x = 0.5 * (float(left_xy[0]) + float(right_xy[0]))
        center_y = 0.5 * (float(left_xy[1]) + float(right_xy[1]))
        return (center_x, center_y, float(pose.yaw))

    def _dock_center_emitter_world(self) -> Tuple[float, float, float]:
        for emitter in self._emitters_world():
            if str(emitter.get('name', '')) == 'center':
                return (
                    float(emitter['wx']),
                    float(emitter['wy']),
                    float(emitter['wyaw']),
                )
        return (self._station_x, self._station_y, self._station_yaw)

    def _is_charge_contact(self, pose: Pose2D) -> bool:
        ir_x, ir_y, ir_yaw = self._robot_front_ir_world(pose)
        dock_x, dock_y, dock_yaw = self._dock_center_emitter_world()
        dist = math.hypot(ir_x - dock_x, ir_y - dock_y)
        yaw_err = abs(abs(wrap_pi(ir_yaw - dock_yaw)) - math.pi)
        return (
            dist <= self._charge_success_dist_m
            and yaw_err <= self._charge_success_yaw_tol_rad
        )

    def _update_charge_contact(self, pose: Pose2D, dt: float) -> None:
        if self._is_charge_contact(pose):
            self._charge_contact_elapsed_s += max(0.0, float(dt))
        else:
            self._charge_contact_elapsed_s = 0.0
        connected = self._charge_contact_elapsed_s >= self._charge_hold_s
        if connected != self._charge_connected:
            self._charge_connected = connected
            self.get_logger().info(
                'charge_connected=%s' % str(self._charge_connected).lower()
            )
        msg = Bool()
        msg.data = bool(self._charge_connected)
        self._charge_pub.publish(msg)

    def _publish_markers(self, pose: Pose2D, stamp) -> None:
        mk = self._ir_model.build_markers(
            pose,
            frame_id=self._map_frame,
            stamp=stamp.to_msg(),
            ns='env_sim',
            visualize_left=self._vis_left,
            visualize_right=self._vis_right,
        )
        mk.markers.extend(self._station_markers(stamp))
        self._marker_pub.publish(mk)

    def _station_markers(self, stamp) -> List[Marker]:
        # Reuse env_sim station marker styling via simplified inline markers.
        arr: List[Marker] = []
        m = Marker()
        m.header.stamp = stamp.to_msg()
        m.header.frame_id = self._map_frame
        m.ns = 'charging_station'
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = self._station_x
        m.pose.position.y = self._station_y
        m.pose.position.z = 0.08
        half = 0.5 * self._station_yaw
        m.pose.orientation.z = math.sin(half)
        m.pose.orientation.w = math.cos(half)
        m.scale.x = 0.12
        m.scale.y = 0.25
        m.scale.z = 0.16
        m.color.r = 0.9
        m.color.g = 0.45
        m.color.b = 0.1
        m.color.a = 0.85
        m.lifetime.sec = 0
        m.lifetime.nanosec = 200_000_000
        arr.append(m)
        return arr


def main() -> None:
    rclpy.init()
    node = StationIrSimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
