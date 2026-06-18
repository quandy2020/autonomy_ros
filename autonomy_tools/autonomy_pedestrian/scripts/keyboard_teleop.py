#!/usr/bin/env python3
# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0

"""Keyboard teleop for standalone pedestrian demos (publishes /robot_state)."""

import math
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.node import Node

MOVE_BINDINGS = {
    'i': (1.0, 0.0),
    'o': (1.0, -1.0),
    'j': (0.0, 1.0),
    'l': (0.0, -1.0),
    'u': (1.0, 1.0),
    ',': (-1.0, 0.0),
    '.': (-1.0, 1.0),
    'm': (-1.0, -1.0),
}

SPEED_BINDINGS = {
    'q': 1.1,
    'z': 0.9,
}

HELP = """
Control the demo robot pose (/robot_state)
------------------------------------------
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease linear speed
k / space : stop
CTRL-C to quit
"""


def yaw_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__('pedestrian_keyboard_teleop')
        self.declare_parameter('robot_x', -8.0)
        self.declare_parameter('robot_y', 0.0)
        self.declare_parameter('robot_yaw', 0.0)
        self.declare_parameter('linear_speed', 0.25)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_hz', 20.0)

        self._x = float(self.get_parameter('robot_x').value)
        self._y = float(self.get_parameter('robot_y').value)
        self._yaw = float(self.get_parameter('robot_yaw').value)
        self._linear_speed = float(self.get_parameter('linear_speed').value)
        self._angular_speed = float(self.get_parameter('angular_speed').value)
        self._frame = self.get_parameter('frame_id').value

        self._move_x = 0.0
        self._move_yaw = 0.0
        self._pub = self.create_publisher(PoseStamped, '/robot_state', 10)
        hz = float(self.get_parameter('publish_hz').value)
        self.create_timer(1.0 / hz, self._on_timer)

    def _on_timer(self) -> None:
        dt = 1.0 / float(self.get_parameter('publish_hz').value)
        self._x += self._move_x * math.cos(self._yaw) * self._linear_speed * dt
        self._y += self._move_x * math.sin(self._yaw) * self._linear_speed * dt
        self._yaw += self._move_yaw * self._angular_speed * dt

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame
        msg.pose.position.x = self._x
        msg.pose.position.y = self._y
        msg.pose.orientation = yaw_quaternion(self._yaw)
        self._pub.publish(msg)

    def apply_key(self, key: str) -> None:
        if key in MOVE_BINDINGS:
            self._move_x, self._move_yaw = MOVE_BINDINGS[key]
        elif key in SPEED_BINDINGS:
            self._linear_speed *= SPEED_BINDINGS[key]
            self.set_parameters([
                rclpy.parameter.Parameter('linear_speed', rclpy.Parameter.Type.DOUBLE, self._linear_speed)
            ])
            self.get_logger().info(f'linear_speed = {self._linear_speed:.2f}')
        elif key in (' ', 'k'):
            self._move_x = 0.0
            self._move_yaw = 0.0


def get_key(settings: termios._AttrReturn) -> str:
    tty.setraw(sys.stdin.fileno())
    readable, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if readable else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main() -> None:
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = KeyboardTeleop()
    print(HELP)
    try:
        while rclpy.ok():
            key = get_key(settings)
            if key == '\x03':
                break
            if key:
                node.apply_key(key)
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
