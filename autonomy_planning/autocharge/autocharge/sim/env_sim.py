"""Simple cmd_vel robot simulator with odom/tf and IR markers."""

from __future__ import annotations

import math
import random
from typing import Dict, List, Optional, Tuple

from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist
from jdbot_interfaces.msg import DockIR
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from autocharge.common.ir_sensor import IRSensorArray, Pose2D, wrap_pi


def _yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    """Returns quaternion (x, y, z, w) for planar yaw."""
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Returns planar yaw from quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _parse_color(name: str, *, default: Tuple[float, float, float]) -> Tuple[float, float, float]:
    t = str(name).strip().lower()
    named = {
        'red': (1.0, 0.1, 0.1),
        'green': (0.1, 1.0, 0.1),
        'blue': (0.1, 0.4, 1.0),
        'yellow': (1.0, 1.0, 0.1),
        'cyan': (0.1, 1.0, 1.0),
        'magenta': (1.0, 0.1, 1.0),
        'white': (1.0, 1.0, 1.0),
        'orange': (1.0, 0.5, 0.1),
        'purple': (0.6, 0.3, 1.0),
    }
    if t in named:
        return named[t]
    if t.startswith('#') and len(t) == 7:
        try:
            r = int(t[1:3], 16) / 255.0
            g = int(t[3:5], 16) / 255.0
            b = int(t[5:7], 16) / 255.0
            return (r, g, b)
        except ValueError:
            pass
    return default


class EnvSimNode(Node):
    """Simple differential-drive simulator publishing odom and TF."""

    def __init__(self) -> None:
        super().__init__('env_sim')
        self._declare_parameters()
        self._load_runtime_config()
        self._build_ir_model()
        self._reset_runtime_state()
        self._create_interfaces()
        self._log_startup()

    def _declare_parameters(self) -> None:
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('update_hz', 50.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('marker_topic', '/env_sim/markers')
        self.declare_parameter('charge_topic', '/dock/charge_connected')
        self.declare_parameter('station_visualize', True)
        self.declare_parameter('station_marker_ns', 'charging_station')
        self.declare_parameter('station_x', 0.8)
        self.declare_parameter('station_y', 0.0)
        self.declare_parameter('station_yaw', 3.1415926)
        self.declare_parameter('charge_success_dist_m', 0.038)
        self.declare_parameter('charge_success_yaw_tol_deg', 8.0)
        self.declare_parameter('charge_hold_s', 0.20)
        self.declare_parameter('dock_frame', 'dock')
        self.declare_parameter('predock_pose.enable', True)
        self.declare_parameter('predock_pose.topic', '/predock_pose')
        self.declare_parameter('predock_pose.distance_m', 1.0)
        self.declare_parameter('predock_pose.sample_std_m', 0.3)
        self.declare_parameter('predock_pose.sample_std_x_m', -1.0)
        self.declare_parameter('predock_pose.sample_std_y_m', 0.3)
        self.declare_parameter('predock_pose.angle_min_deg', -30.0)
        self.declare_parameter('predock_pose.angle_max_deg', 30.0)
        self.declare_parameter('predock_pose.publish_hz', 2.0)
        self.declare_parameter('fov_length', 1.2)
        self.declare_parameter('robot_length_m', 0.40)
        self.declare_parameter('robot_width_m', 0.35)
        self.declare_parameter('ir_spacing_m', 0.0109)
        self.declare_parameter('ir_fov_deg', 15.0)
        self.declare_parameter('ir_mount_x_m', -1.0)
        self.declare_parameter('ir_mount_center_y_m', 0.0)
        self.declare_parameter('ir_mount_yaw_deg', 0.0)
        self.declare_parameter('ir_sensors.receivers.left.x_offset', 0.20)
        self.declare_parameter('ir_sensors.update_hz', 50.0)
        self.declare_parameter('ir_sensors.topic', '/ir/dock')
        self.declare_parameter('ir_sensors.swap_channels', True)
        self.declare_parameter('ir_sensors.receivers.left.y_offset', 0.00545)
        self.declare_parameter('ir_sensors.receivers.left.fov', 15.0)
        self.declare_parameter('ir_sensors.receivers.left.yaw', 0.0)
        self.declare_parameter('ir_sensors.receivers.left.visible', True)
        self.declare_parameter('ir_sensors.receivers.left.mode', 'normal')
        self.declare_parameter('ir_sensors.receivers.left.color', 'cyan')
        self.declare_parameter('ir_sensors.receivers.left.max_length_m', 1.5)
        self.declare_parameter('ir_sensors.receivers.left.alpha', 1.0)
        self.declare_parameter('ir_sensors.receivers.left.fill_alpha', 0.20)
        self.declare_parameter('ir_sensors.receivers.right.x_offset', 0.20)
        self.declare_parameter('ir_sensors.receivers.right.y_offset', -0.00545)
        self.declare_parameter('ir_sensors.receivers.right.fov', 15.0)
        self.declare_parameter('ir_sensors.receivers.right.yaw', 0.0)
        self.declare_parameter('ir_sensors.receivers.right.visible', True)
        self.declare_parameter('ir_sensors.receivers.right.mode', 'normal')
        self.declare_parameter('ir_sensors.receivers.right.color', 'cyan')
        self.declare_parameter('ir_sensors.receivers.right.max_length_m', 1.5)
        self.declare_parameter('ir_sensors.receivers.right.alpha', 1.0)
        self.declare_parameter('ir_sensors.receivers.right.fill_alpha', 0.20)
        self.declare_parameter('ir_sensors.publish', True)
        self.declare_parameter('ir_sensors.use_dock_emitters', True)
        self.declare_parameter('ir_sensors.dock_frame', '')
        self.declare_parameter('ir_sensors.emitters.left.x', 0.0)
        self.declare_parameter('ir_sensors.emitters.left.y', 0.0555)
        self.declare_parameter('ir_sensors.emitters.left.yaw', 10.0)
        self.declare_parameter('ir_sensors.emitters.left.fov', 20.0)
        self.declare_parameter('ir_sensors.emitters.left.visible', True)
        self.declare_parameter('ir_sensors.emitters.left.mode', 'normal')
        self.declare_parameter('ir_sensors.emitters.left.color', 'magenta')
        self.declare_parameter('ir_sensors.emitters.left.alpha', 0.3)
        self.declare_parameter('ir_sensors.emitters.left.max_length_m', 1.5)
        self.declare_parameter('ir_sensors.emitters.left.trap_length_m', 0.03)
        self.declare_parameter('ir_sensors.emitters.left.trap_width_m', 0.002)
        self.declare_parameter('ir_sensors.emitters.far_left.x', 0.0)
        self.declare_parameter('ir_sensors.emitters.far_left.y', 0.01425)
        self.declare_parameter('ir_sensors.emitters.far_left.yaw', 55.0)
        self.declare_parameter('ir_sensors.emitters.far_left.fov', 20.0)
        self.declare_parameter('ir_sensors.emitters.far_left.visible', True)
        self.declare_parameter('ir_sensors.emitters.far_left.mode', 'normal')
        self.declare_parameter('ir_sensors.emitters.far_left.color', 'red')
        self.declare_parameter('ir_sensors.emitters.far_left.alpha', 0.3)
        self.declare_parameter('ir_sensors.emitters.far_left.max_length_m', 1.5)
        self.declare_parameter('ir_sensors.emitters.far_left.trap_length_m', 0.03)
        self.declare_parameter('ir_sensors.emitters.far_left.trap_width_m', 0.002)
        self.declare_parameter('ir_sensors.emitters.center.x', 0.0)
        self.declare_parameter('ir_sensors.emitters.center.y', 0.0)
        self.declare_parameter('ir_sensors.emitters.center.yaw', 0.0)
        self.declare_parameter('ir_sensors.emitters.center.fov', 7.34)
        self.declare_parameter('ir_sensors.emitters.center.visible', True)
        self.declare_parameter('ir_sensors.emitters.center.mode', 'trapezoid')
        self.declare_parameter('ir_sensors.emitters.center.color', 'green')
        self.declare_parameter('ir_sensors.emitters.center.alpha', 0.3)
        self.declare_parameter('ir_sensors.emitters.center.max_length_m', 1.5)
        self.declare_parameter('ir_sensors.emitters.center.trap_length_m', 0.03)
        self.declare_parameter('ir_sensors.emitters.center.trap_width_m', 0.002)
        self.declare_parameter('ir_sensors.emitters.right.x', 0.0)
        self.declare_parameter('ir_sensors.emitters.right.y', -0.0555)
        self.declare_parameter('ir_sensors.emitters.right.yaw', -10.0)
        self.declare_parameter('ir_sensors.emitters.right.fov', 20.0)
        self.declare_parameter('ir_sensors.emitters.right.visible', True)
        self.declare_parameter('ir_sensors.emitters.right.mode', 'normal')
        self.declare_parameter('ir_sensors.emitters.right.color', 'magenta')
        self.declare_parameter('ir_sensors.emitters.right.alpha', 0.3)
        self.declare_parameter('ir_sensors.emitters.right.max_length_m', 1.5)
        self.declare_parameter('ir_sensors.emitters.right.trap_length_m', 0.03)
        self.declare_parameter('ir_sensors.emitters.right.trap_width_m', 0.002)
        self.declare_parameter('ir_sensors.emitters.far_right.x', 0.0)
        self.declare_parameter('ir_sensors.emitters.far_right.y', -0.01425)
        self.declare_parameter('ir_sensors.emitters.far_right.yaw', -55.0)
        self.declare_parameter('ir_sensors.emitters.far_right.fov', 20.0)
        self.declare_parameter('ir_sensors.emitters.far_right.visible', True)
        self.declare_parameter('ir_sensors.emitters.far_right.mode', 'normal')
        self.declare_parameter('ir_sensors.emitters.far_right.color', 'red')
        self.declare_parameter('ir_sensors.emitters.far_right.alpha', 0.3)
        self.declare_parameter('ir_sensors.emitters.far_right.max_length_m', 1.5)
        self.declare_parameter('ir_sensors.emitters.far_right.trap_length_m', 0.03)
        self.declare_parameter('ir_sensors.emitters.far_right.trap_width_m', 0.002)
        self.declare_parameter('bit_left', 0x01)
        self.declare_parameter('bit_center', 0x04)
        self.declare_parameter('bit_right', 0x08)
        # Align with docking strategy: far_left/left/center/right/far_right.
        self.declare_parameter('bit_l1', 0x01)  # far_left
        self.declare_parameter('bit_l2', 0x02)  # left
        self.declare_parameter('bit_c', 0x04)   # center
        self.declare_parameter('bit_r1', 0x08)  # right
        self.declare_parameter('bit_r2', 0x10)  # far_right
        self.declare_parameter('x0', 0.0)
        self.declare_parameter('y0', 0.0)
        self.declare_parameter('yaw0', 0.0)

    def _load_runtime_config(self) -> None:
        self._cmd_topic = str(self.get_parameter('cmd_vel_topic').value)
        self._initialpose_topic = str(self.get_parameter('initialpose_topic').value)
        self._odom_topic = str(self.get_parameter('odom_topic').value)
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._publish_tf = bool(self.get_parameter('publish_tf').value)
        self._marker_topic = str(self.get_parameter('marker_topic').value)
        self._charge_topic = str(self.get_parameter('charge_topic').value)
        self._station_visualize = bool(self.get_parameter('station_visualize').value)
        self._station_ns = str(self.get_parameter('station_marker_ns').value)
        self._station_x = float(self.get_parameter('station_x').value)
        self._station_y = float(self.get_parameter('station_y').value)
        self._station_yaw = float(self.get_parameter('station_yaw').value)
        self._charge_success_dist_m = max(
            0.0, float(self.get_parameter('charge_success_dist_m').value)
        )
        self._charge_success_yaw_tol_rad = math.radians(
            max(0.0, float(self.get_parameter('charge_success_yaw_tol_deg').value))
        )
        self._charge_hold_s = max(0.0, float(self.get_parameter('charge_hold_s').value))
        self._charge_connected = False
        self._charge_contact_elapsed_s = 0.0
        self._predock_enable = bool(self.get_parameter('predock_pose.enable').value)
        self._predock_topic = str(self.get_parameter('predock_pose.topic').value)
        self._predock_distance_m = max(0.0, float(self.get_parameter('predock_pose.distance_m').value))
        self._predock_std_m = max(0.0, float(self.get_parameter('predock_pose.sample_std_m').value))
        self._predock_std_x_m = float(self.get_parameter('predock_pose.sample_std_x_m').value)
        if self._predock_std_x_m < 0.0:
            self._predock_std_x_m = self._predock_std_m
        self._predock_std_y_m = max(
            0.0, float(self.get_parameter('predock_pose.sample_std_y_m').value)
        )
        a_min = float(self.get_parameter('predock_pose.angle_min_deg').value)
        a_max = float(self.get_parameter('predock_pose.angle_max_deg').value)
        self._predock_angle_min_deg = min(a_min, a_max)
        self._predock_angle_max_deg = max(a_min, a_max)
        self._predock_hz = max(0.1, float(self.get_parameter('predock_pose.publish_hz').value))
        self._dock_frame = str(self.get_parameter('dock_frame').value)
        self._ir_topic = str(self.get_parameter('ir_sensors.topic').value)
        self._ir_publish = bool(self.get_parameter('ir_sensors.publish').value)
        self._ir_swap_channels = bool(self.get_parameter('ir_sensors.swap_channels').value)
        self._use_dock_emitters = bool(self.get_parameter('ir_sensors.use_dock_emitters').value)
        if str(self.get_parameter('ir_sensors.dock_frame').value).strip():
            self._dock_frame = str(self.get_parameter('ir_sensors.dock_frame').value).strip()
        self._fov_length = max(0.1, float(self.get_parameter('fov_length').value))
        robot_length = max(0.05, float(self.get_parameter('robot_length_m').value))
        robot_width = max(0.05, float(self.get_parameter('robot_width_m').value))
        ir_spacing = max(1e-4, float(self.get_parameter('ir_spacing_m').value))
        ir_fov_deg = max(0.1, float(self.get_parameter('ir_fov_deg').value))
        ir_mount_x = float(self.get_parameter('ir_mount_x_m').value)
        if ir_mount_x < 0.0:
            ir_mount_x = 0.5 * robot_length
        ir_mount_center_y = float(self.get_parameter('ir_mount_center_y_m').value)
        ir_mount_yaw_deg = float(self.get_parameter('ir_mount_yaw_deg').value)
        ir_left_x_cfg = float(self.get_parameter('ir_sensors.receivers.left.x_offset').value)
        ir_left_y_cfg = float(self.get_parameter('ir_sensors.receivers.left.y_offset').value)
        ir_left_fov_cfg = max(0.1, float(self.get_parameter('ir_sensors.receivers.left.fov').value))
        ir_left_yaw_cfg = float(self.get_parameter('ir_sensors.receivers.left.yaw').value)
        ir_left_mode = str(self.get_parameter('ir_sensors.receivers.left.mode').value)
        ir_left_color = str(self.get_parameter('ir_sensors.receivers.left.color').value)
        ir_left_range = max(
            0.05, float(self.get_parameter('ir_sensors.receivers.left.max_length_m').value)
        )
        ir_left_alpha = min(
            1.0, max(0.0, float(self.get_parameter('ir_sensors.receivers.left.alpha').value))
        )
        ir_left_fill_alpha = min(
            1.0, max(0.0, float(self.get_parameter('ir_sensors.receivers.left.fill_alpha').value))
        )
        self._vis_left = bool(self.get_parameter('ir_sensors.receivers.left.visible').value)
        ir_right_x_cfg = float(self.get_parameter('ir_sensors.receivers.right.x_offset').value)
        ir_right_y_cfg = float(self.get_parameter('ir_sensors.receivers.right.y_offset').value)
        ir_right_fov_cfg = max(0.1, float(self.get_parameter('ir_sensors.receivers.right.fov').value))
        ir_right_yaw_cfg = float(self.get_parameter('ir_sensors.receivers.right.yaw').value)
        ir_right_mode = str(self.get_parameter('ir_sensors.receivers.right.mode').value)
        ir_right_color = str(self.get_parameter('ir_sensors.receivers.right.color').value)
        ir_right_range = max(
            0.05, float(self.get_parameter('ir_sensors.receivers.right.max_length_m').value)
        )
        ir_right_alpha = min(
            1.0, max(0.0, float(self.get_parameter('ir_sensors.receivers.right.alpha').value))
        )
        ir_right_fill_alpha = min(
            1.0, max(0.0, float(self.get_parameter('ir_sensors.receivers.right.fill_alpha').value))
        )
        self._vis_right = bool(self.get_parameter('ir_sensors.receivers.right.visible').value)
        l_rgb = _parse_color(ir_left_color, default=(0.2, 0.6, 1.0))
        r_rgb = _parse_color(ir_right_color, default=(1.0, 0.4, 0.2))
        bit_left = int(self.get_parameter('bit_left').value) & 0xFF
        bit_center = int(self.get_parameter('bit_center').value) & 0xFF
        bit_right = int(self.get_parameter('bit_right').value) & 0xFF
        self._bit_left = bit_left
        self._bit_center = bit_center
        self._bit_right = bit_right
        self._bit_l1 = int(self.get_parameter('bit_l1').value) & 0xFF
        self._bit_l2 = int(self.get_parameter('bit_l2').value) & 0xFF
        self._bit_c = int(self.get_parameter('bit_c').value) & 0xFF
        self._bit_r1 = int(self.get_parameter('bit_r1').value) & 0xFF
        self._bit_r2 = int(self.get_parameter('bit_r2').value) & 0xFF
        # Physical emitter order from left to right → strategy bit map.
        self._name_to_token = {
            'far_left': self._bit_l1,   # 0x01
            'left': self._bit_l2,       # 0x02
            'center': self._bit_c,      # 0x04
            'right': self._bit_r1,      # 0x08
            'far_right': self._bit_r2,  # 0x10
        }
        self._emitters = self._load_emitters()

        sim_update_hz = float(self.get_parameter('update_hz').value)
        self._sim_hz = max(1.0, sim_update_hz)
        self._dt = 1.0 / self._sim_hz
        ir_update_hz = float(self.get_parameter('ir_sensors.update_hz').value)
        if ir_update_hz <= 0.0:
            ir_update_hz = self._sim_hz
        self._ir_hz = max(1.0, ir_update_hz)

    def _build_ir_model(self) -> None:
        l_rgb = _parse_color(
            str(self.get_parameter('ir_sensors.receivers.left.color').value),
            default=(0.2, 0.6, 1.0),
        )
        r_rgb = _parse_color(
            str(self.get_parameter('ir_sensors.receivers.right.color').value),
            default=(1.0, 0.4, 0.2),
        )
        robot_length = max(0.05, float(self.get_parameter('robot_length_m').value))
        robot_width = max(0.05, float(self.get_parameter('robot_width_m').value))
        ir_spacing = max(1e-4, float(self.get_parameter('ir_spacing_m').value))
        ir_fov_deg = max(0.1, float(self.get_parameter('ir_fov_deg').value))
        ir_mount_x = float(self.get_parameter('ir_mount_x_m').value)
        if ir_mount_x < 0.0:
            ir_mount_x = 0.5 * robot_length
        ir_mount_center_y = float(self.get_parameter('ir_mount_center_y_m').value)
        ir_mount_yaw_deg = float(self.get_parameter('ir_mount_yaw_deg').value)

        ir_left_x_cfg = float(self.get_parameter('ir_sensors.receivers.left.x_offset').value)
        ir_left_y_cfg = float(self.get_parameter('ir_sensors.receivers.left.y_offset').value)
        ir_left_fov_cfg = max(
            0.1, float(self.get_parameter('ir_sensors.receivers.left.fov').value)
        )
        ir_left_yaw_cfg = float(self.get_parameter('ir_sensors.receivers.left.yaw').value)
        ir_left_mode = str(self.get_parameter('ir_sensors.receivers.left.mode').value)
        ir_left_range = max(
            0.05,
            float(self.get_parameter('ir_sensors.receivers.left.max_length_m').value),
        )
        ir_left_alpha = min(
            1.0,
            max(0.0, float(self.get_parameter('ir_sensors.receivers.left.alpha').value)),
        )
        ir_left_fill_alpha = min(
            1.0,
            max(
                0.0,
                float(self.get_parameter('ir_sensors.receivers.left.fill_alpha').value),
            ),
        )
        self._vis_left = bool(self.get_parameter('ir_sensors.receivers.left.visible').value)

        ir_right_x_cfg = float(self.get_parameter('ir_sensors.receivers.right.x_offset').value)
        ir_right_y_cfg = float(self.get_parameter('ir_sensors.receivers.right.y_offset').value)
        ir_right_fov_cfg = max(
            0.1, float(self.get_parameter('ir_sensors.receivers.right.fov').value)
        )
        ir_right_yaw_cfg = float(self.get_parameter('ir_sensors.receivers.right.yaw').value)
        ir_right_mode = str(self.get_parameter('ir_sensors.receivers.right.mode').value)
        ir_right_range = max(
            0.05,
            float(self.get_parameter('ir_sensors.receivers.right.max_length_m').value),
        )
        ir_right_alpha = min(
            1.0,
            max(0.0, float(self.get_parameter('ir_sensors.receivers.right.alpha').value)),
        )
        ir_right_fill_alpha = min(
            1.0,
            max(
                0.0,
                float(self.get_parameter('ir_sensors.receivers.right.fill_alpha').value),
            ),
        )
        self._vis_right = bool(self.get_parameter('ir_sensors.receivers.right.visible').value)

        self._ir_model = IRSensorArray(
            robot_length_m=robot_length,
            robot_width_m=robot_width,
            spacing_m=ir_spacing,
            mount_x_m=ir_mount_x,
            mount_center_y_m=ir_mount_center_y,
            mount_yaw_deg=ir_mount_yaw_deg,
            fov_deg=ir_fov_deg,
            max_range_m=self._fov_length,
            bit_left=self._bit_left,
            bit_center=self._bit_center,
            bit_right=self._bit_right,
            use_explicit_pair_pose=True,
            left_x_m=ir_left_x_cfg,
            left_y_m=ir_left_y_cfg,
            left_yaw_deg=ir_left_yaw_cfg,
            left_fov_deg=ir_left_fov_cfg,
            left_mode=ir_left_mode,
            left_range_m=ir_left_range,
            right_x_m=ir_right_x_cfg,
            right_y_m=ir_right_y_cfg,
            right_yaw_deg=ir_right_yaw_cfg,
            right_fov_deg=ir_right_fov_cfg,
            right_mode=ir_right_mode,
            right_range_m=ir_right_range,
            left_sensor_color=(l_rgb[0], l_rgb[1], l_rgb[2], ir_left_alpha),
            right_sensor_color=(r_rgb[0], r_rgb[1], r_rgb[2], ir_right_alpha),
            fov_color=(
                0.5 * (l_rgb[0] + r_rgb[0]),
                0.5 * (l_rgb[1] + r_rgb[1]),
                0.5 * (l_rgb[2] + r_rgb[2]),
                max(ir_left_alpha, ir_right_alpha),
            ),
            fov_fill_color=(
                0.5 * (l_rgb[0] + r_rgb[0]),
                0.5 * (l_rgb[1] + r_rgb[1]),
                0.5 * (l_rgb[2] + r_rgb[2]),
                max(ir_left_fill_alpha, ir_right_fill_alpha),
            ),
            enable_fov_fill=True,
        )

    def _reset_runtime_state(self) -> None:
        self._pose = Pose2D(
            x=float(self.get_parameter('x0').value),
            y=float(self.get_parameter('y0').value),
            yaw=float(self.get_parameter('yaw0').value),
        )
        self._cmd_vx = 0.0
        self._cmd_wz = 0.0
        self._last_stamp = self.get_clock().now()

    def _create_interfaces(self) -> None:
        self.create_subscription(Twist, self._cmd_topic, self._cb_cmd_vel, 20)
        self.create_subscription(
            PoseWithCovarianceStamped,
            self._initialpose_topic,
            self._cb_initialpose,
            10,
        )
        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, 20)
        self._marker_pub = self.create_publisher(MarkerArray, self._marker_topic, 10)
        self._ir_pub = self.create_publisher(DockIR, self._ir_topic, 20)
        self._charge_pub = self.create_publisher(Bool, self._charge_topic, 10)
        self._predock_pub: Optional[rclpy.publisher.Publisher] = None
        self._predock_timer = None
        if self._predock_enable:
            self._predock_pub = self.create_publisher(PoseStamped, self._predock_topic, 10)
            self._predock_timer = self.create_timer(1.0 / self._predock_hz, self._publish_predock_pose)
        self._tf_broadcaster: Optional[TransformBroadcaster] = None
        if self._publish_tf:
            self._tf_broadcaster = TransformBroadcaster(self)
        self._timer = self.create_timer(self._dt, self._on_timer)
        self._ir_timer = self.create_timer(1.0 / self._ir_hz, self._on_ir_timer)

    def _log_startup(self) -> None:
        self.get_logger().info(
            'env_sim started: sub=%s pub=%s tf=%s sim_hz=%.1f ir_hz=%.1f'
            % (
                self._cmd_topic,
                self._odom_topic,
                str(self._publish_tf).lower(),
                float(self._sim_hz),
                float(self._ir_hz),
            )
        )

    def _stop_robot(self) -> None:
        self._cmd_vx = 0.0
        self._cmd_wz = 0.0

    def _cb_cmd_vel(self, msg: Twist) -> None:
        self._cmd_vx = float(msg.linear.x)
        self._cmd_wz = float(msg.angular.z)

    def _cb_initialpose(self, msg: PoseWithCovarianceStamped) -> None:
        """Sets robot pose from /initialpose in odom frame."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = wrap_pi(_quat_to_yaw(float(q.x), float(q.y), float(q.z), float(q.w)))
        self._pose = Pose2D(float(p.x), float(p.y), yaw)
        self._stop_robot()
        self._last_stamp = self.get_clock().now()

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_stamp).nanoseconds * 1e-9
        self._last_stamp = now
        if dt <= 0.0:
            dt = self._dt

        c = math.cos(self._pose.yaw)
        s = math.sin(self._pose.yaw)
        x = self._pose.x + self._cmd_vx * c * dt
        y = self._pose.y + self._cmd_vx * s * dt
        yaw = wrap_pi(self._pose.yaw + self._cmd_wz * dt)
        self._pose = Pose2D(x, y, yaw)

        self._publish_odom_and_tf(now)
        self._update_charge_contact(dt)
        self._publish_markers(now)

    def _on_ir_timer(self) -> None:
        now = self.get_clock().now()
        self._publish_ir(now)

    def _publish_odom_and_tf(self, stamp) -> None:
        qx, qy, qz, qw = _yaw_to_quat(self._pose.yaw)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._pose.x
        odom.pose.pose.position.y = self._pose.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self._cmd_vx
        odom.twist.twist.angular.z = self._cmd_wz
        self._odom_pub.publish(odom)

        if self._tf_broadcaster is None:
            return
        self._publish_dock_tf(odom.header.stamp)
        tfm = TransformStamped()
        tfm.header.stamp = odom.header.stamp
        tfm.header.frame_id = self._odom_frame
        tfm.child_frame_id = self._base_frame
        tfm.transform.translation.x = self._pose.x
        tfm.transform.translation.y = self._pose.y
        tfm.transform.translation.z = 0.0
        tfm.transform.rotation.x = qx
        tfm.transform.rotation.y = qy
        tfm.transform.rotation.z = qz
        tfm.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tfm)

    def _publish_dock_tf(self, stamp_msg) -> None:
        if self._tf_broadcaster is None:
            return
        qx, qy, qz, qw = _yaw_to_quat(self._station_yaw)
        tfm = TransformStamped()
        tfm.header.stamp = stamp_msg
        tfm.header.frame_id = self._odom_frame
        tfm.child_frame_id = self._dock_frame
        tfm.transform.translation.x = self._station_x
        tfm.transform.translation.y = self._station_y
        tfm.transform.translation.z = 0.0
        tfm.transform.rotation.x = qx
        tfm.transform.rotation.y = qy
        tfm.transform.rotation.z = qz
        tfm.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tfm)

    def _publish_predock_pose(self) -> None:
        if self._predock_pub is None:
            return
        pose = self.random_prdock_pose()
        qx, qy, qz, qw = _yaw_to_quat(pose.yaw)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._odom_frame
        msg.pose.position.x = pose.x
        msg.pose.position.y = pose.y
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self._predock_pub.publish(msg)

    def _publish_markers(self, stamp) -> None:
        mk: MarkerArray = self._ir_model.build_markers(
            self._pose,
            frame_id=self._odom_frame,
            stamp=stamp.to_msg(),
            ns='env_sim',
            visualize_left=self._vis_left,
            visualize_right=self._vis_right,
        )
        if self._station_visualize:
            mk.markers.extend(self._mk_station_markers(stamp))
        self._marker_pub.publish(mk)

    def _publish_ir(self, stamp) -> None:
        if not self._ir_publish:
            return
        left_xy, right_xy = self._ir_model.sensor_positions_world(self._pose)
        emitters_w = self._emitters_world()
        left_hits = self._hits_for_receiver(left_xy, self._ir_model.left_sensor.yaw, self._ir_model.left_sensor, emitters_w)
        right_hits = self._hits_for_receiver(
            right_xy, self._ir_model.right_sensor.yaw, self._ir_model.right_sensor, emitters_w
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

    def _dock_center_emitter_world(self) -> Tuple[float, float, float]:
        for emitter in self._emitters_world():
            if str(emitter.get('name', '')) == 'center':
                return (
                    float(emitter['wx']),
                    float(emitter['wy']),
                    float(emitter['wyaw']),
                )
        return (float(self._station_x), float(self._station_y), float(self._station_yaw))

    def _robot_front_ir_world(self) -> Tuple[float, float, float]:
        left_xy, right_xy = self._ir_model.sensor_positions_world(self._pose)
        center_x = 0.5 * (float(left_xy[0]) + float(right_xy[0]))
        center_y = 0.5 * (float(left_xy[1]) + float(right_xy[1]))
        heading = float(self._pose.yaw)
        return (center_x, center_y, heading)

    def _is_charge_contact(self) -> bool:
        ir_x, ir_y, ir_yaw = self._robot_front_ir_world()
        dock_ir_x, dock_ir_y, dock_ir_yaw = self._dock_center_emitter_world()
        dx = ir_x - dock_ir_x
        dy = ir_y - dock_ir_y
        dist = math.hypot(dx, dy)
        # Success requires the robot front IR and dock center emitter to face
        # each other, so the relative heading should be close to pi instead of 0.
        yaw_err = abs(abs(wrap_pi(ir_yaw - dock_ir_yaw)) - math.pi)
        return (
            dist <= self._charge_success_dist_m
            and yaw_err <= self._charge_success_yaw_tol_rad
        )

    def _publish_charge_connected(self) -> None:
        msg = Bool()
        msg.data = bool(self._charge_connected)
        self._charge_pub.publish(msg)

    def _update_charge_contact(self, dt: float) -> None:
        if self._is_charge_contact():
            self._charge_contact_elapsed_s += max(0.0, float(dt))
        else:
            self._charge_contact_elapsed_s = 0.0

        connected = self._charge_contact_elapsed_s >= self._charge_hold_s
        if connected != self._charge_connected:
            self._charge_connected = connected
            self.get_logger().info(
                'charge_connected=%s' % str(self._charge_connected).lower()
            )
        self._publish_charge_connected()

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
                    'color': str(self.get_parameter(f'{base}.color').value),
                    'alpha': min(1.0, max(0.0, float(self.get_parameter(f'{base}.alpha').value))),
                    'max_length_m': max(0.05, float(self.get_parameter(f'{base}.max_length_m').value)),
                    'trap_length_m': max(0.0, float(self.get_parameter(f'{base}.trap_length_m').value)),
                    'trap_width_m': max(0.0, float(self.get_parameter(f'{base}.trap_width_m').value)),
                }
            )
        return out

    def _emitters_world(self) -> List[Dict[str, float]]:
        if not self._use_dock_emitters:
            return []
        tx = self._station_x
        ty = self._station_y
        dock_yaw = self._station_yaw
        c = math.cos(dock_yaw)
        s = math.sin(dock_yaw)
        out: List[Dict[str, float]] = []
        for e in self._emitters:
            if not bool(e['visible']):
                continue
            ex = tx + c * e['x'] - s * e['y']
            ey = ty + s * e['x'] + c * e['y']
            out.append(
                {
                    **e,
                    'wx': ex,
                    'wy': ey,
                    'wyaw': dock_yaw + e['yaw'],
                }
            )
        return out

    @staticmethod
    def _point_in_emitter_fov(rx: float, ry: float, e: Dict[str, float]) -> bool:
        dx = rx - float(e['wx'])
        dy = ry - float(e['wy'])
        dist = math.hypot(dx, dy)
        if dist > float(e['max_length_m']):
            return False
        heading = float(e['wyaw'])
        mode = str(e['mode'])
        half = math.radians(0.5 * float(e['fov']))
        if mode in ('trapezoid', 'trap'):
            ux = math.cos(heading)
            uy = math.sin(heading)
            vx = -math.sin(heading)
            vy = math.cos(heading)
            t = dx * ux + dy * uy
            n = dx * vx + dy * vy
            if t < 0.0 or t > float(e['max_length_m']):
                return False
            near = min(float(e['trap_length_m']), float(e['max_length_m']))
            near_w = 0.5 * float(e['trap_width_m'])
            if t <= near:
                return abs(n) <= near_w
            extra = t - near
            return abs(n) <= (near_w + math.tan(half) * extra)
        ang = math.atan2(dy, dx)
        err = abs(wrap_pi(ang - heading))
        return err <= half

    @staticmethod
    def _point_in_receiver_fov(
        rx: float,
        ry: float,
        r_heading: float,
        receiver,
        tx: float,
        ty: float,
    ) -> Tuple[bool, float]:
        """Checks whether target point is inside one receiver effective FOV."""
        dx = tx - rx
        dy = ty - ry
        dist = math.hypot(dx, dy)
        if dist > float(receiver.max_range_m):
            return (False, dist)
        mode = str(getattr(receiver, 'mode', 'normal')).strip().lower()
        half = math.radians(0.5 * float(receiver.fov_deg))
        if mode in ('trapezoid', 'trap'):
            ux = math.cos(r_heading)
            uy = math.sin(r_heading)
            vx = -math.sin(r_heading)
            vy = math.cos(r_heading)
            t = dx * ux + dy * uy
            n = dx * vx + dy * vy
            if t < 0.0 or t > float(receiver.max_range_m):
                return (False, dist)
            near = 0.05 * float(receiver.max_range_m)
            near_w = 0.5 * near
            if t <= near:
                return (abs(n) <= near_w, dist)
            return (abs(n) <= (near_w + math.tan(half) * (t - near)), dist)
        ang = math.atan2(dy, dx)
        return (abs(wrap_pi(ang - r_heading)) <= half, dist)

    def _hits_for_receiver(
        self,
        rxy: Tuple[float, float],
        r_yaw_rel: float,
        receiver,
        emitters_w: List[Dict[str, float]],
    ) -> List[Dict[str, float]]:
        hits: List[Dict[str, float]] = []
        rx, ry = rxy
        r_heading = self._pose.yaw + float(r_yaw_rel)
        for e in emitters_w:
            in_recv, dist = self._point_in_receiver_fov(
                rx, ry, r_heading, receiver, float(e['wx']), float(e['wy'])
            )
            if not in_recv:
                continue
            if not self._point_in_emitter_fov(rx, ry, e):
                continue
            hit = dict(e)
            hit['dist'] = dist
            hits.append(hit)
        return hits

    def _token_from_hits(self, hits: List[Dict[str, float]]) -> int:
        """OR all hit emitter bits (5 beams can combine on one receiver)."""
        if not hits:
            return 0
        token = 0
        for h in hits:
            name = str(h.get('name', ''))
            if name in self._name_to_token:
                token |= int(self._name_to_token[name]) & 0xFF
        if token != 0:
            return int(token) & 0xFF
        # Fallback if hits lack named emitters.
        if any(str(h.get('name', '')) == 'center' for h in hits):
            return int(self._bit_center) & 0xFF
        score = sum(float(h.get('y', 0.0)) for h in hits)
        return int(self._bit_left if score >= 0.0 else self._bit_right) & 0xFF

    def _mk_station_markers(self, stamp) -> List[Marker]:
        out: List[Marker] = []
        idx = 100
        emitters_w = self._emitters_world()
        for e in emitters_w:
            sphere = Marker()
            sphere.header.stamp = stamp.to_msg()
            sphere.header.frame_id = self._odom_frame
            sphere.ns = self._station_ns
            sphere.id = idx
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.orientation.w = 1.0
            sphere.pose.position.x = float(e['wx'])
            sphere.pose.position.y = float(e['wy'])
            sphere.pose.position.z = 0.06
            sphere.scale.x = 0.003
            sphere.scale.y = 0.003
            sphere.scale.z = 0.003
            cr, cg, cb = _parse_color(str(e['color']), default=(1.0, 0.1, 0.1))
            sphere.color.a = float(e['alpha'])
            sphere.color.r = cr
            sphere.color.g = cg
            sphere.color.b = cb
            out.append(sphere)
            idx += 1

            fov = Marker()
            fov.header.stamp = stamp.to_msg()
            fov.header.frame_id = self._odom_frame
            fov.ns = self._station_ns
            fov.id = idx
            fov.type = Marker.LINE_LIST
            fov.action = Marker.ADD
            fov.pose.orientation.w = 1.0
            fov.scale.x = 0.01
            fov.color.a = float(e['alpha'])
            fov.color.r = cr
            fov.color.g = cg
            fov.color.b = cb
            half = math.radians(0.5 * float(e['fov']))
            fov_len = max(0.05, float(e['max_length_m']))
            ex = float(e['wx'])
            ey = float(e['wy'])
            eyaw = float(e['wyaw'])
            mode = str(e['mode']).strip().lower()
            if mode in ('trapezoid', 'trap'):
                near = min(max(0.0, float(e['trap_length_m'])), fov_len)
                far = fov_len
                near_w = 0.5 * float(e['trap_width_m'])
                far_w = near_w + math.tan(half) * max(0.0, far - near)
                ux = math.cos(eyaw)
                uy = math.sin(eyaw)
                vx = -math.sin(eyaw)
                vy = math.cos(eyaw)
                p_nl = Point(x=ex + ux * near - vx * near_w, y=ey + uy * near - vy * near_w, z=0.05)
                p_nr = Point(x=ex + ux * near + vx * near_w, y=ey + uy * near + vy * near_w, z=0.05)
                p_fl = Point(x=ex + ux * far - vx * far_w, y=ey + uy * far - vy * far_w, z=0.05)
                p_fr = Point(x=ex + ux * far + vx * far_w, y=ey + uy * far + vy * far_w, z=0.05)
                fov.points.extend([p_nl, p_nr, p_nl, p_fl, p_nr, p_fr, p_fl, p_fr])
            else:
                for sign in (-1.0, 1.0):
                    ang = eyaw + sign * half
                    fov.points.append(Point(x=ex, y=ey, z=0.05))
                    fov.points.append(
                        Point(
                            x=ex + fov_len * math.cos(ang),
                            y=ey + fov_len * math.sin(ang),
                            z=0.05,
                        )
                    )
            out.append(fov)
            idx += 1
        return out


    def random_prdock_pose(self) -> Pose2D:
        """Samples predock pose around point 1.0m in front of dock."""
        # Dock front-center point in odom.
        cx = self._station_x + self._predock_distance_m * math.cos(self._station_yaw)
        cy = self._station_y + self._predock_distance_m * math.sin(self._station_yaw)
        # Add independent local-frame x/y perturbation (y now has explicit variance).
        dx_local = random.gauss(0.0, self._predock_std_x_m)
        dy_local = random.gauss(0.0, self._predock_std_y_m)
        c = math.cos(self._station_yaw)
        s = math.sin(self._station_yaw)
        x = cx + c * dx_local - s * dy_local
        y = cy + s * dx_local + c * dy_local
        ang = math.radians(random.uniform(self._predock_angle_min_deg, self._predock_angle_max_deg))
        # Predock heading points roughly to dock center, with same angular spread.
        yaw = wrap_pi(self._station_yaw + math.pi + ang)
        return Pose2D(x=x, y=y, yaw=yaw)


def main(args: Optional[list[str]] = None) -> None:
    """Runs env simulator node."""
    rclpy.init(args=args)
    node = EnvSimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
