"""Visualization and monitoring publishers for docking."""

from __future__ import annotations

import math
from typing import Callable, Optional

from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from autocharge.common.geometry import yaw_to_quat
from autocharge.docking.type import DockState, MissionPhase, PredockGoal


class DockVisualization:
    """Publishes poses, state strings, markers, and test metrics."""

    def __init__(self, node: Node) -> None:
        self.node = node
        self._stats_pub = None
        self._state_pub = None
        self._dock_est_pub = None
        self._predock_pose_pub = None
        self._marker_pub = None
        self._success_metrics_published = False
        self._abort_metrics_published = False

    def setup(self) -> None:
        node = self.node
        self._stats_pub = node.create_publisher(
            String, str(node.get_parameter('stats_topic').value), 10
        )
        self._state_pub = node.create_publisher(
            String, str(node.get_parameter('state_topic').value), 10
        )
        self._dock_est_pub = node.create_publisher(
            PoseStamped, '/dock/estimated_pose', 10
        )
        self._marker_pub = node.create_publisher(MarkerArray, '/dock/markers', 10)
        if bool(node.get_parameter('predock.publish_pose').value):
            topic = str(node.get_parameter('predock.pose_topic').value)
            self._predock_pose_pub = node.create_publisher(PoseStamped, topic, 10)

    def reset_metrics_latch(self) -> None:
        self._success_metrics_published = False
        self._abort_metrics_published = False

    def publish_predock_pose(self, goal: Optional[PredockGoal]) -> None:
        if self._predock_pose_pub is None or goal is None:
            return
        msg = goal.to_msg(
            frame_id='odom',
            stamp=self.node.get_clock().now().to_msg(),
        )
        self._predock_pose_pub.publish(msg)

    def publish_estimated_dock_pose(self, dock_x: float, dock_y: float, dock_yaw: float) -> None:
        if self._dock_est_pub is None:
            return
        qx, qy, qz, qw = yaw_to_quat(dock_yaw)
        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = float(dock_x)
        msg.pose.position.y = float(dock_y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self._dock_est_pub.publish(msg)

    def publish_dock_state(self, phase: MissionPhase, fsm_state: DockState) -> None:
        if self._state_pub is None:
            return
        msg = String()
        if phase == MissionPhase.PREDOCK:
            msg.data = 'PREDOCK'
        elif fsm_state == DockState.SUCCESS:
            msg.data = 'SUCCESS'
        elif fsm_state == DockState.ABORT:
            msg.data = 'FAIL'
        else:
            msg.data = fsm_state.name
        self._state_pub.publish(msg)

    def publish_dock_markers(
        self,
        *,
        fsm_state: DockState,
        dock_x: float,
        dock_y: float,
        dock_yaw: float,
    ) -> None:
        """Publish dock centerline (and dock origin) to /dock/markers."""
        del fsm_state
        if self._marker_pub is None:
            return
        stamp = self.node.get_clock().now().to_msg()
        arr = MarkerArray()
        ns = 'dock_guidance'
        _MAX_MARKER_ID = 8

        def _hdr(m: Marker, mid: int) -> Marker:
            m.header.stamp = stamp
            m.header.frame_id = 'odom'
            m.ns = ns
            m.id = mid
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.lifetime.sec = 0
            m.lifetime.nanosec = 200_000_000
            return m

        def _color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
            c = ColorRGBA()
            c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
            return c

        def _pt(x: float, y: float, z: float = 0.02) -> Point:
            p = Point()
            p.x, p.y, p.z = float(x), float(y), float(z)
            return p

        c_d = math.cos(float(dock_yaw))
        s_d = math.sin(float(dock_yaw))
        axis_len = 1.2
        cl0 = (float(dock_x), float(dock_y))
        cl1 = (
            float(dock_x) + c_d * axis_len,
            float(dock_y) + s_d * axis_len,
        )
        line = _hdr(Marker(), 0)
        line.type = Marker.LINE_STRIP
        line.scale.x = 0.012
        line.color = _color(0.2, 0.9, 0.35, 0.85)
        line.points = [_pt(*cl0), _pt(*cl1)]
        arr.markers.append(line)

        dock_dot = _hdr(Marker(), 1)
        dock_dot.type = Marker.SPHERE
        dock_dot.pose.position.x = float(dock_x)
        dock_dot.pose.position.y = float(dock_y)
        dock_dot.pose.position.z = 0.04
        dock_dot.scale.x = dock_dot.scale.y = dock_dot.scale.z = 0.04
        dock_dot.color = _color(0.2, 1.0, 0.4, 0.95)
        arr.markers.append(dock_dot)

        for mid in range(2, _MAX_MARKER_ID + 1):
            d = _hdr(Marker(), mid)
            d.action = Marker.DELETE
            arr.markers.append(d)
        self._marker_pub.publish(arr)

    def publish_test_metrics(
        self,
        *,
        enabled: bool,
        fsm_state: DockState,
        dist_to_dock: Optional[float],
        lateral_y_err: float,
        yaw_err_rad: float,
        ir_left: int,
        ir_right: int,
    ) -> None:
        if not enabled or self._stats_pub is None:
            return
        if fsm_state not in (DockState.SUCCESS, DockState.ABORT):
            return
        if fsm_state == DockState.SUCCESS:
            if self._success_metrics_published:
                return
            self._success_metrics_published = True
        else:
            if self._abort_metrics_published:
                return
            self._abort_metrics_published = True
        dist_front = -1.0 if dist_to_dock is None else float(dist_to_dock)
        yaw_err_abs_rad = abs(float(yaw_err_rad))
        yaw_err_abs_deg = math.degrees(yaw_err_abs_rad)
        msg = String()
        msg.data = (
            'lateral_front_y_to_dock_center_y_m=%.4f,'
            'front_contact_to_dock_center_m=%.4f,'
            'yaw_err_to_dock_rad=%.6f,'
            'yaw_abs_err_to_dock_rad=%.6f,'
            'yaw_abs_err_to_dock_deg=%.3f,'
            'state=%s,ir=(0x%02x,0x%02x)'
            % (
                float(lateral_y_err),
                float(dist_front),
                float(yaw_err_rad),
                float(yaw_err_abs_rad),
                float(yaw_err_abs_deg),
                fsm_state.name,
                int(ir_left),
                int(ir_right),
            )
        )
        self._stats_pub.publish(msg)
        self.node.get_logger().debug(
            'terminal metrics published: state=%s lat_err=%.4f front_err=%.4f '
            'yaw_abs_deg=%.3f ir=(0x%02x,0x%02x)'
            % (
                fsm_state.name,
                float(lateral_y_err),
                float(dist_front),
                float(yaw_err_abs_deg),
                int(ir_left),
                int(ir_right),
            )
        )

    def log_control(
        self,
        *,
        enabled: bool,
        fsm_state_name: str,
        vx_out: float,
        wz_out: float,
        ir_l: int,
        ir_r: int,
        ir_left_raw: int,
        ir_right_raw: int,
        dist_to_dock: Optional[float],
        format_token: Callable[[int], str],
    ) -> None:
        if not enabled:
            return
        dist_s = 'N/A' if dist_to_dock is None else f'{float(dist_to_dock):.4f}'
        self.node.get_logger().info(
            'state=%s cmd(vx=%.3f,wz=%.3f) ir=(%s,%s) raw=(%s,%s) dist=%s'
            % (
                fsm_state_name,
                vx_out,
                wz_out,
                format_token(int(ir_l)),
                format_token(int(ir_r)),
                format_token(int(ir_left_raw)),
                format_token(int(ir_right_raw)),
                dist_s,
            ),
            throttle_duration_sec=0.25,
        )
