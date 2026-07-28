#!/usr/bin/env python3
# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Full autocharge demo: predock nav -> dock -> leave to predock -> nav origin."""

from __future__ import annotations

import math
from enum import Enum, auto

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from jdbot_interfaces.action import AutoCharge
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from autonomy_autocharge.geometry_utils import (
    make_pose_stamped,
    quaternion_to_yaw,
    resolve_predock_pose,
)


class DemoPhase(Enum):
    WAIT = auto()
    NAV_PREDOCK = auto()
    DOCK = auto()
    CHARGE_HOLD = auto()
    NAV_LEAVE_PREDOCK = auto()
    NAV_ORIGIN = auto()
    DONE = auto()
    FAILED = auto()


class AutochargeDemoNode(Node):
    """Orchestrate predock nav, dock, leave predock, and return to map origin."""

    def __init__(self) -> None:
        super().__init__('autocharge_demo_node')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('navigate_to_pose_action', '/navigate_to_pose')
        self.declare_parameter('dock_to_charger_action', '/dock_to_charger')
        self.declare_parameter('predock_pose_topic', '/predock_pose')
        self.declare_parameter('charger_x', 2.0)
        self.declare_parameter('charger_y', 0.0)
        self.declare_parameter('charger_yaw', 0.0)
        self.declare_parameter('use_explicit_predock', True)
        self.declare_parameter('predock_x', 1.4)
        self.declare_parameter('predock_y', -0.08)
        self.declare_parameter('predock_yaw', 0.0)
        self.declare_parameter('predock_distance_m', 0.8)
        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', 0.0)
        self.declare_parameter('origin_yaw', 0.0)
        self.declare_parameter('dock_timeout_s', 180.0)
        self.declare_parameter('nav_timeout_s', 120.0)
        self.declare_parameter('post_dock_hold_s', 3.0)
        self.declare_parameter('run_return_mission', True)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('start_delay_s', 20.0)
        self.declare_parameter('server_wait_timeout_s', 120.0)
        self.declare_parameter('nav_retry_on_cancel', True)

        self._map_frame = str(self.get_parameter('map_frame').value)
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._phase = DemoPhase.WAIT
        self._nav_goal_handle = None
        self._dock_goal_handle = None
        self._nav_retry_count = 0
        self._nav_watchdog = None
        self._start_timer = None
        self._hold_timer = None
        self._nav_label = ''

        self._cb_group = ReentrantCallbackGroup()
        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter('navigate_to_pose_action').value),
            callback_group=self._cb_group,
        )
        self._dock_client = ActionClient(
            self,
            AutoCharge,
            str(self.get_parameter('dock_to_charger_action').value),
            callback_group=self._cb_group,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        predock_topic = str(self.get_parameter('predock_pose_topic').value)
        self._predock_pub = self.create_publisher(PoseStamped, predock_topic, 10)

        delay = float(self.get_parameter('start_delay_s').value)
        if bool(self.get_parameter('auto_start').value):
            self._start_timer = self.create_timer(
                max(1.0, delay),
                self._on_start_timer,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )

        self.get_logger().info(
            'autocharge demo: predock -> dock -> leave predock -> origin '
            '(start in %.0fs)' % delay
        )

    def _on_start_timer(self) -> None:
        if self._phase != DemoPhase.WAIT:
            return
        if self._start_timer is not None:
            self._start_timer.cancel()
            self._start_timer = None
        self._begin_demo()

    def _begin_demo(self) -> None:
        if self._phase != DemoPhase.WAIT:
            return
        wait_s = float(self.get_parameter('server_wait_timeout_s').value)
        if not self._nav_client.wait_for_server(timeout_sec=wait_s):
            self.get_logger().error('navigate_to_pose action server unavailable')
            self._phase = DemoPhase.FAILED
            return
        if not self._dock_client.wait_for_server(timeout_sec=wait_s):
            self.get_logger().error('dock_to_charger action server unavailable')
            self._phase = DemoPhase.FAILED
            return
        self._publish_predock_pose()
        px, py, pyaw = self._predock_map()
        self._send_nav_goal(
            DemoPhase.NAV_PREDOCK,
            px,
            py,
            pyaw,
            'predock approach',
        )

    def _charger_map(self) -> tuple[float, float, float]:
        return (
            float(self.get_parameter('charger_x').value),
            float(self.get_parameter('charger_y').value),
            float(self.get_parameter('charger_yaw').value),
        )

    def _predock_map(self) -> tuple[float, float, float]:
        cx, cy, cyaw = self._charger_map()
        return resolve_predock_pose(
            use_explicit_predock=bool(self.get_parameter('use_explicit_predock').value),
            predock_x=float(self.get_parameter('predock_x').value),
            predock_y=float(self.get_parameter('predock_y').value),
            predock_yaw=float(self.get_parameter('predock_yaw').value),
            charger_x=cx,
            charger_y=cy,
            charger_yaw=cyaw,
            predock_distance_m=float(self.get_parameter('predock_distance_m').value),
        )

    def _origin_map(self) -> tuple[float, float, float]:
        return (
            float(self.get_parameter('origin_x').value),
            float(self.get_parameter('origin_y').value),
            float(self.get_parameter('origin_yaw').value),
        )

    def _publish_predock_pose(self) -> None:
        px, py, pyaw = self._predock_map()
        msg = make_pose_stamped(self._map_frame, px, py, pyaw, self.get_clock().now().to_msg())
        self._predock_pub.publish(msg)

    def _send_nav_goal(
        self,
        phase: DemoPhase,
        x: float,
        y: float,
        yaw: float,
        label: str,
    ) -> None:
        goal = NavigateToPose.Goal()
        goal.pose = make_pose_stamped(self._map_frame, x, y, yaw, Time().to_msg())
        self._phase = phase
        self._nav_label = label
        self._nav_retry_count = 0
        self.get_logger().info(
            'Nav2 %s goal map=(%.2f, %.2f, %.1fdeg)' % (label, x, y, math.degrees(yaw))
        )
        self._start_nav_watchdog()
        send_future = self._nav_client.send_goal_async(
            goal,
            feedback_callback=self._on_nav_feedback,
        )
        send_future.add_done_callback(self._on_nav_goal_response)

    def _start_nav_watchdog(self) -> None:
        self._stop_nav_watchdog()
        timeout_s = float(self.get_parameter('nav_timeout_s').value)
        if timeout_s <= 0.0:
            return
        self._nav_watchdog = self.create_timer(
            timeout_s,
            self._on_nav_timeout,
            callback_group=self._cb_group,
        )

    def _stop_nav_watchdog(self) -> None:
        if self._nav_watchdog is not None:
            self._nav_watchdog.cancel()
            self._nav_watchdog = None

    def _on_nav_timeout(self) -> None:
        if self._phase not in (
            DemoPhase.NAV_PREDOCK,
            DemoPhase.NAV_LEAVE_PREDOCK,
            DemoPhase.NAV_ORIGIN,
        ):
            self._stop_nav_watchdog()
            return
        self.get_logger().error(
            'Nav2 %s timeout (%.0fs); cancel goal'
            % (self._nav_label, float(self.get_parameter('nav_timeout_s').value))
        )
        if self._nav_goal_handle is not None:
            cancel_future = self._nav_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda _f: None)
        self._phase = DemoPhase.FAILED
        self._stop_nav_watchdog()

    def _on_nav_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        self.get_logger().info(
            'Nav2 %s feedback: distance_remaining=%.2fm'
            % (self._nav_label, float(fb.distance_remaining)),
            throttle_duration_sec=3.0,
        )

    def _on_nav_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 %s goal rejected' % self._nav_label)
            self._phase = DemoPhase.FAILED
            self._stop_nav_watchdog()
            return
        self._nav_goal_handle = goal_handle
        self.get_logger().info('Nav2 %s goal accepted' % self._nav_label)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future) -> None:
        self._stop_nav_watchdog()
        result = future.result()
        status = int(result.status)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._on_nav_succeeded()
            return
        if (
            status == GoalStatus.STATUS_CANCELED
            and bool(self.get_parameter('nav_retry_on_cancel').value)
            and self._nav_retry_count < 1
            and self._phase == DemoPhase.NAV_PREDOCK
        ):
            self._nav_retry_count += 1
            self.get_logger().warning(
                'Nav2 %s canceled (do not use RViz "2D Goal Pose"); retrying once'
                % self._nav_label
            )
            self._phase = DemoPhase.WAIT
            retry_timer = self.create_timer(
                2.0,
                self._retry_nav_once,
                callback_group=self._cb_group,
            )
            self._retry_timer = retry_timer
            return
        self.get_logger().error(
            'Nav2 %s failed, status=%d (%s)'
            % (self._nav_label, status, self._status_name(status))
        )
        self._phase = DemoPhase.FAILED

    def _on_nav_succeeded(self) -> None:
        if self._phase == DemoPhase.NAV_PREDOCK:
            self.get_logger().info('Nav2 predock succeeded, starting dock_to_charger')
            self._send_dock_action()
            return
        if self._phase == DemoPhase.NAV_LEAVE_PREDOCK:
            ox, oy, oyaw = self._origin_map()
            self.get_logger().info('Left charger at predock, navigating to map origin')
            self._send_nav_goal(DemoPhase.NAV_ORIGIN, ox, oy, oyaw, 'map origin')
            return
        if self._phase == DemoPhase.NAV_ORIGIN:
            self.get_logger().info('autocharge demo mission complete (origin reached)')
            self._phase = DemoPhase.DONE
            return
        self.get_logger().warning('Nav2 succeeded in unexpected phase %s' % self._phase.name)
        self._phase = DemoPhase.FAILED

    def _retry_nav_once(self) -> None:
        if hasattr(self, '_retry_timer') and self._retry_timer is not None:
            self._retry_timer.cancel()
            self._retry_timer = None
        if self._phase != DemoPhase.WAIT:
            return
        px, py, pyaw = self._predock_map()
        self._send_nav_goal(DemoPhase.NAV_PREDOCK, px, py, pyaw, 'predock approach')

    @staticmethod
    def _status_name(status: int) -> str:
        names = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        return names.get(status, 'status_%d' % status)

    def _transform_pose_map_to_odom(self, pose_map: PoseStamped) -> PoseStamped | None:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._odom_frame,
                self._map_frame,
                Time(),
            )
        except Exception as exc:
            self.get_logger().error('map->odom TF unavailable: %s' % exc)
            return None
        out = PoseStamped()
        out.header.frame_id = self._odom_frame
        out.header.stamp = self.get_clock().now().to_msg()
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        tf_yaw = quaternion_to_yaw(tf.transform.rotation)
        c = math.cos(tf_yaw)
        s = math.sin(tf_yaw)
        mx = pose_map.pose.position.x
        my = pose_map.pose.position.y
        out.pose.position.x = c * mx - s * my + tx
        out.pose.position.y = s * mx + c * my + ty
        map_yaw = quaternion_to_yaw(pose_map.pose.orientation)
        out_yaw = wrap_pi(map_yaw + tf_yaw)
        out.pose.orientation.z = math.sin(out_yaw * 0.5)
        out.pose.orientation.w = math.cos(out_yaw * 0.5)
        return out

    def _send_dock_action(self) -> None:
        cx, cy, cyaw = self._charger_map()
        charger_map = make_pose_stamped(
            self._map_frame,
            cx,
            cy,
            cyaw,
            Time().to_msg(),
        )
        charger_odom = self._transform_pose_map_to_odom(charger_map)
        if charger_odom is None:
            self._phase = DemoPhase.FAILED
            return
        goal = AutoCharge.Goal()
        goal.charger_pose = charger_odom
        goal.timeout = float(self.get_parameter('dock_timeout_s').value)
        self.get_logger().info(
            'dock_to_charger goal odom=(%.2f, %.2f, %.1fdeg)'
            % (
                charger_odom.pose.position.x,
                charger_odom.pose.position.y,
                math.degrees(quaternion_to_yaw(charger_odom.pose.orientation)),
            )
        )
        self._phase = DemoPhase.DOCK
        send_future = self._dock_client.send_goal_async(
            goal,
            feedback_callback=self._on_dock_feedback,
        )
        send_future.add_done_callback(self._on_dock_goal_response)

    def _on_dock_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        self.get_logger().info(
            'dock feedback state=%d progress=%.2f %s'
            % (int(fb.state), float(fb.progress), str(fb.status_msg)),
            throttle_duration_sec=2.0,
        )

    def _on_dock_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('dock_to_charger goal rejected')
            self._phase = DemoPhase.FAILED
            return
        self._dock_goal_handle = goal_handle
        self.get_logger().info('dock_to_charger goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_dock_result)

    def _on_dock_result(self, future) -> None:
        result = future.result().result
        if not result.success:
            self.get_logger().error(
                'dock_to_charger FAILED code=%d: %s'
                % (int(result.error_code), str(result.message))
            )
            self._phase = DemoPhase.FAILED
            return
        self.get_logger().info(
            'dock_to_charger SUCCESS in %.1fs: %s'
            % (float(result.total_elapsed_seconds), str(result.message))
        )
        if not bool(self.get_parameter('run_return_mission').value):
            self.get_logger().info('run_return_mission=false; demo done after dock')
            self._phase = DemoPhase.DONE
            return
        self._begin_post_dock_hold()

    def _begin_post_dock_hold(self) -> None:
        hold_s = float(self.get_parameter('post_dock_hold_s').value)
        if hold_s <= 0.0:
            self._begin_leave_predock()
            return
        self._phase = DemoPhase.CHARGE_HOLD
        self.get_logger().info('Charging hold %.1fs before leaving to predock' % hold_s)
        if self._hold_timer is not None:
            self._hold_timer.cancel()
        self._hold_timer = self.create_timer(
            hold_s,
            self._on_hold_timer,
            callback_group=self._cb_group,
        )

    def _on_hold_timer(self) -> None:
        if self._hold_timer is not None:
            self._hold_timer.cancel()
            self._hold_timer = None
        if self._phase != DemoPhase.CHARGE_HOLD:
            return
        self._begin_leave_predock()

    def _begin_leave_predock(self) -> None:
        px, py, pyaw = self._predock_map()
        self.get_logger().info(
            'Leaving charger: Nav2 to predock (%.2f, %.2f, %.1fdeg)'
            % (px, py, math.degrees(pyaw))
        )
        self._publish_predock_pose()
        self._send_nav_goal(
            DemoPhase.NAV_LEAVE_PREDOCK,
            px,
            py,
            pyaw,
            'leave charger to predock',
        )


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def main() -> None:
    rclpy.init()
    node = AutochargeDemoNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
