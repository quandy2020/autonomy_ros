"""External command I/O: mission topics, action server, sensors, and cmd_vel output."""

from __future__ import annotations

import math
import threading
from typing import TYPE_CHECKING, Callable, Optional

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist
from jdbot_interfaces.action import AutoCharge
from jdbot_interfaces.msg import DockIR
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool

from autocharge.common.geometry import quat_to_yaw, yaw_to_quat
from autocharge.docking.type import DockState, MissionPhase

if TYPE_CHECKING:
    from autocharge.docking.node import DockingNode


class DockCmd:
    """Handles external subscriptions, dock_to_charger action, and cmd_vel output."""

    def __init__(self, node: Node) -> None:
        self.node = node
        self._cmd_pub = None
        self._initialpose_pub = None
        self._action_server = None
        self._action_cb_group = ReentrantCallbackGroup()
        self._goal_handle = None
        self._goal_done = threading.Event()
        self._action_result: Optional[AutoCharge.Result] = None
        self._default_action_timeout_s = 300.0
        self._cmd_vx = 0.0
        self._cmd_wz = 0.0

    def setup(
        self,
        *,
        ir_topic: str,
        on_ir: Callable[[DockIR], None],
        on_odom: Callable[[Odometry], None],
    ) -> None:
        node = self.node
        self._default_action_timeout_s = max(
            1.0, float(node.get_parameter('action_default_timeout_s').value)
        )
        node.create_subscription(
            DockIR,
            ir_topic,
            on_ir,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            Bool,
            str(node.get_parameter('charge_topic').value),
            self.on_charge,
            10,
        )
        node.create_subscription(
            Bool,
            str(node.get_parameter('abort_topic').value),
            self.on_abort,
            10,
        )
        node.create_subscription(
            Bool,
            str(node.get_parameter('start_topic').value),
            self.on_start,
            10,
        )
        node.create_subscription(
            Bool,
            str(node.get_parameter('reset_topic').value),
            self.on_reset,
            10,
        )
        node.create_subscription(
            Odometry,
            str(node.get_parameter('odom_topic').value),
            on_odom,
            qos_profile_sensor_data,
        )
        self._cmd_pub = node.create_publisher(
            Twist, str(node.get_parameter('cmd_vel_topic').value), 10
        )
        self._initialpose_pub = node.create_publisher(
            PoseWithCovarianceStamped,
            str(node.get_parameter('initialpose_topic').value),
            10,
        )
        action_name = str(node.get_parameter('dock_to_charger_action').value).strip()
        if action_name:
            self._action_server = ActionServer(
                node,
                AutoCharge,
                action_name,
                self._execute_callback,
                goal_callback=self._goal_callback,
                cancel_callback=self._cancel_callback,
                callback_group=self._action_cb_group,
            )
            node.get_logger().info('AutoCharge action server: %s' % action_name)

    def reset_filter(self) -> None:
        self._cmd_vx = 0.0
        self._cmd_wz = 0.0

    def publish_twist(
        self,
        vx: float,
        wz: float,
        *,
        immediate: bool = False,
    ) -> tuple[float, float]:
        if self._cmd_pub is None:
            return (0.0, 0.0)
        if immediate:
            self.reset_filter()
            vx_out = 0.0
            wz_out = 0.0
        else:
            alpha = max(
                0.0,
                min(0.99, float(self.node.get_parameter('cmd_vel_alpha').value)),
            )
            self._cmd_vx = alpha * self._cmd_vx + (1.0 - alpha) * float(vx)
            self._cmd_wz = alpha * self._cmd_wz + (1.0 - alpha) * float(wz)
            vx_out = self._cmd_vx
            wz_out = self._cmd_wz
        tw = Twist()
        tw.linear.x = vx_out
        tw.angular.z = wz_out
        self._cmd_pub.publish(tw)
        return vx_out, wz_out

    def publish_reset_pose(self, x: float, y: float, yaw: float) -> None:
        if self._initialpose_pub is None:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quat(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.02
        self._initialpose_pub.publish(msg)
        self.node.get_logger().info(
            'reset robot pose published: x=%.3f y=%.3f yaw=%.3f' % (x, y, yaw)
        )

    def publish_action_feedback(
        self,
        *,
        phase: MissionPhase,
        fsm_state: DockState,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        dist_to_dock: Optional[float],
        attempt_count: int,
        remaining_time: float,
    ) -> None:
        if self._goal_handle is None:
            return
        fb = AutoCharge.Feedback()
        fb.state = self._map_feedback_state(phase, fsm_state, dist_to_dock)
        fb.sub_state = self._map_feedback_sub_state(fsm_state)
        fb.progress = self._estimate_progress(phase, fsm_state, dist_to_dock)
        fb.attempt_count = int(attempt_count)
        fb.remaining_time = max(0.0, float(remaining_time))
        fb.current_robot_pose = self._make_robot_pose(robot_x, robot_y, robot_yaw)
        if dist_to_dock is None:
            fb.status_msg = 'phase=%s fsm=%s' % (phase.name, fsm_state.name)
        else:
            fb.status_msg = 'phase=%s fsm=%s dist=%.3fm' % (
                phase.name,
                fsm_state.name,
                float(dist_to_dock),
            )
        self._goal_handle.publish_feedback(fb)

    def finish_action(
        self,
        *,
        success: bool,
        error_code: int,
        message: str,
        elapsed_s: float,
    ) -> None:
        result = AutoCharge.Result()
        result.success = bool(success)
        result.error_code = int(error_code)
        result.message = str(message)
        result.total_elapsed_seconds = max(0.0, float(elapsed_s))
        self._action_result = result
        self._goal_done.set()

    def on_charge(self, msg: Bool) -> None:
        self._host().mission.charge_connected = bool(msg.data)

    def on_abort(self, msg: Bool) -> None:
        host = self._host()
        if bool(msg.data):
            host.mission.abort = True
            if host.mission.action.active:
                host.mission.action.cancel_requested = True
            return
        host.mission.abort = False

    def on_reset(self, msg: Bool) -> None:
        if not bool(msg.data):
            return
        host = self._host()
        if host.mission.action.active:
            self.node.get_logger().warn(
                'ignore /dock/reset while dock_to_charger action is active; cancel action first'
            )
            return
        host.reset(reset_robot_pose=host.mission.reset_pose_on_reset)
        self.publish_twist(0.0, 0.0, immediate=True)
        self.node.get_logger().info('docking reset requested')

    def on_start(self, msg: Bool) -> None:
        host = self._host()
        if bool(msg.data):
            if host.mission.action.active:
                self.node.get_logger().warn(
                    'ignore /dock/start=true while dock_to_charger action is active'
                )
                return
            was_active = bool(host.mission.active)
            terminal = host.fsm.state in (DockState.SUCCESS, DockState.ABORT)
            if was_active and not terminal:
                self.node.get_logger().info(
                    'docking restart requested (was %s/%s)'
                    % (host.mission.phase.name, host.fsm.state.name)
                )
            host.reset(keep_active=False, reset_robot_pose=False)
            host.begin_mission()
            self.node.get_logger().info('docking start requested')
            return
        if host.mission.action.active:
            self.node.get_logger().warn(
                'ignore /dock/start=false while dock_to_charger action is active; use action cancel'
            )
            return
        host.mission.active = False
        host.mission.phase = MissionPhase.IDLE
        self.publish_twist(0.0, 0.0, immediate=True)
        self.node.get_logger().info('docking paused by start topic=false')

    def _goal_callback(self, goal_request: AutoCharge.Goal) -> GoalResponse:
        host = self._host()
        if host.mission.active:
            self.node.get_logger().warn('reject dock_to_charger: mission already active')
            return GoalResponse.REJECT
        if self._goal_handle is not None:
            self.node.get_logger().warn('reject dock_to_charger: previous goal still tracked')
            return GoalResponse.REJECT
        timeout_s = float(goal_request.timeout)
        if timeout_s <= 0.0:
            timeout_s = self._default_action_timeout_s
        self.node.get_logger().info(
            'accept dock_to_charger goal timeout=%.1fs frame=%s'
            % (timeout_s, str(goal_request.charger_pose.header.frame_id))
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        host = self._host()
        host.mission.action.cancel_requested = True
        host.mission.abort = True
        self.node.get_logger().info('dock_to_charger cancel requested')
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        host = self._host()
        req = goal_handle.request
        timeout_s = float(req.timeout)
        if timeout_s <= 0.0:
            timeout_s = self._default_action_timeout_s

        self._goal_handle = goal_handle
        self._goal_done.clear()
        self._action_result = None

        if not host.start_action_mission(req.charger_pose, timeout_s):
            result = self._make_result(
                success=False,
                error_code=AutoCharge.Result.ERROR_OTHER,
                message='failed to start docking mission',
                elapsed_s=0.0,
            )
            goal_handle.abort()
            self._goal_handle = None
            return result

        finished = self._goal_done.wait(timeout=timeout_s + 1.0)
        if (
            (not finished)
            and host.mission.active
            and (not host.mission.action.timed_out)
        ):
            host.handle_action_timeout()
            self._goal_done.wait(timeout=2.0)

        result = self._action_result
        if result is None:
            result = self._make_result(
                success=False,
                error_code=AutoCharge.Result.ERROR_TIMEOUT,
                message='action timed out before result was produced',
                elapsed_s=host.action_elapsed_s(),
            )

        if goal_handle.is_cancel_requested or host.mission.action.cancel_requested:
            goal_handle.canceled()
        elif result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        self._goal_handle = None
        host.clear_action_context()
        return result

    @staticmethod
    def _make_result(
        *,
        success: bool,
        error_code: int,
        message: str,
        elapsed_s: float,
    ) -> AutoCharge.Result:
        result = AutoCharge.Result()
        result.success = bool(success)
        result.error_code = int(error_code)
        result.message = str(message)
        result.total_elapsed_seconds = max(0.0, float(elapsed_s))
        return result

    @staticmethod
    def _make_robot_pose(x: float, y: float, yaw: float) -> Pose:
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quat(yaw)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    @staticmethod
    def _map_feedback_state(
        phase: MissionPhase,
        fsm_state: DockState,
        dist_to_dock: Optional[float],
    ) -> int:
        if fsm_state == DockState.SUCCESS:
            return AutoCharge.Feedback.STATE_DOCKED
        if fsm_state == DockState.ABORT:
            return AutoCharge.Feedback.STATE_ABORTING
        if phase == MissionPhase.IDLE:
            return AutoCharge.Feedback.STATE_IDLE
        if phase == MissionPhase.PREDOCK:
            return AutoCharge.Feedback.STATE_NAVIGATING
        if fsm_state == DockState.SEARCH:
            return AutoCharge.Feedback.STATE_SEARCHING
        if fsm_state in (
            DockState.BIAS_LEFT_FAR,
            DockState.BIAS_RIGHT_FAR,
            DockState.BIAS_LEFT,
            DockState.BIAS_RIGHT,
        ):
            return AutoCharge.Feedback.STATE_ALIGNING
        if fsm_state == DockState.CENTER:
            if dist_to_dock is not None and float(dist_to_dock) <= 0.08:
                return AutoCharge.Feedback.STATE_DOCKING
            return AutoCharge.Feedback.STATE_APPROACHING
        return AutoCharge.Feedback.STATE_IDLE

    @staticmethod
    def _map_feedback_sub_state(fsm_state: DockState) -> int:
        mapping = {
            DockState.SEARCH: 1,
            DockState.BIAS_LEFT_FAR: 2,
            DockState.BIAS_RIGHT_FAR: 3,
            DockState.BIAS_LEFT: 4,
            DockState.BIAS_RIGHT: 5,
            DockState.CENTER: 6,
            DockState.SUCCESS: 7,
            DockState.ABORT: 8,
        }
        return int(mapping.get(fsm_state, 0))

    @staticmethod
    def _estimate_progress(
        phase: MissionPhase,
        fsm_state: DockState,
        dist_to_dock: Optional[float],
    ) -> float:
        if fsm_state == DockState.SUCCESS:
            return 1.0
        if fsm_state == DockState.ABORT:
            return 0.0
        if phase == MissionPhase.PREDOCK:
            return 0.15
        if fsm_state == DockState.SEARCH:
            return 0.35
        if fsm_state in (
            DockState.BIAS_LEFT_FAR,
            DockState.BIAS_RIGHT_FAR,
            DockState.BIAS_LEFT,
            DockState.BIAS_RIGHT,
        ):
            return 0.55
        if fsm_state == DockState.CENTER:
            if dist_to_dock is None:
                return 0.75
            return float(max(0.70, min(0.95, 1.0 - float(dist_to_dock) / 0.5)))
        return 0.05

    @staticmethod
    def parse_odom(msg: Odometry) -> tuple[float, float, float]:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(float(q.x), float(q.y), float(q.z), float(q.w))
        return x, y, yaw

    def _host(self) -> DockingNode:
        return self.node  # type: ignore[return-value]
