"""Auto-docking node: optional predock + strategy-1 IR docking FSM."""

from __future__ import annotations

from typing import Optional
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from jdbot_interfaces.action import AutoCharge
from jdbot_interfaces.msg import DockIR
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from autocharge.common.geometry import quat_to_yaw, wrap_pi
from autocharge.docking.cmd import DockCmd
from autocharge.docking.config import declare_parameters, load_runtime
from autocharge.docking.dock_pose_refine import DockPoseRefiner
from autocharge.docking.fsm import DockingFSM
from autocharge.docking.utils import IrToken, body_contact_point
from autocharge.docking.predock import PredockGoalSampler
from autocharge.docking.type import (
    ActionMissionContext,
    DockObservation,
    DockState,
    MissionPhase,
    MissionState,
    PredockGoal,
)
from autocharge.docking.visualization import DockVisualization


class DockingNode(Node):
    """Orchestrates optional predock navigation and IR docking."""

    def __init__(self) -> None:
        super().__init__('docking_node')
        declare_parameters(self)

        self.mission = MissionState()
        self.cmd = DockCmd(self)
        self.viz = DockVisualization(self)

        runtime = load_runtime(self)
        self.runtime = runtime
        self.geom = runtime.geom
        self.fsm_cfg = runtime.fsm_cfg
        self.fsm = DockingFSM(self.fsm_cfg)
        self.dock_refine = DockPoseRefiner()
        self.ir_filter = runtime.ir_filter
        self.predock_enable_nav = runtime.predock_enable_nav
        self.predock_navigator = runtime.predock_navigator
        self.stats_enable = runtime.stats_enable
        self.control_log = runtime.control_log
        self.start_reset_fsm = runtime.start_reset_fsm
        self.reset_pose_x = runtime.reset_pose_x
        self.reset_pose_y = runtime.reset_pose_y
        self.reset_pose_yaw = runtime.reset_pose_yaw
        self.ir_eval_every_cycles = runtime.ir_eval_every_cycles
        self.fail_on_dock_overrun_enable = runtime.fail_on_dock_overrun_enable
        self.fail_on_dock_overrun_margin_m = runtime.fail_on_dock_overrun_margin_m
        self.mission.reset_pose_on_reset = runtime.reset_pose_on_reset
        self.mission.active = runtime.start_on_boot

        self.ir_left = 0
        self.ir_right = 0
        self.ir_code = 0
        self.ir_message = ''
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.predock_goal: Optional[PredockGoal] = None
        self.control_cycle_count = 0
        self.last_t = self.get_clock().now()

        self.cmd.setup(
            ir_topic=runtime.ir_topic,
            on_ir=self.on_ir_dock,
            on_odom=self.on_odom,
        )
        self.viz.setup()
        self._control_cb_group = ReentrantCallbackGroup()
        self._timer = self.create_timer(
            1.0 / runtime.control_hz,
            self.on_control,
            callback_group=self._control_cb_group,
        )
        self.get_logger().info('autocharge docking node started, ir=%s' % runtime.ir_topic)

        if self.mission.active:
            self.begin_mission()

    def on_ir_dock(self, msg: DockIR) -> None:
        l_raw = int(msg.left) & 0xFF
        r_raw = int(msg.right) & 0xFF
        if (l_raw & 0xE0) != 0 or (r_raw & 0xE0) != 0:
            self.get_logger().warning(
                'illegal IR high bits: left=0x%02x right=0x%02x code=%d msg=%s'
                % (l_raw, r_raw, int(msg.code), str(msg.message)),
                throttle_duration_sec=1.0,
            )
        l_val, r_val, unknown = self.ir_filter.validate_raw(l_raw, r_raw)
        if unknown:
            self.get_logger().warning(
                'unknown IR token (low5): L=0x%02x R=0x%02x' % (l_val, r_val),
                throttle_duration_sec=1.0,
            )
        self.ir_left = l_val
        self.ir_right = r_val
        self.ir_code = int(msg.code)
        self.ir_message = str(msg.message)

    def on_odom(self, msg: Odometry) -> None:
        self.robot_x, self.robot_y, self.robot_yaw = DockCmd.parse_odom(msg)

    def apply_charger_pose(self, charger_pose: PoseStamped) -> None:
        """Apply goal charger / predock pose to dock geometry (odom frame)."""
        p = charger_pose.pose.position
        q = charger_pose.pose.orientation
        yaw = quat_to_yaw(float(q.x), float(q.y), float(q.z), float(q.w))
        self.geom.dock_x = float(p.x)
        self.geom.dock_y = float(p.y)
        self.geom.dock_yaw = float(yaw)
        self.geom.capture_defaults()
        self.get_logger().info(
            'action charger_pose applied: dock=(%.3f, %.3f, %.1fdeg) frame=%s'
            % (
                self.geom.dock_x,
                self.geom.dock_y,
                math.degrees(self.geom.dock_yaw),
                str(charger_pose.header.frame_id),
            )
        )

    def start_action_mission(self, charger_pose: PoseStamped, timeout_s: float) -> bool:
        if self.mission.active:
            return False
        self.apply_charger_pose(charger_pose)
        act = self.mission.action
        act.active = True
        act.timeout_s = max(1.0, float(timeout_s))
        act.start_time_ns = int(self.get_clock().now().nanoseconds)
        act.attempt_count = 0
        act.cancel_requested = False
        act.timed_out = False
        act.overrun_abort = False
        self.reset(keep_active=False, reset_robot_pose=False)
        self.begin_mission()
        return bool(self.mission.active)

    def action_elapsed_s(self) -> float:
        act = self.mission.action
        if (not act.active) or act.start_time_ns <= 0:
            return 0.0
        return max(
            0.0,
            (int(self.get_clock().now().nanoseconds) - act.start_time_ns) * 1e-9,
        )

    def action_remaining_s(self) -> float:
        act = self.mission.action
        if not act.active:
            return 0.0
        return max(0.0, float(act.timeout_s) - self.action_elapsed_s())

    def clear_action_context(self) -> None:
        self.mission.action = ActionMissionContext()

    def handle_action_timeout(self) -> None:
        if not self.mission.action.active:
            return
        self.mission.action.timed_out = True
        self.mission.abort = True
        self.get_logger().warning(
            'dock_to_charger timeout after %.1fs' % self.action_elapsed_s()
        )

    def _action_timed_out(self) -> bool:
        act = self.mission.action
        if (not act.active) or act.timeout_s <= 0.0:
            return False
        return self.action_elapsed_s() >= float(act.timeout_s)

    def _resolve_action_error_code(self) -> int:
        act = self.mission.action
        if act.cancel_requested:
            return AutoCharge.Result.ERROR_CANCELED
        if act.timed_out:
            return AutoCharge.Result.ERROR_TIMEOUT
        if act.overrun_abort:
            return AutoCharge.Result.ERROR_IR_OR_CONTACT
        if self.mission.phase == MissionPhase.PREDOCK:
            return AutoCharge.Result.ERROR_NAVIGATION
        return AutoCharge.Result.ERROR_APPROACH_STUCK

    def _publish_action_feedback(self, dist_to_dock: Optional[float]) -> None:
        if not self.mission.action.active:
            return
        self.cmd.publish_action_feedback(
            phase=self.mission.phase,
            fsm_state=self.fsm.state,
            robot_x=self.robot_x,
            robot_y=self.robot_y,
            robot_yaw=self.robot_yaw,
            dist_to_dock=dist_to_dock,
            attempt_count=self.mission.action.attempt_count,
            remaining_time=self.action_remaining_s(),
        )

    def _finish_action_if_needed(self, *, success: bool, message: str) -> None:
        if not self.mission.action.active:
            return
        error_code = (
            AutoCharge.Result.ERROR_NONE
            if success
            else self._resolve_action_error_code()
        )
        self.cmd.finish_action(
            success=success,
            error_code=error_code,
            message=message,
            elapsed_s=self.action_elapsed_s(),
        )

    def begin_mission(self) -> None:
        # Clear sticky terminal flags from the previous run.
        self.mission.charge_connected = False
        self.mission.abort = False
        if self.start_reset_fsm:
            self.fsm.reset()
            self.ir_filter.reset()
        self.dock_refine.reset()
        if self.predock_enable_nav:
            self.prepare_predock_goal()
            if self.predock_goal is not None and not self.predock_navigator.goal_reached(
                self.robot_x,
                self.robot_y,
                self.robot_yaw,
                self.predock_goal,
            ):
                self.mission.phase = MissionPhase.PREDOCK
                self.mission.active = True
                self.get_logger().info('predock navigation started')
                return
        self.begin_docking_phase()

    def begin_docking_phase(self) -> None:
        if self.start_reset_fsm:
            self.fsm.reset()
            self.ir_filter.reset()
        self.dock_refine.reset()
        self.mission.charge_connected = False
        self.mission.abort = False
        self.mission.phase = MissionPhase.DOCKING
        self.mission.active = True
        self.last_t = self.get_clock().now()
        self.get_logger().info('IR docking phase started')

    def prepare_predock_goal(self) -> None:
        distance_m = self.runtime.predock_distance_m
        if self.mission.action.active:
            self.predock_goal = PredockGoalSampler.from_dock(
                self.geom.dock_x,
                self.geom.dock_y,
                self.geom.dock_yaw,
                distance_m,
            )
        else:
            self.predock_goal = PredockGoalSampler.sample(
                self.geom.dock_x,
                self.geom.dock_y,
                self.geom.dock_yaw,
                distance_m,
                self.runtime.predock_noise,
            )
            self.geom.update_from_predock(
                self.predock_goal.x,
                self.predock_goal.y,
                self.predock_goal.yaw,
                distance_m,
            )
        self.viz.publish_predock_pose(self.predock_goal)

        self.get_logger().info(
            'predock goal=(%.3f,%.3f,%.1fdeg) robot=(%.3f,%.3f,%.1fdeg)'
            % (
                self.predock_goal.x,
                self.predock_goal.y,
                math.degrees(self.predock_goal.yaw),
                self.robot_x,
                self.robot_y,
                math.degrees(self.robot_yaw),
            )
        )

    def tick_predock(self) -> None:
        if self.predock_goal is None:
            self.begin_docking_phase()
            return
        goal = self.predock_goal
        if self.predock_navigator.goal_reached(
            self.robot_x,
            self.robot_y,
            self.robot_yaw,
            goal,
        ):
            self.cmd.publish_twist(0.0, 0.0, immediate=True)
            self.get_logger().info('predock reached')
            self.begin_docking_phase()
            return
        vx, wz = self.predock_navigator.compute_cmd(
            self.robot_x,
            self.robot_y,
            self.robot_yaw,
            goal,
        )
        vx_out, wz_out = self.cmd.publish_twist(vx, wz)
        if self.control_log:
            import math

            from autocharge.common.geometry import wrap_pi

            rho = math.hypot(goal.x - self.robot_x, goal.y - self.robot_y)
            yaw_err = wrap_pi(goal.yaw - self.robot_yaw)
            self.get_logger().info(
                'phase=PREDOCK cmd(vx=%.3f,wz=%.3f) rho=%.3f yaw_err=%.1fdeg '
                'pose=(%.3f,%.3f,%.1fdeg) goal=(%.3f,%.3f,%.1fdeg) raw_ir=(%s,%s)'
                % (
                    vx_out,
                    wz_out,
                    rho,
                    math.degrees(yaw_err),
                    self.robot_x,
                    self.robot_y,
                    math.degrees(self.robot_yaw),
                    goal.x,
                    goal.y,
                    math.degrees(goal.yaw),
                    IrToken.format_combo(int(self.ir_left)),
                    IrToken.format_combo(int(self.ir_right)),
                ),
                throttle_duration_sec=0.25,
            )
        self._publish_action_feedback(None)

    def on_control(self) -> None:
        if self.mission.action.active and self.mission.active and self._action_timed_out():
            self.handle_action_timeout()
        if not self.mission.active:
            return
        if self.mission.phase == MissionPhase.PREDOCK:
            self.tick_predock()
            return
        if self.mission.phase != MissionPhase.DOCKING:
            return
        self.tick_docking()

    def _maybe_abort_on_dock_overrun(self) -> None:
        if not self.fail_on_dock_overrun_enable:
            return
        if self.mission.charge_connected:
            return
        if self.fsm.state in (DockState.SUCCESS, DockState.ABORT):
            return
        gap = self.geom.signed_gap_to_dock_front(
            self.robot_x, self.robot_y, self.robot_yaw
        )
        if gap is None:
            return
        if gap >= -float(self.fail_on_dock_overrun_margin_m):
            return
        if not self.mission.abort:
            self.get_logger().warning(
                'dock overrun abort: signed_gap_to_front=%.4f m margin=%.4f m'
                % (gap, self.fail_on_dock_overrun_margin_m)
            )
        self.mission.action.overrun_abort = True
        self.mission.abort = True

    def tick_docking(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now
        if dt <= 0.0:
            dt = 0.05

        self._maybe_abort_on_dock_overrun()

        dist_to_dock = self.geom.distance_to_dock(self.robot_x, self.robot_y, self.robot_yaw)
        self.control_cycle_count += 1
        ir_l, ir_r = self.ir_filter.eval_tokens(
            self.control_cycle_count,
            self.ir_eval_every_cycles,
            self.ir_left,
            self.ir_right,
            self.robot_x,
            self.robot_y,
            self.robot_yaw,
            self.geom.dock_x,
            self.geom.dock_y,
        )

        dock_x, dock_y, dock_yaw = self.dock_refine.refine(
            self.geom.dock_x,
            self.geom.dock_y,
            self.geom.dock_yaw,
            int(ir_l),
            int(ir_r),
            self.fsm_cfg,
            dist_to_dock,
            dt,
        )

        obs = DockObservation(
            left_token=int(ir_l),
            right_token=int(ir_r),
            dist_m=float(dist_to_dock) if dist_to_dock is not None else None,
            yaw_rad=float(self.robot_yaw),
            odom_x=float(self.robot_x),
            odom_y=float(self.robot_y),
            lateral_m=float(
                self._lateral_to_dock(dock_x, dock_y, dock_yaw)
            ),
            yaw_err_rad=float(wrap_pi(self.robot_yaw - dock_yaw)),
            dock_x=float(dock_x),
            dock_y=float(dock_y),
            dock_yaw=float(dock_yaw),
            charge_connected=bool(self.mission.charge_connected),
            abort=bool(self.mission.abort),
        )
        cmd = self.fsm.tick(dt, obs)

        for evt in self.fsm.drain_debug_events():
            self.get_logger().info('fsm_event: %s' % evt)

        if self.fsm.state == DockState.SUCCESS:
            self._handle_terminal_state(success=True)
            vx_out, wz_out = 0.0, 0.0
        elif self.fsm.state == DockState.ABORT:
            self._handle_terminal_state(success=False)
            vx_out, wz_out = 0.0, 0.0
        else:
            vx_out, wz_out = self.cmd.publish_twist(float(cmd.vx), float(cmd.wz))

        self.viz.publish_estimated_dock_pose(dock_x, dock_y, dock_yaw)
        self.viz.publish_dock_state(self.mission.phase, self.fsm.state)
        self.viz.publish_dock_markers(
            fsm_state=self.fsm.state,
            dock_x=float(dock_x),
            dock_y=float(dock_y),
            dock_yaw=float(dock_yaw),
        )
        self.viz.publish_test_metrics(
            enabled=self.stats_enable,
            fsm_state=self.fsm.state,
            dist_to_dock=dist_to_dock,
            lateral_y_err=self._lateral_to_dock(dock_x, dock_y, dock_yaw),
            yaw_err_rad=self.geom.yaw_error(self.robot_yaw),
            ir_left=int(self.ir_left),
            ir_right=int(self.ir_right),
        )
        self.viz.log_control(
            enabled=self.control_log,
            fsm_state_name=self.fsm.state.name,
            vx_out=vx_out,
            wz_out=wz_out,
            ir_l=int(ir_l),
            ir_r=int(ir_r),
            ir_left_raw=int(self.ir_left),
            ir_right_raw=int(self.ir_right),
            dist_to_dock=dist_to_dock,
            format_token=IrToken.format_combo,
        )
        self._publish_action_feedback(dist_to_dock)

    def _handle_terminal_state(self, *, success: bool) -> None:
        self.cmd.publish_twist(0.0, 0.0, immediate=True)
        if self.mission.active:
            self.mission.active = False
            self.mission.phase = MissionPhase.IDLE
        if success:
            self.get_logger().info('docking success; idle until next start/action')
            self._finish_action_if_needed(success=True, message='docking succeeded')
        else:
            self.get_logger().info('docking abort; idle until next start/action')
            self._finish_action_if_needed(success=False, message='docking aborted')

    def _lateral_to_dock(self, dock_x: float, dock_y: float, dock_yaw: float) -> float:
        """Front contact lateral error vs IR-refined dock center."""
        del dock_x, dock_yaw
        _, front_y = body_contact_point(
            self.robot_x,
            self.robot_y,
            self.robot_yaw,
            self.geom.success_contact_x_m,
            self.geom.success_contact_y_m,
        )
        return float(front_y) - float(dock_y)

    def reset(self, keep_active: bool = False, reset_robot_pose: bool = False) -> None:
        self.fsm.reset()
        self.ir_filter.reset()
        self.dock_refine.reset()
        self.control_cycle_count = 0
        self.geom.restore_defaults()
        self.ir_left = 0
        self.ir_right = 0
        self.ir_code = 0
        self.ir_message = ''
        self.mission.charge_connected = False
        self.mission.abort = False
        self.mission.phase = MissionPhase.IDLE if not keep_active else self.mission.phase
        self.predock_goal = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.mission.active = bool(keep_active)
        self.last_t = self.get_clock().now()
        self.cmd.reset_filter()
        self.viz.reset_metrics_latch()
        if bool(reset_robot_pose):
            self.cmd.publish_reset_pose(
                self.reset_pose_x,
                self.reset_pose_y,
                self.reset_pose_yaw,
            )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = DockingNode()
    # Action execute blocks until mission ends; control timer must run concurrently.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def run(args: Optional[list[str]] = None) -> None:
    main(args)
