"""Per-robot Nav2 and recording client."""

from __future__ import annotations

import math
import time
from enum import Enum
from typing import TYPE_CHECKING

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import SetBool

from autonomy_task.config import TaskConfig
from autonomy_task.waypoint import Status, Waypoint
from autonomy_task.waypoint_filter import Pose

if TYPE_CHECKING:
    from rclpy.task import Future

_SUCCEEDED = 4
_CANCELED = 5
_ABORTED = 6
_TERMINAL_NAV = frozenset({_SUCCEEDED, _CANCELED, _ABORTED})
_MAX_GOAL_REJECTS = 15
_STATUS_NAME = {
    0: 'unknown', 1: 'accepted', 2: 'executing', 3: 'canceling',
    4: 'succeeded', 5: 'canceled', 6: 'aborted',
}


class Phase(str, Enum):
    IDLE = 'idle'
    PRE_RECORD = 'pre_record'
    NAV = 'nav'
    POST_RECORD = 'post_record'
    SAVING = 'saving'


def _quat_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _pose_from_odom(msg: Odometry) -> Pose:
    p = msg.pose.pose.position
    o = msg.pose.pose.orientation
    yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z))
    return Pose(x=p.x, y=p.y, yaw=yaw)


class Robot:
    """Nav2 navigate_to_pose client with optional LeRobot recording."""

    def __init__(self, node: Node, ns: str, cfg: TaskConfig) -> None:
        self._node = node
        self._ns = ns
        self._cfg = cfg
        cb = ReentrantCallbackGroup()
        self._nav = ActionClient(node, NavigateToPose, f'/{ns}/navigate_to_pose', callback_group=cb)
        self._goal_fut: Future | None = None
        self._result_fut: Future | None = None
        self._handle = None
        self._pose: Pose | None = None
        self._deadline: Time | None = None
        self._nav_deadline: Time | None = None
        self._goal_rejects = 0
        self._episode_t0: Time | None = None
        self._path_m = 0.0
        self._last_xy: tuple[float, float] | None = None
        self._start_pose: Pose | None = None
        self._stall_xy: tuple[float, float] | None = None
        self._stall_deadline: Time | None = None
        self._best_goal_dist: float | None = None
        self._nav_t0: Time | None = None
        self._cancel_reason: str | None = None
        self._cancel_deadline: Time | None = None

        node.create_subscription(Odometry, f'/{ns}/odom', self._on_odom, 10)
        self._rec_cli = None
        if cfg.record.enabled:
            base = f'/{ns}/{cfg.record.bridge}'
            self._rec_cli = node.create_client(SetBool, f'{base}/set_recording', callback_group=cb)

        self.phase = Phase.IDLE
        self.active: Waypoint | None = None
        self.done: Waypoint | None = None
        self._goal_frame = cfg.nav_goal_frame(ns)
        self._recording_active = False
        self._nav_ok = False

    @property
    def recording_active(self) -> bool:
        return self._recording_active

    @property
    def namespace(self) -> str:
        return self._ns

    def set_goal_frame(self, frame: str) -> None:
        if frame:
            self._goal_frame = frame

    def goal_frame(self) -> str:
        fid = self._goal_frame.strip('/')
        if not fid:
            return self._cfg.nav_goal_frame(self._ns)
        return fid.split('/')[-1] or 'map'

    def pose(self) -> Pose | None:
        return self._pose

    def idle(self) -> bool:
        return self.phase == Phase.IDLE and self.active is None

    def nav_elapsed_sec(self) -> float:
        if self._nav_t0 is None:
            return 0.0
        return (self._now() - self._nav_t0).nanoseconds * 1e-9

    def force_fail_nav(self, reason: str) -> str | None:
        if self.phase != Phase.NAV:
            return None
        return self._abort_nav(reason, force=True)

    def pending_servers(self) -> list[str]:
        pending: list[str] = []
        if not self._nav.wait_for_server(timeout_sec=0.0):
            pending.append('navigate_to_pose')
        if self._rec_cli is not None and not self._rec_cli.service_is_ready():
            pending.append('set_recording')
        return pending

    def servers_ready(self) -> bool:
        return not self.pending_servers()

    def wait_ready(self, timeout: float) -> bool:
        if not self._nav.wait_for_server(timeout_sec=timeout):
            self._node.get_logger().error(f'{self._ns}: navigate_to_pose unavailable')
            return False
        if self._rec_cli and not self._rec_cli.wait_for_service(timeout_sec=timeout):
            self._node.get_logger().warning(f'{self._ns}: recording unavailable')
        return True

    def start(self, wp: Waypoint) -> bool:
        """Begin pre-record / nav for *wp*. Returns False if recording could not start."""
        self.active = wp
        wp.status = Status.IN_PROGRESS
        wp.robot = self._ns
        self._episode_t0 = self._now()
        self._path_m = 0.0
        self._last_xy = None
        self._start_pose = self._pose
        t = self._cfg.thresholds
        if self._cfg.record.enabled and self._rec_cli is not None:
            if not self._ensure_recording_started():
                self._node.get_logger().error(
                    f'{self._ns}: recording failed for {wp.id}, aborting assign')
                wp.status = Status.PENDING
                wp.robot = ''
                self.active = None
                self._episode_t0 = None
                return False
        if t.record_before_sec > 0.0:
            self.phase = Phase.PRE_RECORD
            self._deadline = self._after(t.record_before_sec)
        else:
            self._send_goal(wp)
        return True

    def tick(self) -> str | None:
        if self.phase == Phase.SAVING:
            return self._flush_save()
        if self.phase == Phase.IDLE:
            return None
        now = self._now()
        if self.phase == Phase.PRE_RECORD:
            if self.active and self._deadline is not None and now >= self._deadline:
                self._send_goal(self.active)
            return None
        if self.phase == Phase.NAV:
            return self._tick_nav()
        if self.phase == Phase.POST_RECORD and self._deadline is not None and now >= self._deadline:
            return self._finish(True)
        return None

    def trajectory_stats(self) -> dict[str, float]:
        straight = 0.0
        target = self.done or self.active
        if self._start_pose is not None and target is not None:
            straight = target.dist_to(self._start_pose.x, self._start_pose.y)
        duration = 0.0
        if self._episode_t0 is not None:
            duration = (self._now() - self._episode_t0).nanoseconds * 1e-9
        return {
            'path_length_m': self._path_m,
            'straight_line_m': straight,
            'duration_sec': duration,
        }

    def set_recording_sync(self, on: bool, timeout: float | None = None) -> tuple[bool, str]:
        if not self._rec_cli:
            return False, 'recording disabled'
        if not rclpy.ok() or not self._node.context.ok:
            return False, 'shutting down'
        if timeout is None:
            timeout = self._cfg.record.save_timeout_sec
        if not self._rec_cli.service_is_ready():
            if not self._rec_cli.wait_for_service(timeout_sec=timeout):
                return False, 'set_recording not ready'
        req = SetBool.Request()
        req.data = on
        future = self._rec_cli.call_async(req)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                return False, 'set_recording timed out'
            time.sleep(0.02)
        if not future.done():
            return False, 'set_recording timed out'
        try:
            result = future.result()
        except Exception as exc:
            return False, str(exc)
        ok = bool(result.success)
        msg = str(result.message)
        if ok:
            self._recording_active = on
        return ok, msg

    def stop_recording_sync(self) -> tuple[bool, str]:
        """Stop recording and wait for lerobot_bridge to finish save_episode."""
        if not self._recording_active:
            return True, ''
        return self.set_recording_sync(False)

    def _ensure_recording_started(self) -> bool:
        if not self._rec_cli or not self._cfg.record.enabled or self._recording_active:
            return True
        ok, msg = self.set_recording_sync(True)
        if not ok:
            self._node.get_logger().warning(f'{self._ns}: recording start failed: {msg}')
        return ok

    def _stop_recording(self) -> tuple[bool, str]:
        if not self._recording_active:
            return True, ''
        ok, msg = self.set_recording_sync(False)
        if not ok:
            self._node.get_logger().error(f'{self._ns}: recording stop/save failed: {msg}')
        return ok, msg

    def _now(self) -> Time:
        return self._node.get_clock().now()

    def _after(self, sec: float) -> Time:
        return self._now() + Duration(seconds=sec)

    def _on_odom(self, msg: Odometry) -> None:
        pose = _pose_from_odom(msg)
        if self.phase != Phase.IDLE and self.active is not None:
            if self._last_xy is not None:
                dx = pose.x - self._last_xy[0]
                dy = pose.y - self._last_xy[1]
                self._path_m += math.hypot(dx, dy)
            self._last_xy = (pose.x, pose.y)
            if self._start_pose is None:
                self._start_pose = pose
        self._pose = pose

    def _send_goal(self, wp: Waypoint) -> None:
        if self._cfg.thresholds.record_before_sec <= 0.0 and self._cfg.record.enabled:
            if not self._ensure_recording_started():
                self._node.get_logger().error(
                    f'{self._ns}: recording failed before nav to {wp.id}')
                self._finish(False)
                return
        self._try_cancel_goal()
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        frame = self.goal_frame()
        pose.header.frame_id = frame
        pose.header.stamp = self._now().to_msg()
        pose.pose.position.x = wp.x
        pose.pose.position.y = wp.y
        pose.pose.position.z = wp.z
        pose.pose.orientation = _quat_yaw(wp.yaw)
        goal.pose = pose
        self._goal_fut = self._nav.send_goal_async(goal)
        self._handle = None
        self._result_fut = None
        self._cancel_reason = None
        self._cancel_deadline = None
        self._nav_t0 = self._now()
        self._arm_stall()
        self.phase = Phase.NAV
        timeout = self._cfg.nav.action_timeout_sec
        self._nav_deadline = self._after(timeout) if timeout > 0.0 else None
        self._node.get_logger().info(
            f'{self._ns}: nav {wp.id} ({wp.x:.2f}, {wp.y:.2f}) frame={frame}')

    def _goal_dist(self) -> float | None:
        if self._pose is None or self.active is None:
            return None
        return self.active.dist_to(self._pose.x, self._pose.y)

    def _at_goal(self) -> bool:
        dist = self._goal_dist()
        return dist is not None and dist <= self._cfg.nav.goal_tolerance_m

    def _nav_status(self) -> int | None:
        if self._handle is None or not self._handle.accepted:
            return None
        status = int(self._handle.status)
        return status if status in _TERMINAL_NAV else None

    def _try_cancel_goal(self) -> None:
        if self._handle is None:
            return
        try:
            self._handle.cancel_goal_async()
        except Exception:
            pass

    def _reset_stall(self) -> None:
        self._stall_xy = None
        self._stall_deadline = None
        self._best_goal_dist = None

    def _arm_stall(self) -> None:
        if self._pose is None:
            self._reset_stall()
            return
        self._stall_xy = (self._pose.x, self._pose.y)
        self._stall_deadline = self._after(self._cfg.nav.stall_sec)
        dist = self._goal_dist()
        if dist is not None and (self._best_goal_dist is None or dist < self._best_goal_dist):
            self._best_goal_dist = dist

    def _clear_nav(self) -> None:
        self._goal_fut = None
        self._result_fut = None
        self._handle = None
        self._nav_deadline = None
        self._nav_t0 = None
        self._cancel_reason = None
        self._cancel_deadline = None
        self._reset_stall()

    def _abort_nav(self, reason: str, *, force: bool = False) -> str | None:
        if not force and self._cancel_reason is not None:
            return None
        self._cancel_reason = reason
        self._nav_deadline = None
        self._reset_stall()
        timeout = self._cfg.nav.cancel_timeout_sec
        self._cancel_deadline = self._after(timeout) if timeout > 0.0 else None
        self._node.get_logger().warning(f'{self._ns}: cancel nav ({reason})')
        if force:
            self._try_cancel_goal()
            self._clear_nav()
            self._node.get_logger().warning(f'{self._ns}: force fail nav ({reason})')
            return self._finish(False)
        self._try_cancel_goal()
        if self._handle is not None:
            return None
        if self._goal_fut and not self._goal_fut.done():
            return None
        self._cancel_reason = None
        self._cancel_deadline = None
        return self._finish(False)

    def _stall_detected(self, now: Time) -> bool:
        nav = self._cfg.nav
        dist = self._goal_dist()
        if dist is not None and self._best_goal_dist is not None:
            if dist < self._best_goal_dist - nav.stall_goal_progress_m:
                self._best_goal_dist = dist
                self._stall_deadline = self._after(nav.stall_sec)
                return False
            self._node.get_logger().warning(
                f'{self._ns}: nav stalled (<{nav.stall_goal_progress_m:.2f}m toward goal '
                f'in {nav.stall_sec:.0f}s, dist={dist:.2f}m best={self._best_goal_dist:.2f}m)')
            return True
        if self._pose is not None and self._stall_xy is not None:
            moved = math.hypot(self._pose.x - self._stall_xy[0], self._pose.y - self._stall_xy[1])
            if moved < nav.stall_move_m:
                self._node.get_logger().warning(
                    f'{self._ns}: nav stalled (<{nav.stall_move_m:.1f}m in {nav.stall_sec:.0f}s)')
                return True
        return False

    def _complete_nav(self, status: int) -> str | None:
        cancel = self._cancel_reason
        self._clear_nav()
        if cancel:
            return self._finish(False)
        if status == _SUCCEEDED:
            return self._arrive()
        wp_id = self.active.id if self.active else '?'
        name = _STATUS_NAME.get(status, str(status))
        self._node.get_logger().warning(f'{self._ns}: nav failed {wp_id} status={name}')
        return self._finish(False)

    def _arrive(self) -> str | None:
        after = self._cfg.thresholds.record_after_sec
        if after > 0.0:
            self._node.get_logger().info(f'{self._ns}: arrived, post-record {after:.1f}s')
            self.phase = Phase.POST_RECORD
            self._deadline = self._after(after)
            return None
        return self._finish(True)

    def _tick_nav(self) -> str | None:
        now = self._now()
        nav = self._cfg.nav
        if self._cancel_reason and self._cancel_deadline is not None and now >= self._cancel_deadline:
            return self._abort_nav('cancel_timeout', force=True)
        if (
            self._cancel_reason is None
            and nav.max_nav_sec > 0.0
            and self._nav_t0 is not None
            and (now - self._nav_t0).nanoseconds * 1e-9 >= nav.max_nav_sec
        ):
            wp_id = self.active.id if self.active else '?'
            self._node.get_logger().warning(f'{self._ns}: nav exceeded {nav.max_nav_sec:.0f}s on {wp_id}')
            return self._abort_nav('max_nav_duration')
        if self._cancel_reason is None and self._stall_deadline is not None and now >= self._stall_deadline:
            if self._stall_detected(now):
                return self._abort_nav('stall')
            self._stall_deadline = self._after(nav.stall_sec)
        if self._cancel_reason is None and self._nav_deadline is not None and now >= self._nav_deadline:
            self._node.get_logger().warning(
                f'{self._ns}: nav timeout ({nav.action_timeout_sec:.0f}s)')
            return self._abort_nav('timeout')
        if self._goal_fut and not self._goal_fut.done():
            return None
        if self._goal_fut and self._handle is None:
            self._handle = self._goal_fut.result()
            self._goal_fut = None
            if self._cancel_reason:
                if self._handle and self._handle.accepted:
                    self._try_cancel_goal()
                    self._result_fut = self._handle.get_result_async()
                    return None
                self._cancel_reason = None
                return self._finish(False)
            if not self._handle or not self._handle.accepted:
                self._goal_rejects += 1
                if self._goal_rejects <= _MAX_GOAL_REJECTS and self.active:
                    self._node.get_logger().warning(
                        f'{self._ns}: goal rejected, retry '
                        f'{self._goal_rejects}/{_MAX_GOAL_REJECTS}')
                    self._handle = None
                    self._send_goal(self.active)
                    return None
                self._node.get_logger().warning(f'{self._ns}: goal rejected')
                return self._abort_nav('rejected')
            self._goal_rejects = 0
            self._result_fut = self._handle.get_result_async()
            return None
        if self._handle is not None and self._handle.accepted:
            if self._result_fut is None:
                self._result_fut = self._handle.get_result_async()
            if self._result_fut is not None and self._result_fut.done():
                return self._complete_nav(int(self._result_fut.result().status))
            terminal = self._nav_status()
            if terminal is not None:
                return self._complete_nav(terminal)
            if self._cancel_reason is None and self._at_goal():
                self._node.get_logger().info(f'{self._ns}: at goal (pose), completing nav')
                return self._arrive()
        elif self._cancel_reason is None and self._nav_t0 is not None and self._at_goal():
            # Nav2 may report success before the action handle is accepted.
            elapsed = (now - self._nav_t0).nanoseconds * 1e-9
            if elapsed >= 1.0:
                self._node.get_logger().info(f'{self._ns}: at goal (pose), completing nav')
                return self._arrive()
        if self._result_fut is not None and self._result_fut.done():
            return self._complete_nav(int(self._result_fut.result().status))
        return None

    def _finish(self, ok: bool) -> str | None:
        self.done = self.active
        self.active = None
        self._clear_nav()
        self._nav_ok = ok
        self.phase = Phase.SAVING
        return None

    def _flush_save(self) -> str:
        had_recording = self._recording_active
        save_ok = True
        if had_recording:
            save_ok, _ = self._stop_recording()
        self.phase = Phase.IDLE
        if self._nav_ok and self._cfg.record.enabled and self._rec_cli:
            if not had_recording or not save_ok:
                return 'save_failed'
        return 'collected' if self._nav_ok else 'failed'
