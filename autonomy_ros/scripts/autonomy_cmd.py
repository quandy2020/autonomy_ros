#!/usr/bin/env python3
# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0
#
# CLI for autonomy_ros CommandInterface (/autonomy/* actions & services).
#
# Quick start (see docs/autonomy_cmd.md):
#   ros2 run autonomy_ros autonomy_cmd.py list-config
#   ros2 run autonomy_ros autonomy_cmd.py run-pose point_a --repeat 3
#   ros2 run autonomy_ros autonomy_cmd.py navigate-pose --x 1.0 --y 0.5 --feedback

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import uuid
from copy import deepcopy
from pathlib import Path
from typing import Any, Callable, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from autonomy_msgs.action import Dock, Follow, GuidedTour, NavigatePose, NavigateThrough, Teleop
from autonomy_msgs.msg import Error, ExhibitPoint, FollowTarget, TaskType, Waypoint
from autonomy_msgs.srv import (
    CancelTask,
    ContinueTour,
    GetTaskStatus,
    ListDocks,
    PauseTask,
    ResumeTask,
    SetInitialPose,
    SetTeleopMode,
    SkipToExhibit,
    TriggerEmergencyStop,
)
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, Quaternion
from std_msgs.msg import Header

_SCRIPT_DIR = Path(__file__).resolve().parent


# ---------------------------------------------------------------------------
# tasks.json
# ---------------------------------------------------------------------------

def load_task_config(path: str | Path) -> dict[str, Any]:
    p = Path(path).expanduser().resolve()
    with p.open(encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError("Task config root must be a JSON object")
    data["_config_path"] = str(p)
    data["_config_dir"] = str(p.parent)
    return data


class TaskCatalog:
    """Resolved view of tasks.json (poses, tasks, missions, defaults)."""

    def __init__(self, data: dict[str, Any]) -> None:
        self.data = data
        self.defaults: dict[str, Any] = dict(data.get("defaults", {}))
        self.poses: dict[str, Any] = dict(data.get("poses", {}))
        self.tasks: dict[str, Any] = dict(data.get("tasks", {}))
        self.missions: dict[str, Any] = dict(data.get("missions", {}))
        self.config_dir = Path(data.get("_config_dir", "."))

    @classmethod
    def load(cls, path: str | Path) -> TaskCatalog:
        return cls(load_task_config(path))

    def resolve_pose(self, name: str) -> dict[str, Any]:
        if name not in self.poses:
            raise KeyError(f"Unknown pose '{name}'. Available: {sorted(self.poses)}")
        raw = self.poses[name]
        if not isinstance(raw, dict):
            raise ValueError(f"Pose '{name}' must be an object")
        frame = str(raw.get("frame", self.defaults.get("frame", "odom")))
        return {
            "frame": frame,
            "x": float(raw.get("x", 0.0)),
            "y": float(raw.get("y", 0.0)),
            "z": float(raw.get("z", 0.0)),
            "yaw": float(raw.get("yaw", 0.0)),
        }

    def merge(self, spec: dict[str, Any]) -> dict[str, Any]:
        out = deepcopy(self.defaults)
        out.update(spec)
        return out

    def resolve_task_spec(self, name_or_spec: str | dict[str, Any]) -> dict[str, Any]:
        if isinstance(name_or_spec, str):
            if name_or_spec not in self.tasks:
                raise KeyError(
                    f"Unknown task '{name_or_spec}'. Available: {sorted(self.tasks)}"
                )
            spec = deepcopy(self.tasks[name_or_spec])
        else:
            spec = deepcopy(name_or_spec)
        return self.merge(spec)

    def expand_steps(self, spec: dict[str, Any]) -> list[dict[str, Any]]:
        """Expand repeat into atomic steps (re-sends whole action N times)."""
        spec = self.merge(spec)
        t = str(spec.get("type", "navigate_pose"))
        if t == "wait":
            return [{"type": "wait", "seconds": float(spec.get("seconds", 1.0))}]

        repeat = max(1, int(spec.get("repeat", 1)))
        pause = float(spec.get("pause_between", self.defaults.get("pause_between", 0.0)))
        steps: list[dict[str, Any]] = []
        for i in range(repeat):
            step = deepcopy(spec)
            step["repeat"] = 1
            step["_iteration"] = i
            step["_pause_after"] = pause if i < repeat - 1 else 0.0
            steps.append(step)
        return steps

    def mission_steps(self, mission_name: str) -> list[dict[str, Any]]:
        if mission_name not in self.missions:
            raise KeyError(
                f"Unknown mission '{mission_name}'. Available: {sorted(self.missions)}"
            )
        mission = self.missions[mission_name]
        sequence = mission.get("sequence", [])
        if not sequence:
            raise ValueError(f"Mission '{mission_name}' has empty sequence")
        steps: list[dict[str, Any]] = []
        for entry in sequence:
            steps.extend(self.expand_steps(self.resolve_task_spec(entry)))
        return steps

    def mission_stop_on_failure(self, mission_name: str, *, cli_default: bool = True) -> bool:
        mission = self.missions[mission_name]
        return bool(mission.get("stop_on_failure", cli_default))

    def list_summary(self) -> str:
        lines = [f"Config: {self.data.get('_config_path', '?')}", ""]
        lines.append("poses:")
        for name, p in sorted(self.poses.items()):
            lines.append(
                f"  {name}: ({p.get('x', 0)}, {p.get('y', 0)}) "
                f"yaw={p.get('yaw', 0)} frame={p.get('frame', self.defaults.get('frame', 'odom'))}"
            )
        lines.append("tasks:")
        for name, t in sorted(self.tasks.items()):
            desc = t.get("comment", "")
            lines.append(f"  {name}: type={t.get('type')} repeat={t.get('repeat', 1)} {desc}")
        lines.append("missions:")
        for name, m in sorted(self.missions.items()):
            seq = m.get("sequence", [])
            lines.append(f"  {name}: {len(seq)} step(s) — {m.get('description', '')}")
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# Goal builders
# ---------------------------------------------------------------------------

def resolve_interface_prefix(namespace: str) -> str:
    """ROS interface prefix; empty or 'autonomy' -> /autonomy/."""
    ns = (namespace or "").strip().strip("/")
    if not ns or ns == "autonomy":
        return "/autonomy/"
    return f"/{ns}/"


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def make_pose_stamped(frame_id: str, x: float, y: float, z: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header = Header(frame_id=frame_id)
    pose.pose.position = Point(x=x, y=y, z=z)
    pose.pose.orientation = yaw_to_quaternion(yaw)
    return pose


def make_duration(sec: float) -> Duration:
    d = Duration()
    whole = int(sec)
    d.sec = whole
    d.nanosec = int((sec - whole) * 1e9)
    return d


def load_goal_json(path: str | Path) -> dict[str, Any]:
    with Path(path).open(encoding="utf-8") as f:
        return json.load(f)


def resolve_goal_file(catalog: TaskCatalog, path: str) -> Path:
    p = Path(path)
    if not p.is_absolute():
        p = catalog.config_dir / p
    return p.resolve()


def parse_pose_stamped(
    pose_block: dict[str, Any],
    *,
    default_frame: str = "odom",
    extra: dict[str, Any] | None = None,
) -> PoseStamped:
    extra = extra or {}
    if "header" in pose_block or ("pose" in pose_block and "position" in pose_block.get("pose", {})):
        hdr = pose_block.get("header", {})
        inner = pose_block.get("pose", pose_block)
        pos = inner.get("position", {})
        ori = inner.get("orientation", {})
        frame = str(hdr.get("frame_id", extra.get("frame", default_frame)))
        x, y, z = float(pos.get("x", 0)), float(pos.get("y", 0)), float(pos.get("z", 0))
        if "w" in ori:
            ps = make_pose_stamped(frame, x, y, z, 0.0)
            ps.pose.orientation.w = float(ori["w"])
            ps.pose.orientation.x = float(ori.get("x", 0))
            ps.pose.orientation.y = float(ori.get("y", 0))
            ps.pose.orientation.z = float(ori.get("z", 0))
            return ps
        yaw = float(pose_block.get("yaw", extra.get("yaw", 0)))
        return make_pose_stamped(frame, x, y, z, yaw)
    frame = str(pose_block.get("frame", extra.get("frame", default_frame)))
    return make_pose_stamped(
        frame,
        float(pose_block.get("x", 0)),
        float(pose_block.get("y", 0)),
        float(pose_block.get("z", 0)),
        float(pose_block.get("yaw", extra.get("yaw", 0))),
    )


def waypoint_from_pose(
    pose_id: str,
    label: str,
    frame: str,
    x: float,
    y: float,
    z: float,
    yaw: float,
    *,
    wait_duration: float = 0.0,
) -> Waypoint:
    wp = Waypoint()
    wp.id = pose_id
    wp.label = label
    wp.pose = make_pose_stamped(frame, x, y, z, yaw)
    wp.wait_duration = wait_duration
    return wp


def waypoint_from_dict(d: dict[str, Any]) -> Waypoint:
    wp = Waypoint()
    wp.id = str(d.get("id", ""))
    wp.label = str(d.get("label", ""))
    wp.pose = parse_pose_stamped(d.get("pose", d), extra=d)
    wp.wait_duration = float(d.get("wait_duration", 0.0))
    wp.narration_id = str(d.get("narration_id", ""))
    wp.wait_for_continue = bool(d.get("wait_for_continue", False))
    return wp


def exhibit_from_dict(d: dict[str, Any]) -> ExhibitPoint:
    ex = ExhibitPoint()
    ex.exhibit_id = str(d.get("exhibit_id", ""))
    ex.exhibit_name = str(d.get("exhibit_name", ""))
    ex.narration_id = str(d.get("narration_id", ""))
    ex.pose = parse_pose_stamped(d.get("pose", d), extra=d)
    ex.dwell_duration = float(d.get("dwell_duration", 5.0))
    ex.wait_for_continue = bool(d.get("wait_for_continue", False))
    ex.xy_tolerance = float(d.get("xy_tolerance", 0.0))
    ex.yaw_tolerance = float(d.get("yaw_tolerance", 0.0))
    return ex


def build_navigate_pose_goal(
    *,
    task_id: str,
    frame: str,
    x: float,
    y: float,
    z: float,
    yaw: float,
    max_speed: float = 0.0,
) -> NavigatePose.Goal:
    goal = NavigatePose.Goal()
    goal.task_id = task_id
    goal.goal = make_pose_stamped(frame, x, y, z, yaw)
    goal.max_speed = max_speed
    return goal


def build_navigate_pose_from_spec(catalog: TaskCatalog, spec: dict[str, Any], task_id: str) -> NavigatePose.Goal:
    pose_name = spec.get("pose")
    if not pose_name:
        raise ValueError("navigate_pose requires 'pose' (named pose in config)")
    p = catalog.resolve_pose(str(pose_name))
    return build_navigate_pose_goal(
        task_id=task_id,
        frame=p["frame"],
        x=p["x"],
        y=p["y"],
        z=p["z"],
        yaw=p["yaw"],
        max_speed=float(spec.get("max_speed", 0.0)),
    )


def build_navigate_through_goal(
    *,
    task_id: str,
    waypoints: list[Waypoint],
    number_of_loops: int = 1,
    start_index: int = 0,
    stop_on_failure: bool = True,
    behavior_tree: str = "",
) -> NavigateThrough.Goal:
    goal = NavigateThrough.Goal()
    goal.task_id = task_id
    goal.waypoints = waypoints
    goal.number_of_loops = number_of_loops
    goal.start_index = start_index
    goal.stop_on_failure = stop_on_failure
    goal.behavior_tree = behavior_tree
    return goal


def build_navigate_through_from_spec(catalog: TaskCatalog, spec: dict[str, Any], task_id: str) -> NavigateThrough.Goal:
    pose_names = spec.get("poses", [])
    if not pose_names:
        raise ValueError("navigate_through requires 'poses': [name, ...]")
    wps = [
        waypoint_from_pose(str(name), str(name), **catalog.resolve_pose(str(name)))
        for name in pose_names
    ]
    return build_navigate_through_goal(
        task_id=task_id,
        waypoints=wps,
        number_of_loops=int(spec.get("number_of_loops", 1)),
        start_index=int(spec.get("start_index", 0)),
        stop_on_failure=bool(spec.get("stop_on_failure", True)),
    )


def build_navigate_through_from_json(
    data: dict[str, Any],
    task_id: str,
    *,
    loops_default: int = 1,
) -> NavigateThrough.Goal:
    wps = [waypoint_from_dict(w) for w in data.get("waypoints", [])]
    return build_navigate_through_goal(
        task_id=task_id,
        waypoints=wps,
        number_of_loops=int(data.get("number_of_loops", loops_default)),
        start_index=int(data.get("start_index", 0)),
        stop_on_failure=bool(data.get("stop_on_failure", True)),
        behavior_tree=str(data.get("behavior_tree", "")),
    )


def build_guided_tour_from_json(
    data: dict[str, Any],
    task_id: str,
    *,
    tour_id: str = "tour_cli",
    tour_name: str = "CLI tour",
) -> GuidedTour.Goal:
    goal = GuidedTour.Goal()
    goal.task_id = task_id
    goal.tour_id = str(data.get("tour_id", tour_id))
    goal.tour_name = str(data.get("tour_name", tour_name))
    goal.exhibits = [exhibit_from_dict(e) for e in data.get("exhibits", [])]
    goal.number_of_loops = int(data.get("number_of_loops", 1))
    goal.start_exhibit_index = int(data.get("start_exhibit_index", 0))
    goal.start_from_beginning = bool(data.get("start_from_beginning", True))
    goal.return_to_dock_on_complete = bool(data.get("return_to_dock_on_complete", False))
    goal.auto_dock_on_low_battery = bool(data.get("auto_dock_on_low_battery", False))
    goal.low_battery_threshold = float(data.get("low_battery_threshold", 20.0))
    goal.cruise_speed = float(data.get("cruise_speed", 0.0))
    return goal


def build_guided_tour_from_spec(catalog: TaskCatalog, spec: dict[str, Any], task_id: str) -> GuidedTour.Goal:
    gf = spec.get("goal_file")
    if not gf:
        raise ValueError("guided_tour requires 'goal_file'")
    data = load_goal_json(resolve_goal_file(catalog, str(gf)))
    goal = build_guided_tour_from_json(
        data,
        task_id,
        tour_id=str(spec.get("tour_id", data.get("tour_id", "tour_cli"))),
        tour_name=str(spec.get("tour_name", data.get("tour_name", "CLI tour"))),
    )
    if "cruise_speed" in spec:
        goal.cruise_speed = float(spec["cruise_speed"])
    elif "cruise_speed" in data:
        goal.cruise_speed = float(data["cruise_speed"])
    return goal


def build_dock_goal(
    task_id: str,
    *,
    dock_id: str = "dock_main",
    navigate_to_staging_pose: bool = True,
    max_staging_time: float = 300.0,
) -> Dock.Goal:
    goal = Dock.Goal()
    goal.task_id = task_id
    goal.use_dock_id = True
    goal.dock_id = dock_id
    goal.navigate_to_staging_pose = navigate_to_staging_pose
    goal.max_staging_time = max_staging_time
    return goal


def build_dock_from_spec(spec: dict[str, Any], task_id: str) -> Dock.Goal:
    return build_dock_goal(
        task_id,
        dock_id=str(spec.get("dock_id", "dock_main")),
        navigate_to_staging_pose=bool(spec.get("navigate_to_staging_pose", True)),
        max_staging_time=float(spec.get("max_staging_time", 300.0)),
    )


def build_follow_goal(
    task_id: str,
    *,
    frame: str,
    x: float,
    y: float,
    z: float,
    yaw: float,
    target_id: str,
    mode: str,
    follow_distance: float,
    max_speed: float,
    duration_sec: float,
) -> Follow.Goal:
    target = FollowTarget()
    if mode == "id" or target_id:
        target.use_target_id = True
        target.target_id = target_id
    else:
        target.use_target_pose = True
        target.target_pose = make_pose_stamped(frame, x, y, z, yaw)
    target.follow_distance = follow_distance
    target.max_linear_speed = max_speed
    target.target_frame = frame
    goal = Follow.Goal()
    goal.task_id = task_id
    goal.target = target
    goal.time_allowance = make_duration(duration_sec)
    return goal


def build_teleop_goal(
    task_id: str,
    *,
    duration_sec: float,
    preempt: bool,
    max_linear: float,
    max_angular: float,
) -> Teleop.Goal:
    goal = Teleop.Goal()
    goal.task_id = task_id
    goal.time_allowance = make_duration(duration_sec)
    goal.preempt_other_tasks = preempt
    goal.max_linear_speed = max_linear
    goal.max_angular_speed = max_angular
    return goal


def build_initial_pose_request(frame: str, x: float, y: float, z: float, yaw: float) -> SetInitialPose.Request:
    req = SetInitialPose.Request()
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = frame
    pose.pose.pose.position.x = x
    pose.pose.pose.position.y = y
    pose.pose.pose.position.z = z
    pose.pose.pose.orientation = yaw_to_quaternion(yaw)
    req.pose = pose
    return req


# ---------------------------------------------------------------------------
# ROS client & task runner
# ---------------------------------------------------------------------------

def default_task_id(prefix: str) -> str:
    return f"{prefix}_{uuid.uuid4().hex[:8]}"


def default_config_path() -> str:
    env = os.environ.get("AUTONOMY_TASKS_CONFIG")
    if env:
        return env
    return str(_SCRIPT_DIR / "examples" / "tasks.json")


def print_error(err: Error) -> None:
    print(f"error_code={err.error_code} msg={err.error_msg!r}")


def runner_options(args: argparse.Namespace, catalog: TaskCatalog) -> tuple[bool, float]:
    d = catalog.defaults
    feedback = bool(getattr(args, "feedback", False) or d.get("feedback", False))
    timeout = float(getattr(args, "timeout", 0.0) or d.get("timeout", 0.0) or 0.0)
    return feedback, timeout


class AutonomyCmd(Node):
    """ROS 2 client for /autonomy/* with cached action clients."""

    def __init__(self, namespace: str = "") -> None:
        super().__init__("autonomy_cmd")
        self._prefix = resolve_interface_prefix(namespace)
        self._action_clients: dict[str, ActionClient] = {}

    def _action_name(self, name: str) -> str:
        return f"{self._prefix}{name}"

    def _service_name(self, name: str) -> str:
        return f"{self._prefix}{name}"

    def action_client(self, action_type: type, name: str) -> ActionClient:
        if name not in self._action_clients:
            self._action_clients[name] = ActionClient(
                self, action_type, self._action_name(name)
            )
        return self._action_clients[name]

    def run_action(
        self,
        action_type: type,
        name: str,
        goal: Any,
        *,
        feedback: bool = False,
        timeout_sec: float = 0.0,
    ) -> int:
        client = self.action_client(action_type, name)
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"Action server not available: {self._action_name(name)}")
            return 1

        done = {"finished": False, "result": None}

        def _feedback_cb(fb_msg: Any) -> None:
            if feedback:
                print(f"[feedback] {fb_msg}")

        def _result_cb(future: Any) -> None:
            done["result"] = future.result().result
            done["finished"] = True

        send_future = client.send_goal_async(goal, feedback_callback=_feedback_cb)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=30.0)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            print("Goal rejected")
            return 1

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(_result_cb)

        deadline = time.monotonic() + (timeout_sec if timeout_sec > 0 else 3600.0)
        while rclpy.ok() and not done["finished"]:
            if time.monotonic() > deadline:
                print("Timeout waiting for action result")
                goal_handle.cancel_goal_async()
                return 1
            rclpy.spin_once(self, timeout_sec=0.1)

        result = done["result"]
        if hasattr(result, "error"):
            print_error(result.error)
        print(f"Result: {result}")
        return 0 if getattr(result, "error", Error()).error_code == 0 else 1

    def call_service(self, srv_type: type, name: str, request: Any, *, timeout_sec: float = 10.0) -> int:
        client = self.create_client(srv_type, self._service_name(name))
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(f"Service not available: {self._service_name(name)}")
            return 1
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        response = future.result()
        if response is None:
            print("Service call failed")
            return 1
        if hasattr(response, "error"):
            print_error(response.error)
            if response.error.error_code != 0:
                return 1
        if hasattr(response, "success"):
            print(f"success={response.success}")
            return 0 if response.success else 1
        if hasattr(response, "status"):
            print(f"status={response.status}")
        if hasattr(response, "docks"):
            for dock in response.docks:
                p = dock.dock_pose.pose.position
                print(f"  dock_id={dock.dock_id} pose=({p.x:.2f}, {p.y:.2f})")
        return 0


class TaskRunner:
    """Run expanded steps from TaskCatalog."""

    def __init__(
        self,
        node: AutonomyCmd,
        catalog: TaskCatalog,
        *,
        feedback: bool = False,
        timeout: float = 0.0,
    ) -> None:
        self.node = node
        self.catalog = catalog
        self.feedback = feedback
        self.timeout = timeout

    def _pause(self, seconds: float) -> None:
        if seconds <= 0:
            return
        print(f"[task] pause {seconds:.1f}s")
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def _run_goal(self, action_type: type, action_name: str, goal: Any, *, log: str) -> int:
        print(log)
        return self.node.run_action(
            action_type,
            action_name,
            goal,
            feedback=self.feedback,
            timeout_sec=self.timeout,
        )

    def execute_step(self, spec: dict[str, Any], *, label: str = "") -> int:
        t = str(spec.get("type", "navigate_pose"))
        if t == "wait":
            self._pause(float(spec.get("seconds", 1.0)))
            return 0

        task_id = str(spec.get("task_id") or default_task_id(t[:4]))
        tag = f" {label}" if label else ""

        if t == "navigate_pose":
            pose_name = spec.get("pose", "?")
            goal = build_navigate_pose_from_spec(self.catalog, spec, task_id)
            log = (
                f"[task] navigate_pose -> {pose_name}{tag} "
                f"iter={spec.get('_iteration', 0) + 1} id={task_id}"
            )
            rc = self._run_goal(NavigatePose, "navigate_pose", goal, log=log)
        elif t == "navigate_through":
            goal = build_navigate_through_from_spec(self.catalog, spec, task_id)
            log = f"[task] navigate_through {spec.get('poses')}{tag} id={task_id}"
            rc = self._run_goal(NavigateThrough, "navigate_through", goal, log=log)
        elif t == "guided_tour":
            goal = build_guided_tour_from_spec(self.catalog, spec, task_id)
            log = f"[task] guided_tour {spec.get('goal_file')}{tag} id={task_id}"
            rc = self._run_goal(GuidedTour, "guided_tour", goal, log=log)
        elif t == "dock":
            goal = build_dock_from_spec(spec, task_id)
            log = f"[task] dock {goal.dock_id}{tag} id={task_id}"
            rc = self._run_goal(Dock, "dock", goal, log=log)
        else:
            raise ValueError(f"Unsupported task type '{t}' in config")

        if rc == 0:
            self._pause(float(spec.get("_pause_after", 0.0)))
        return rc

    def run_steps(
        self,
        steps: list[dict[str, Any]],
        *,
        label_prefix: str = "",
        stop_on_failure: bool = True,
    ) -> int:
        for idx, step in enumerate(steps):
            label = label_prefix or f"step {idx + 1}/{len(steps)}"
            if label_prefix and len(steps) > 1:
                label = f"{label_prefix} [{idx + 1}/{len(steps)}]"
            rc = self.execute_step(step, label=label)
            if rc != 0:
                print(f"[task] failed at {label}")
                return rc if stop_on_failure else 0
        if label_prefix:
            print(f"[task] {label_prefix} completed")
        return 0

    def run_named_task(self, task_name: str) -> int:
        spec = self.catalog.resolve_task_spec(task_name)
        print(f"[task] === {task_name} (type={spec.get('type')}) ===")
        return self.run_steps(self.catalog.expand_steps(spec), label_prefix=task_name)

    def run_mission(self, mission_name: str, *, stop_on_failure: bool = True) -> int:
        stop = self.catalog.mission_stop_on_failure(mission_name, cli_default=stop_on_failure)
        mission = self.catalog.missions[mission_name]
        print(f"[task] === mission {mission_name}: {mission.get('description', '')} ===")
        return self.run_steps(
            self.catalog.mission_steps(mission_name),
            label_prefix=mission_name,
            stop_on_failure=stop,
        )

    def run_pose(self, pose_name: str, *, repeat: int = 1, pause_between: float = -1.0) -> int:
        spec: dict[str, Any] = {"type": "navigate_pose", "pose": pose_name, "repeat": repeat}
        if pause_between >= 0:
            spec["pause_between"] = pause_between
        print(f"[task] === pose {pose_name} x{repeat} ===")
        return self.run_steps(self.catalog.expand_steps(spec), label_prefix=pose_name)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def add_config_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--config",
        default="",
        help=f"tasks.json (default: {default_config_path()} or $AUTONOMY_TASKS_CONFIG)",
    )


def add_pose_args(parser: argparse.ArgumentParser, *, default_frame: str = "odom") -> None:
    parser.add_argument("--frame", default=default_frame, help="Pose header frame_id")
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0, help="Yaw radians")


def add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--task-id", default="", help="Task id (auto-generated if empty)")
    parser.add_argument(
        "--namespace",
        default="",
        help="Interface namespace (default: /autonomy/; also accepts 'autonomy')",
    )
    parser.add_argument("--feedback", action="store_true", help="Print action feedback")
    parser.add_argument("--timeout", type=float, default=0.0, help="Action wait timeout sec (0=3600)")


def add_namespace_arg(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--namespace",
        default="",
        help="Interface namespace (default: /autonomy/)",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="CLI for autonomy_ros CommandInterface",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s navigate-pose --x 1.0 --y 0.5 --feedback
  %(prog)s run-task goto_a_x3
  %(prog)s run-mission demo_patrol
""",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p_list = sub.add_parser("list-config", help="List poses, tasks, missions")
    add_config_args(p_list)

    for name, help_text, extra in (
        ("run-pose", "Navigate to named pose", ("pose_name",)),
        ("run-task", "Run named task", ("task_name",)),
        ("run-mission", "Run mission sequence", ("mission_name",)),
    ):
        p = sub.add_parser(name, help=help_text)
        add_config_args(p)
        p.add_argument(extra[0])
        if name == "run-pose":
            p.add_argument("--repeat", type=int, default=1)
            p.add_argument("--pause-between", type=float, default=-1.0)
        if name == "run-mission":
            p.add_argument("--continue-on-failure", action="store_true")
        add_common_args(p)

    p_nav = sub.add_parser("navigate-pose", help="NavigatePose action")
    add_common_args(p_nav)
    add_pose_args(p_nav)
    p_nav.add_argument("--max-speed", type=float, default=0.0)
    p_nav.add_argument("--goal-json", metavar="FILE", help="Optional JSON file (fields override CLI)")

    p_wp = sub.add_parser("navigate-through", help="NavigateThrough action")
    add_common_args(p_wp)
    p_wp.add_argument("--goal-json", required=True)
    p_wp.add_argument("--loops", type=int, default=1)

    p_tour = sub.add_parser("guided-tour", help="GuidedTour action")
    add_common_args(p_tour)
    p_tour.add_argument("--goal-json", required=True)
    p_tour.add_argument("--tour-id", default="tour_cli")
    p_tour.add_argument("--tour-name", default="CLI tour")

    p_follow = sub.add_parser("follow", help="Follow action")
    add_common_args(p_follow)
    p_follow.add_argument("--target-id", default="")
    p_follow.add_argument("--follow-distance", type=float, default=1.0)
    p_follow.add_argument("--max-speed", type=float, default=0.5)
    p_follow.add_argument("--duration", type=float, default=0.0)
    add_pose_args(p_follow)
    p_follow.add_argument("--mode", choices=("pose", "id"), default="pose")

    p_dock = sub.add_parser("dock", help="Dock action")
    add_common_args(p_dock)
    p_dock.add_argument("--dock-id", default="dock_main")
    p_dock.add_argument("--no-staging", action="store_true")
    p_dock.add_argument("--staging-timeout", type=float, default=300.0)

    p_teleop = sub.add_parser("teleop", help="Teleop action")
    add_common_args(p_teleop)
    p_teleop.add_argument("--duration", type=float, default=60.0)
    p_teleop.add_argument("--preempt", action="store_true", default=True)
    p_teleop.add_argument("--max-linear", type=float, default=0.5)
    p_teleop.add_argument("--max-angular", type=float, default=1.5)

    for svc_name, svc_help in (
        ("cancel", "CancelTask"),
        ("status", "GetTaskStatus"),
        ("pause", "PauseTask"),
        ("resume", "ResumeTask"),
        ("continue-tour", "ContinueTour"),
        ("skip-exhibit", "SkipToExhibit"),
        ("estop", "TriggerEmergencyStop"),
        ("list-docks", "ListDocks"),
    ):
        p = sub.add_parser(svc_name, help=svc_help)
        add_namespace_arg(p)
        if svc_name in ("cancel", "status", "pause", "resume", "continue-tour"):
            p.add_argument("--task-id", default="")
        if svc_name == "cancel":
            p.add_argument("--all", action="store_true", dest="cancel_all")
        if svc_name == "pause":
            p.add_argument("--reason", default="cli")
        if svc_name == "estop":
            p.add_argument("--release", action="store_true")
            p.add_argument("--reason", default="cli")
        if svc_name == "continue-tour":
            p.add_argument("--tour-id", default="")
        if svc_name == "skip-exhibit":
            p.add_argument("--exhibit-id", default="")
            p.add_argument("--exhibit-index", type=int, default=-1)

    p_init = sub.add_parser("set-initial-pose", help="SetInitialPose service")
    add_namespace_arg(p_init)
    add_pose_args(p_init)

    p_tmode = sub.add_parser("set-teleop-mode", help="SetTeleopMode service")
    add_namespace_arg(p_tmode)
    p_tmode.add_argument("--enable", action="store_true")
    p_tmode.add_argument("--disable", action="store_true")
    p_tmode.add_argument("--max-linear", type=float, default=0.5)
    p_tmode.add_argument("--max-angular", type=float, default=1.5)
    p_tmode.add_argument("--preempt", action="store_true")

    return parser


def _load_catalog(args: argparse.Namespace) -> TaskCatalog:
    path = getattr(args, "config", "") or default_config_path()
    return TaskCatalog.load(path)


def _task_runner(node: AutonomyCmd, args: argparse.Namespace) -> TaskRunner:
    catalog = _load_catalog(args)
    feedback, timeout = runner_options(args, catalog)
    return TaskRunner(node, catalog, feedback=feedback, timeout=timeout)


def cmd_list_config(_node: AutonomyCmd, args: argparse.Namespace) -> int:
    print(_load_catalog(args).list_summary())
    return 0


def cmd_run_pose(node: AutonomyCmd, args: argparse.Namespace) -> int:
    catalog = _load_catalog(args)
    feedback, timeout = runner_options(args, catalog)
    runner = TaskRunner(node, catalog, feedback=feedback, timeout=timeout)
    pause = float(args.pause_between)
    if pause < 0:
        pause = float(catalog.defaults.get("pause_between", 1.0))
    return runner.run_pose(args.pose_name, repeat=max(1, args.repeat), pause_between=pause)


def cmd_run_task(node: AutonomyCmd, args: argparse.Namespace) -> int:
    return _task_runner(node, args).run_named_task(args.task_name)


def cmd_run_mission(node: AutonomyCmd, args: argparse.Namespace) -> int:
    runner = _task_runner(node, args)
    return runner.run_mission(
        args.mission_name,
        stop_on_failure=not args.continue_on_failure,
    )


def cmd_navigate_pose(node: AutonomyCmd, args: argparse.Namespace) -> int:
    task_id = args.task_id or default_task_id("nav")
    goal = build_navigate_pose_goal(
        task_id=task_id,
        frame=args.frame,
        x=args.x,
        y=args.y,
        z=args.z,
        yaw=args.yaw,
        max_speed=float(args.max_speed),
    )
    if args.goal_json:
        data = load_goal_json(args.goal_json)
        if "max_speed" in data:
            goal.max_speed = float(data["max_speed"])
        if "goal" in data:
            goal.goal = parse_pose_stamped(data["goal"], default_frame=args.frame)
    return node.run_action(
        NavigatePose, "navigate_pose", goal, feedback=args.feedback, timeout_sec=args.timeout
    )


def cmd_navigate_through(node: AutonomyCmd, args: argparse.Namespace) -> int:
    task_id = args.task_id or default_task_id("wp")
    data = load_goal_json(args.goal_json)
    goal = build_navigate_through_from_json(data, task_id, loops_default=args.loops)
    return node.run_action(
        NavigateThrough, "navigate_through", goal, feedback=args.feedback, timeout_sec=args.timeout
    )


def cmd_guided_tour(node: AutonomyCmd, args: argparse.Namespace) -> int:
    task_id = args.task_id or default_task_id("tour")
    data = load_goal_json(args.goal_json)
    goal = build_guided_tour_from_json(
        data, task_id, tour_id=args.tour_id, tour_name=args.tour_name
    )
    return node.run_action(
        GuidedTour, "guided_tour", goal, feedback=args.feedback, timeout_sec=args.timeout
    )


def cmd_follow(node: AutonomyCmd, args: argparse.Namespace) -> int:
    task_id = args.task_id or default_task_id("follow")
    goal = build_follow_goal(
        task_id,
        frame=args.frame,
        x=args.x,
        y=args.y,
        z=args.z,
        yaw=args.yaw,
        target_id=args.target_id,
        mode=args.mode,
        follow_distance=float(args.follow_distance),
        max_speed=float(args.max_speed),
        duration_sec=float(args.duration),
    )
    return node.run_action(Follow, "follow", goal, feedback=args.feedback, timeout_sec=args.timeout)


def cmd_dock(node: AutonomyCmd, args: argparse.Namespace) -> int:
    task_id = args.task_id or default_task_id("dock")
    goal = build_dock_goal(
        task_id,
        dock_id=args.dock_id,
        navigate_to_staging_pose=not args.no_staging,
        max_staging_time=float(args.staging_timeout),
    )
    return node.run_action(Dock, "dock", goal, feedback=args.feedback, timeout_sec=args.timeout)


def cmd_teleop(node: AutonomyCmd, args: argparse.Namespace) -> int:
    task_id = args.task_id or default_task_id("teleop")
    goal = build_teleop_goal(
        task_id,
        duration_sec=float(args.duration),
        preempt=bool(args.preempt),
        max_linear=float(args.max_linear),
        max_angular=float(args.max_angular),
    )
    return node.run_action(Teleop, "teleop", goal, feedback=args.feedback, timeout_sec=args.timeout)


def cmd_cancel(node: AutonomyCmd, args: argparse.Namespace) -> int:
    req = CancelTask.Request()
    req.task_id = args.task_id
    req.cancel_all = bool(args.cancel_all)
    req.task_type = TaskType()
    return node.call_service(CancelTask, "cancel_task", req)


def cmd_status(node: AutonomyCmd, args: argparse.Namespace) -> int:
    req = GetTaskStatus.Request()
    req.task_id = args.task_id
    return node.call_service(GetTaskStatus, "get_task_status", req)


def cmd_pause(node: AutonomyCmd, args: argparse.Namespace) -> int:
    req = PauseTask.Request()
    req.task_id = args.task_id
    req.reason = args.reason
    return node.call_service(PauseTask, "pause_task", req)


def cmd_resume(node: AutonomyCmd, args: argparse.Namespace) -> int:
    req = ResumeTask.Request()
    req.task_id = args.task_id
    return node.call_service(ResumeTask, "resume_task", req)


def cmd_continue_tour(node: AutonomyCmd, args: argparse.Namespace) -> int:
    req = ContinueTour.Request()
    req.tour_id = args.tour_id
    req.task_id = args.task_id
    return node.call_service(ContinueTour, "continue_tour", req)


def cmd_skip_exhibit(node: AutonomyCmd, args: argparse.Namespace) -> int:
    req = SkipToExhibit.Request()
    req.exhibit_id = args.exhibit_id
    req.exhibit_index = int(args.exhibit_index)
    return node.call_service(SkipToExhibit, "skip_to_exhibit", req)


def cmd_estop(node: AutonomyCmd, args: argparse.Namespace) -> int:
    req = TriggerEmergencyStop.Request()
    req.engage = not args.release
    req.reason = args.reason
    return node.call_service(TriggerEmergencyStop, "trigger_estop", req)


def cmd_set_initial_pose(node: AutonomyCmd, args: argparse.Namespace) -> int:
    req = build_initial_pose_request(args.frame, args.x, args.y, args.z, args.yaw)
    return node.call_service(SetInitialPose, "set_initial_pose", req)


def cmd_set_teleop_mode(node: AutonomyCmd, args: argparse.Namespace) -> int:
    req = SetTeleopMode.Request()
    req.enable = args.enable or not args.disable
    req.max_linear_speed = float(args.max_linear)
    req.max_angular_speed = float(args.max_angular)
    req.preempt_other_tasks = bool(args.preempt)
    return node.call_service(SetTeleopMode, "set_teleop_mode", req)


def cmd_list_docks(node: AutonomyCmd, args: argparse.Namespace) -> int:
    return node.call_service(ListDocks, "list_docks", ListDocks.Request())


_COMMANDS: dict[str, Callable[[AutonomyCmd, argparse.Namespace], int]] = {
    "list-config": cmd_list_config,
    "run-pose": cmd_run_pose,
    "run-task": cmd_run_task,
    "run-mission": cmd_run_mission,
    "navigate-pose": cmd_navigate_pose,
    "navigate-through": cmd_navigate_through,
    "guided-tour": cmd_guided_tour,
    "follow": cmd_follow,
    "dock": cmd_dock,
    "teleop": cmd_teleop,
    "cancel": cmd_cancel,
    "status": cmd_status,
    "pause": cmd_pause,
    "resume": cmd_resume,
    "continue-tour": cmd_continue_tour,
    "skip-exhibit": cmd_skip_exhibit,
    "estop": cmd_estop,
    "set-initial-pose": cmd_set_initial_pose,
    "set-teleop-mode": cmd_set_teleop_mode,
    "list-docks": cmd_list_docks,
}


def run_command(node: AutonomyCmd, args: argparse.Namespace) -> int:
    handler = _COMMANDS.get(args.command)
    if handler is None:
        print(f"Unknown command: {args.command}", file=sys.stderr)
        return 1
    return handler(node, args)


def main(argv: Optional[list[str]] = None) -> int:
    argv = argv if argv is not None else sys.argv[1:]
    if "--" in argv:
        argv = argv[argv.index("--") + 1 :]
    args = build_parser().parse_args(argv)

    rclpy.init(args=None)
    node = AutonomyCmd(namespace=getattr(args, "namespace", "") or "")
    try:
        return run_command(node, args)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
