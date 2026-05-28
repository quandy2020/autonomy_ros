#!/usr/bin/env python3
# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0
"""Navigation CLI for autonomy_ros actions and services."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import uuid
from pathlib import Path
from typing import Any

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from autonomy_msgs.action import NavigatePose, NavigateThrough
from autonomy_msgs.msg import Error, Waypoint
from autonomy_msgs.srv import (
    CancelTask,
    GetTaskStatus,
    PauseTask,
    ResumeTask,
    SetInitialPose,
    TriggerEmergencyStop,
)
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, Quaternion
from std_msgs.msg import Header

def make_task_id(prefix: str) -> str:
    return f'{prefix}_{uuid.uuid4().hex[:8]}'


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


def load_json(path: str | Path) -> dict[str, Any]:
    with Path(path).expanduser().resolve().open(encoding='utf-8') as file:
        return json.load(file)


def default_tasks_file() -> str:
    env = os.environ.get('AUTONOMY_NAVIGATION_TASKS')
    if env:
        return env
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('autonomy_ros')
        return os.path.join(share, 'config', 'navigation_tasks.json')
    except Exception:
        return str(Path(__file__).resolve().parent.parent / 'config' / 'navigation_tasks.json')


def resolve_pose(tasks: dict[str, Any], name: str) -> tuple[str, float, float, float, float]:
    poses = tasks.get('poses', {})
    if name not in poses:
        raise KeyError(f"Unknown pose '{name}'. Available: {sorted(poses)}")
    pose = poses[name]
    frame = str(pose.get('frame', tasks.get('frame', 'odom')))
    return (
        frame,
        float(pose.get('x', 0.0)),
        float(pose.get('y', 0.0)),
        float(pose.get('z', 0.0)),
        float(pose.get('yaw', 0.0)),
    )


def build_navigate_pose_goal(task_id: str, frame: str, x: float, y: float, z: float, yaw: float) -> NavigatePose.Goal:
    goal = NavigatePose.Goal()
    goal.task_id = task_id
    goal.goal = make_pose_stamped(frame, x, y, z, yaw)
    return goal


def build_navigate_through_goal(data: dict[str, Any], task_id: str) -> NavigateThrough.Goal:
    goal = NavigateThrough.Goal()
    goal.task_id = task_id
    goal.waypoints = []
    for entry in data.get('waypoints', []):
        wp = Waypoint()
        wp.id = str(entry.get('id', ''))
        wp.label = str(entry.get('label', ''))
        wp.wait_duration = float(entry.get('wait_duration', 0.0))
        pose_block = entry.get('pose', entry)
        header = pose_block.get('header', {})
        inner = pose_block.get('pose', pose_block)
        pos = inner.get('position', {})
        ori = inner.get('orientation', {})
        frame = str(header.get('frame_id', data.get('frame', 'odom')))
        wp.pose = make_pose_stamped(
            frame,
            float(pos.get('x', 0.0)),
            float(pos.get('y', 0.0)),
            float(pos.get('z', 0.0)),
            0.0,
        )
        if 'w' in ori:
            wp.pose.pose.orientation.w = float(ori['w'])
            wp.pose.pose.orientation.x = float(ori.get('x', 0.0))
            wp.pose.pose.orientation.y = float(ori.get('y', 0.0))
            wp.pose.pose.orientation.z = float(ori.get('z', 0.0))
        goal.waypoints.append(wp)
    goal.number_of_loops = int(data.get('number_of_loops', 1))
    goal.start_index = int(data.get('start_index', 0))
    goal.stop_on_failure = bool(data.get('stop_on_failure', True))
    return goal


class NavigationClient(Node):
    def __init__(self, namespace: str = '') -> None:
        super().__init__('navigation_client')
        ns = (namespace or '').strip().strip('/')
        self._prefix = '/' if not ns else f'/{ns}/'
        self._action_clients: dict[str, ActionClient] = {}

    def _action_name(self, name: str) -> str:
        return f'{self._prefix}{name}'

    def _service_name(self, name: str) -> str:
        return f'{self._prefix}{name}'

    def run_action(self, action_type: type, name: str, goal: Any, *, feedback: bool, timeout_sec: float) -> int:
        if name not in self._action_clients:
            self._action_clients[name] = ActionClient(self, action_type, self._action_name(name))
        client = self._action_clients[name]
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f'Action not available: {self._action_name(name)}')
            return 1

        done = {'finished': False, 'result': None}

        def on_result(future: Any) -> None:
            done['result'] = future.result().result
            done['finished'] = True

        send_future = client.send_goal_async(
            goal,
            feedback_callback=(lambda msg: print(f'[feedback] {msg}')) if feedback else None,
        )
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=30.0)
        handle = send_future.result()
        if not handle.accepted:
            print('Goal rejected')
            return 1

        result_future = handle.get_result_async()
        result_future.add_done_callback(on_result)
        deadline = time.monotonic() + (timeout_sec if timeout_sec > 0 else 3600.0)
        while rclpy.ok() and not done['finished']:
            if time.monotonic() > deadline:
                handle.cancel_goal_async()
                print('Timeout')
                return 1
            rclpy.spin_once(self, timeout_sec=0.1)

        result = done['result']
        if hasattr(result, 'error'):
            print(f'error_code={result.error.error_code} msg={result.error.error_msg!r}')
        return 0 if getattr(result, 'error', Error()).error_code == 0 else 1

    def call_service(self, service_type: type, name: str, request: Any) -> int:
        client = self.create_client(service_type, self._service_name(name))
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f'Service not available: {self._service_name(name)}')
            return 1
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        response = future.result()
        if response is None:
            return 1
        if hasattr(response, 'error') and response.error.error_code != 0:
            print(f'error_code={response.error.error_code} msg={response.error.error_msg!r}')
            return 1
        if hasattr(response, 'success'):
            print(f'success={response.success}')
            return 0 if response.success else 1
        return 0


def run_task_from_config(client: NavigationClient, tasks_file: str, task_name: str, *, feedback: bool, timeout: float) -> int:
    tasks = load_json(tasks_file)
    catalog = tasks.get('tasks', {})
    if task_name not in catalog:
        raise KeyError(f"Unknown task '{task_name}'. Available: {sorted(catalog)}")
    spec = catalog[task_name]
    task_type = spec.get('type', 'navigate_pose')
    task_id = make_task_id('nav')

    if task_type == 'navigate_pose':
        frame, x, y, z, yaw = resolve_pose(tasks, str(spec['pose']))
        goal = build_navigate_pose_goal(task_id, frame, x, y, z, yaw)
        return client.run_action(NavigatePose, 'navigate_pose', goal, feedback=feedback, timeout_sec=timeout)

    if task_type == 'navigate_through':
        waypoints = []
        for pose_name in spec.get('poses', []):
            frame, x, y, z, yaw = resolve_pose(tasks, str(pose_name))
            wp = Waypoint()
            wp.id = str(pose_name)
            wp.label = str(pose_name)
            wp.pose = make_pose_stamped(frame, x, y, z, yaw)
            waypoints.append(wp)
        goal = NavigateThrough.Goal()
        goal.task_id = task_id
        goal.waypoints = waypoints
        goal.number_of_loops = int(spec.get('number_of_loops', 1))
        goal.stop_on_failure = bool(spec.get('stop_on_failure', True))
        return client.run_action(NavigateThrough, 'navigate_through', goal, feedback=feedback, timeout_sec=timeout)

    raise ValueError(f"Unsupported task type '{task_type}'")


def print_tasks_summary(tasks_file: str) -> None:
    tasks = load_json(tasks_file)
    print(f'File: {tasks_file}')
    print(f"frame: {tasks.get('frame', 'odom')}")
    print('poses:')
    for name, pose in sorted(tasks.get('poses', {}).items()):
        print(f"  {name}: ({pose.get('x', 0)}, {pose.get('y', 0)}) yaw={pose.get('yaw', 0)}")
    print('tasks:')
    for name, spec in sorted(tasks.get('tasks', {}).items()):
        print(f"  {name}: type={spec.get('type')}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Navigation client for autonomy_ros')
    parser.add_argument('--namespace', default='', help='Optional ROS namespace prefix (default: root /)')
    parser.add_argument('--feedback', action='store_true', help='Print action feedback')
    parser.add_argument('--timeout', type=float, default=0.0, help='Action timeout seconds (0 = 3600)')

    sub = parser.add_subparsers(dest='command', required=True)

    list_tasks = sub.add_parser('list-tasks', help='List poses and named tasks')
    list_tasks.add_argument('--tasks-file', default='', help='navigation_tasks.json path')

    run_task = sub.add_parser('run-task', help='Run a named task from navigation_tasks.json')
    run_task.add_argument('task_name')
    run_task.add_argument('--tasks-file', default='')

    navigate_pose = sub.add_parser('navigate-pose', help='Send NavigatePose goal')
    navigate_pose.add_argument('--task-id', default='')
    navigate_pose.add_argument('--frame', default='odom')
    navigate_pose.add_argument('--x', type=float, default=0.0)
    navigate_pose.add_argument('--y', type=float, default=0.0)
    navigate_pose.add_argument('--z', type=float, default=0.0)
    navigate_pose.add_argument('--yaw', type=float, default=0.0)

    navigate_through = sub.add_parser('navigate-through', help='Send NavigateThrough goal from JSON')
    navigate_through.add_argument('--goal-file', required=True)
    navigate_through.add_argument('--task-id', default='')

    cancel = sub.add_parser('cancel')
    cancel.add_argument('--task-id', default='')
    cancel.add_argument('--all', action='store_true', dest='cancel_all')

    status = sub.add_parser('status')
    status.add_argument('--task-id', default='')

    pause = sub.add_parser('pause')
    pause.add_argument('--task-id', default='')
    pause.add_argument('--reason', default='manual')

    resume = sub.add_parser('resume')
    resume.add_argument('--task-id', default='')

    estop = sub.add_parser('estop')
    estop.add_argument('--release', action='store_true')
    estop.add_argument('--reason', default='manual')

    initial_pose = sub.add_parser('set-initial-pose')
    initial_pose.add_argument('--frame', default='odom')
    initial_pose.add_argument('--x', type=float, default=0.0)
    initial_pose.add_argument('--y', type=float, default=0.0)
    initial_pose.add_argument('--z', type=float, default=0.0)
    initial_pose.add_argument('--yaw', type=float, default=0.0)

    return parser


def main() -> int:
    args = build_parser().parse_args()
    tasks_file = getattr(args, 'tasks_file', '') or default_tasks_file()

    rclpy.init()
    client = NavigationClient(namespace=args.namespace)
    try:
        if args.command == 'list-tasks':
            print_tasks_summary(tasks_file)
            return 0
        if args.command == 'run-task':
            return run_task_from_config(
                client, tasks_file, args.task_name, feedback=args.feedback, timeout=args.timeout)
        if args.command == 'navigate-pose':
            task_id = args.task_id or make_task_id('nav')
            goal = build_navigate_pose_goal(task_id, args.frame, args.x, args.y, args.z, args.yaw)
            return client.run_action(
                NavigatePose, 'navigate_pose', goal, feedback=args.feedback, timeout_sec=args.timeout)
        if args.command == 'navigate-through':
            task_id = args.task_id or make_task_id('through')
            goal = build_navigate_through_goal(load_json(args.goal_file), task_id)
            return client.run_action(
                NavigateThrough, 'navigate_through', goal, feedback=args.feedback, timeout_sec=args.timeout)
        if args.command == 'cancel':
            request = CancelTask.Request()
            request.task_id = args.task_id
            request.cancel_all = getattr(args, 'cancel_all', False)
            return client.call_service(CancelTask, 'cancel_task', request)
        if args.command == 'status':
            request = GetTaskStatus.Request()
            request.task_id = args.task_id
            return client.call_service(GetTaskStatus, 'get_task_status', request)
        if args.command == 'pause':
            request = PauseTask.Request()
            request.task_id = args.task_id
            request.reason = args.reason
            return client.call_service(PauseTask, 'pause_task', request)
        if args.command == 'resume':
            request = ResumeTask.Request()
            request.task_id = args.task_id
            return client.call_service(ResumeTask, 'resume_task', request)
        if args.command == 'estop':
            request = TriggerEmergencyStop.Request()
            request.release = args.release
            request.reason = args.reason
            return client.call_service(TriggerEmergencyStop, 'trigger_estop', request)
        if args.command == 'set-initial-pose':
            request = SetInitialPose.Request()
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = args.frame
            pose.pose.pose.position.x = args.x
            pose.pose.pose.position.y = args.y
            pose.pose.pose.position.z = args.z
            pose.pose.pose.orientation = yaw_to_quaternion(args.yaw)
            request.pose = pose
            return client.call_service(SetInitialPose, 'set_initial_pose', request)
    finally:
        client.destroy_node()
        rclpy.shutdown()
    return 1


if __name__ == '__main__':
    sys.exit(main())
