#!/usr/bin/env python3
# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Send NavigateToPose goals to all namespaced robots at once.

Works with ``navigation_nav2_multi.launch.py`` (robot1 … robotN).

Examples::

  # 10 robots, goals on a circle (radius 3 m)
  ros2 run autonomy_ros multi_robot_navigate_to_pose.py

  # Same target for every robot
  ros2 run autonomy_ros multi_robot_navigate_to_pose.py --x 2.0 --y 1.0 --yaw 0.0

  # Per-robot goals from JSON
  ros2 run autonomy_ros multi_robot_navigate_to_pose.py --goals-json goals.json
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _make_pose(
    frame_id: str,
    x: float,
    y: float,
    yaw: float,
    stamp,
) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation = _yaw_to_quaternion(yaw)
    return pose


def _robot_names(prefix: str, num_robots: int) -> list[str]:
    return [f'{prefix}{index}' for index in range(1, num_robots + 1)]


def _circle_goals(
    num_robots: int,
    center_x: float,
    center_y: float,
    radius: float,
    yaw: float,
) -> list[tuple[float, float, float]]:
    if num_robots == 1:
        return [(center_x + radius, center_y, yaw)]
    goals: list[tuple[float, float, float]] = []
    for index in range(num_robots):
        angle = 2.0 * math.pi * index / num_robots
        goals.append((
            center_x + radius * math.cos(angle),
            center_y + radius * math.sin(angle),
            yaw + angle,
        ))
    return goals


def _load_goals_json(
    path: str,
    names: list[str],
) -> list[tuple[float, float, float]]:
    data: dict[str, Any] = json.loads(open(path, encoding='utf-8').read())
    goals: list[tuple[float, float, float]] = []
    for name in names:
        if name not in data:
            raise KeyError(f'missing goal for {name!r} in {path}')
        entry = data[name]
        goals.append((
            float(entry['x']),
            float(entry['y']),
            float(entry.get('yaw', 0.0)),
        ))
    return goals


class MultiRobotNavigateToPose(Node):
    """Action clients for /robotK/navigate_to_pose."""

    def __init__(
        self,
        robot_names: list[str],
        goals: list[tuple[float, float, float]],
        frame_id: str,
        server_timeout_sec: float,
        result_timeout_sec: float,
        wait_for_result: bool,
    ) -> None:
        super().__init__('multi_robot_navigate_to_pose')
        self._frame_id = frame_id
        self._result_timeout_sec = result_timeout_sec
        self._wait_for_result = wait_for_result
        self._robot_action_clients: list[
            tuple[str, ActionClient, tuple[float, float, float]]
        ] = []

        for name, goal in zip(robot_names, goals):
            action_name = f'/{name}/navigate_to_pose'
            client = ActionClient(self, NavigateToPose, action_name)
            self._robot_action_clients.append((name, client, goal))
            self.get_logger().info(
                f'{name}: target ({goal[0]:.2f}, {goal[1]:.2f}, yaw={goal[2]:.2f}) '
                f'via {action_name}')

        if not self._wait_for_servers(server_timeout_sec):
            raise RuntimeError('Not all navigate_to_pose action servers are available')

    def run(self) -> int:
        executor = MultiThreadedExecutor()
        executor.add_node(self)
        try:
            return self._send_all(executor)
        finally:
            executor.remove_node(self)

    def _wait_for_servers(self, timeout_sec: float) -> bool:
        deadline = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        for name, client, _ in self._robot_action_clients:
            remaining = max(0.0, (deadline - self.get_clock().now().nanoseconds) / 1e9)
            if not client.wait_for_server(timeout_sec=remaining):
                self.get_logger().error(f'{name}: action server not available')
                return False
        return True

    @staticmethod
    def _spin_until_all_done(
        executor,
        futures: list[tuple[str, Any]],
        timeout_sec: float,
    ) -> list[tuple[str, Any]]:
        """Spin until every future completes or the timeout elapses."""
        deadline = time.monotonic() + timeout_sec
        remaining = list(futures)
        while rclpy.ok() and remaining and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
            remaining = [(name, future) for name, future in remaining if not future.done()]
        return remaining

    def _send_all(self, executor) -> int:
        stamp = self.get_clock().now().to_msg()
        send_futures: list[tuple[str, Any]] = []

        for name, client, (x, y, yaw) in self._robot_action_clients:
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = _make_pose(self._frame_id, x, y, yaw, stamp)
            send_futures.append((name, client.send_goal_async(goal_msg)))

        send_timeout = 10.0 * max(1, len(send_futures))
        timed_out = self._spin_until_all_done(executor, send_futures, send_timeout)
        timed_out_names = {name for name, _ in timed_out}
        for name, future in send_futures:
            if name in timed_out_names:
                self.get_logger().error(f'{name}: goal send timeout')
                continue
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f'{name}: goal rejected')
                continue
            self.get_logger().info(f'{name}: goal accepted')

        if not self._wait_for_result:
            self.get_logger().info('All goals dispatched')
            return 0

        result_futures: list[tuple[str, Any]] = []
        for name, future in send_futures:
            if name in timed_out_names:
                continue
            goal_handle = future.result()
            if goal_handle.accepted:
                result_futures.append((name, goal_handle.get_result_async()))

        if not result_futures:
            return 0

        result_timeout = self._result_timeout_sec
        timed_out = self._spin_until_all_done(executor, result_futures, result_timeout)
        timed_out_names = {name for name, _ in timed_out}
        for name, future in result_futures:
            if name in timed_out_names:
                self.get_logger().warn(f'{name}: result timeout')
                continue
            status = future.result().status
            if status == 4:  # SUCCEEDED
                self.get_logger().info(f'{name}: navigation succeeded')
            else:
                self.get_logger().warn(
                    f'{name}: navigation finished with status {status}')
        return 0


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Send Nav2 goals to all namespaced robots simultaneously')
    parser.add_argument('--num-robots', type=int, default=10)
    parser.add_argument('--robot-prefix', default='robot')
    parser.add_argument('--frame-id', default='map')
    parser.add_argument('--x', type=float, default=None, help='Same goal X for all robots')
    parser.add_argument('--y', type=float, default=None, help='Same goal Y for all robots')
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument(
        '--spread-radius', type=float, default=3.0,
        help='Place goals on a circle (default when --x/--y omitted)')
    parser.add_argument('--center-x', type=float, default=0.0)
    parser.add_argument('--center-y', type=float, default=0.0)
    parser.add_argument(
        '--goals-json',
        default='',
        help='JSON file: {"robot1": {"x": 1, "y": 2, "yaw": 0}, ...}',
    )
    parser.add_argument(
        '--server-timeout', type=float, default=30.0,
        help='Seconds to wait for each action server')
    parser.add_argument(
        '--result-timeout', type=float, default=300.0,
        help='Seconds to wait for each navigation result')
    parser.add_argument(
        '--no-wait', action='store_true',
        help='Send goals and exit without waiting for results')
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    if args.num_robots < 1:
        print('num_robots must be >= 1', file=sys.stderr)
        return 1

    names = _robot_names(args.robot_prefix, args.num_robots)

    if args.goals_json:
        goals = _load_goals_json(args.goals_json, names)
    elif args.x is not None and args.y is not None:
        goals = [(args.x, args.y, args.yaw)] * args.num_robots
    else:
        goals = _circle_goals(
            args.num_robots, args.center_x, args.center_y,
            args.spread_radius, args.yaw)

    rclpy.init()
    node = MultiRobotNavigateToPose(
        robot_names=names,
        goals=goals,
        frame_id=args.frame_id,
        server_timeout_sec=args.server_timeout,
        result_timeout_sec=args.result_timeout,
        wait_for_result=not args.no_wait,
    )
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
