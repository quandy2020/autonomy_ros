#!/usr/bin/env python3
# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0

"""Drive hunav_agent_manager via compute_agents using a HuNav scenario YAML."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Dict, List, Tuple

import rclpy
import yaml
from geometry_msgs.msg import Pose
from hunav_msgs.msg import Agent, AgentBehavior, Agents
from hunav_msgs.srv import ComputeAgents
from rclpy.node import Node


def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _pose_from_xy(x: float, y: float, yaw: float = 0.0) -> Pose:
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0.0
    qx, qy, qz, qw = _quaternion_from_yaw(yaw)
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


BEHAVIOR_MAP = {
    'Regular': AgentBehavior.BEH_REGULAR,
    'Impassive': AgentBehavior.BEH_IMPASSIVE,
    'Surprised': AgentBehavior.BEH_SURPRISED,
    'Scared': AgentBehavior.BEH_SCARED,
    'Curious': AgentBehavior.BEH_CURIOUS,
    'Threatening': AgentBehavior.BEH_THREATENING,
}

CONF_MAP = {
    0: AgentBehavior.BEH_CONF_DEFAULT,
    1: AgentBehavior.BEH_CONF_CUSTOM,
    2: AgentBehavior.BEH_CONF_RANDOM_NORMAL,
    3: AgentBehavior.BEH_CONF_RANDOM_UNIFORM,
}


def _load_params(yaml_path: Path) -> Dict[str, Any]:
    with yaml_path.open(encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    return data['hunav_loader']['ros__parameters']


def _global_goals(params: Dict[str, Any]) -> Dict[int, Tuple[float, float]]:
    raw = params.get('global_goals') or {}
    return {int(k): (float(v['x']), float(v['y'])) for k, v in raw.items()}


def _build_behavior(raw: Dict[str, Any]) -> AgentBehavior:
    beh = AgentBehavior()
    beh.type = BEHAVIOR_MAP.get(str(raw.get('type', 'Regular')), AgentBehavior.BEH_REGULAR)
    beh.configuration = CONF_MAP.get(int(raw.get('configuration', 0)), AgentBehavior.BEH_CONF_DEFAULT)
    beh.goal_force_factor = float(raw.get('goal_force_factor', 2.0))
    beh.obstacle_force_factor = float(raw.get('obstacle_force_factor', 10.0))
    beh.social_force_factor = float(raw.get('social_force_factor', 5.0))
    beh.other_force_factor = float(raw.get('other_force_factor', 20.0))
    beh.duration = float(raw.get('duration', 0.0))
    beh.once = bool(raw.get('once', False))
    beh.vel = float(raw.get('vel', 0.0))
    beh.dist = float(raw.get('dist', 0.0))
    return beh


def _build_agent(name: str, raw: Dict[str, Any], goals: Dict[int, Tuple[float, float]]) -> Agent:
    init_pose = raw.get('init_pose') or {}
    yaw = float(init_pose.get('h', 0.0))
    agent = Agent()
    agent.id = int(raw.get('id', 0))
    agent.type = Agent.PERSON
    agent.name = name
    agent.skin = int(raw.get('skin', -1))
    agent.group_id = int(raw.get('group_id', -1))
    agent.position = _pose_from_xy(float(init_pose.get('x', 0.0)), float(init_pose.get('y', 0.0)), yaw)
    agent.position.position.z = float(init_pose.get('z', 0.0))
    agent.yaw = yaw
    agent.desired_velocity = float(raw.get('max_vel', 1.5))
    agent.radius = float(raw.get('radius', 0.4))
    agent.goal_radius = float(raw.get('goal_radius', 0.3))
    agent.cyclic_goals = bool(raw.get('cyclic_goals', True))
    agent.behavior = _build_behavior(raw.get('behavior') or {})

    for gid in raw.get('goals') or []:
        gx, gy = goals[int(gid)]
        agent.goals.append(_pose_from_xy(gx, gy))

    agent.linear_vel = 0.0
    agent.angular_vel = 0.0
    return agent


class HunavSimDriver(Node):
    def __init__(self) -> None:
        super().__init__('hunav_sim_driver')
        self.declare_parameter('scenario_yaml', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('update_hz', 10.0)
        self.declare_parameter('robot_x', 0.0)
        self.declare_parameter('robot_y', 0.0)
        self.declare_parameter('robot_yaw', 0.0)

        scenario = self.get_parameter('scenario_yaml').value
        if not scenario:
            raise RuntimeError('scenario_yaml parameter is required')
        self._frame_id = self.get_parameter('frame_id').value
        hz = float(self.get_parameter('update_hz').value)

        params = _load_params(Path(scenario).expanduser().resolve())
        goals = _global_goals(params)
        self._agents = Agents()
        self._agents.header.frame_id = params.get('map', self._frame_id)
        for name in params.get('agents') or []:
            self._agents.agents.append(_build_agent(name, params[name], goals))

        self._robot = self._build_robot()
        self._client = self.create_client(ComputeAgents, 'compute_agents')
        self._timer = self.create_timer(1.0 / max(hz, 1.0), self._tick)
        self.get_logger().info(
            f'Driving compute_agents at {hz:.1f} Hz for {len(self._agents.agents)} agents'
        )

    def _build_robot(self) -> Agent:
        robot = Agent()
        robot.id = -1
        robot.type = Agent.ROBOT
        robot.name = 'robot'
        robot.group_id = -1
        yaw = float(self.get_parameter('robot_yaw').value)
        robot.position = _pose_from_xy(
            float(self.get_parameter('robot_x').value),
            float(self.get_parameter('robot_y').value),
            yaw,
        )
        robot.yaw = yaw
        robot.desired_velocity = 1.0
        robot.radius = 0.4
        robot.cyclic_goals = False
        robot.behavior = AgentBehavior()
        robot.behavior.type = AgentBehavior.BEH_REGULAR
        return robot

    def _tick(self) -> None:
        if not self._client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn('compute_agents service not ready', throttle_duration_sec=2.0)
            return

        req = ComputeAgents.Request()
        req.robot = self._robot
        self._agents.header.stamp = self.get_clock().now().to_msg()
        req.current_agents = self._agents

        future = self._client.call_async(req)
        future.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'compute_agents failed: {exc}')
            return
        if response is None:
            return
        self._agents = response.updated_agents
        self._agents.header.frame_id = self._frame_id


def main() -> None:
    rclpy.init()
    node = HunavSimDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
