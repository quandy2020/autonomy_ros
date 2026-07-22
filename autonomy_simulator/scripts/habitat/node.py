#
# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

"""ROS 2 node bridging Habitat-Sim MP3D scenes to camera and pose topics."""

from __future__ import annotations

from collections.abc import Callable

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from habitat.config import load
from habitat.odom import OdomPublisher


class BridgeNode(Node):
    """Bridge Habitat agent pose and camera sensors to ROS 2 topics."""

    def __init__(self) -> None:
        super().__init__('habitat_node')
        qos = qos_profile_system_default
        cfg = self._cfg = load(self)
        self._dt = 1.0 / cfg.update_rate_hz
        self._cmd_time = self.get_clock().now()

        self.get_logger().info(
            'Loading Habitat-Sim / scene (first launch can take 30–90s)...'
        )
        # Defer habitat_sim imports so launch logs show progress first.
        from habitat.camera import CameraPublisher
        from habitat.navmesh import NavMeshPublisher
        from habitat.ply import PlyPublisher
        from habitat.sim import Session

        self._cam = CameraPublisher(self, cfg)
        self._odom = OdomPublisher(self, cfg)
        # habitat_odom_tf_node keeps map→odom→base_footprint alive while Session() blocks here.
        self._session = Session(cfg, self.get_logger())
        self._ply = None
        if cfg.semantic_pointcloud_rate_hz >= 0.0 or cfg.occupancy_grid_rate_hz >= 0.0:
            self._ply = PlyPublisher(
                self,
                cfg,
                self.get_logger(),
                map_floor_height=self._session.floor_height,
            )
            grid = self._ply.map_grid
            if grid is not None:
                self._session.ensure_spawn_on_free_map(grid, self.get_logger())
        self._scene_bounds = None
        from habitat.scene_bounds import SceneBoundsPublisher
        self._scene_bounds = SceneBoundsPublisher(self, cfg, self._session, self.get_logger())
        self._topdown_interactive = None
        if cfg.topdown_enabled and cfg.topdown_mode == 'interactive':
            from habitat.topdown_interactive import TopdownInteractive
            self._topdown_interactive = TopdownInteractive(
                self, cfg, self._session, self.get_logger(),
            )
        self._pedestrians: list[object] = []
        if (
            cfg.human_agent_count > 0
            or cfg.robot_agent_count > 0
            or (cfg.pedestrians_enabled and cfg.pedestrian_count > 0)
        ):
            from habitat.pedestrians import PedestrianSim
            occupied_positions: list[tuple[float, float, float]] = []
            if cfg.human_agent_count > 0:
                human_sim = PedestrianSim(
                    self,
                    cfg,
                    self._session,
                    actor_kind='humanoid',
                    agent_count=cfg.human_agent_count,
                    goal_count=cfg.human_agent_goal_count,
                    linear_speed=cfg.human_agent_linear_speed,
                    seed=cfg.human_agent_seed,
                    tracked_topic=cfg.human_agents_tracked_topic,
                    viz_topic=cfg.human_agents_viz_topic,
                    track_id_offset=0,
                    blocked_positions=occupied_positions,
                )
                self._pedestrians.append(human_sim)
                occupied_positions.extend(human_sim.positions())
            if cfg.robot_agent_count > 0:
                robot_sim = PedestrianSim(
                    self,
                    cfg,
                    self._session,
                    actor_kind='robot',
                    agent_count=cfg.robot_agent_count,
                    goal_count=cfg.robot_agent_goal_count,
                    linear_speed=cfg.robot_agent_linear_speed,
                    seed=cfg.robot_agent_seed,
                    tracked_topic=cfg.robot_agents_tracked_topic,
                    viz_topic=cfg.robot_agents_viz_topic,
                    track_id_offset=10000,
                    blocked_positions=occupied_positions,
                )
                self._pedestrians.append(robot_sim)
                occupied_positions.extend(robot_sim.positions())
        self._navmesh = None
        if cfg.navmesh_rate_hz >= 0.0:
            self._navmesh = NavMeshPublisher(self, cfg, self._session)

        stamp = self.get_clock().now()
        self._odom.publish(stamp, self._session)
        if self._scene_bounds is not None:
            self._scene_bounds.publish_once(stamp)

        self._pose_pub = self.create_publisher(PoseStamped, cfg.agent_pose_topic, qos)
        self.create_subscription(PoseStamped, cfg.set_agent_pose_topic, self._on_pose, qos)
        self.create_subscription(Twist, cfg.cmd_vel_topic, self._on_cmd, qos)
        self.create_timer(self._dt, self._on_tick)
        if self._ply is not None:
            self._timer(cfg.semantic_pointcloud_rate_hz, self._on_cloud)
            self._timer(cfg.occupancy_grid_rate_hz, self._on_map)
        if self._navmesh is not None:
            navmesh_hz = cfg.navmesh_rate_hz
            self._timer(1.0 if navmesh_hz == 0.0 else navmesh_hz, self._on_navmesh)

        self.get_logger().info(
            f'[habitat] BridgeNode ready scene={cfg.scene_id} '
            f'path={cfg.scene_dir()} cmd_vel={cfg.cmd_vel_topic}'
            + (f' human_agents={cfg.human_agent_count}' if cfg.human_agent_count > 0 else '')
            + (f' robot_agents={cfg.robot_agent_count}' if cfg.robot_agent_count > 0 else '')
            + (f' topdown={cfg.topdown_topic}' if cfg.topdown_enabled else '')
            + (
                f' mode={cfg.topdown_mode}'
                if cfg.topdown_enabled else ''
            )
        )

    def _timer(self, hz: float, cb: Callable[[], None]) -> None:
        if float(hz) > 0.0:
            self.create_timer(1.0 / float(hz), cb)

    def _on_cloud(self) -> None:
        if self._ply is not None:
            self._ply.publish_cloud(self.get_clock().now())

    def _on_map(self) -> None:
        if self._ply is not None:
            self._ply.publish_map(self.get_clock().now())

    def _on_navmesh(self) -> None:
        if self._navmesh is not None:
            self._navmesh.publish(self.get_clock().now())

    def _on_cmd(self, msg: Twist) -> None:
        self._cmd_time = self.get_clock().now()
        self._session.set_velocity(float(msg.linear.x), float(msg.angular.z))

    def _on_pose(self, msg: PoseStamped) -> None:
        self._session.set_pose(msg)
        self._session.set_velocity(0.0, 0.0)

    def _timed_out(self) -> bool:
        dt = (self.get_clock().now() - self._cmd_time).nanoseconds * 1e-9
        return dt > self._cfg.cmd_vel_timeout

    def _on_tick(self) -> None:
        if not rclpy.ok():
            return
        stamp = self.get_clock().now()
        timed_out = self._timed_out()
        self._session.advance(self._dt, timed_out)
        snapshots = [sim.positions() for sim in self._pedestrians]
        for idx, sim in enumerate(self._pedestrians):
            external_positions = []
            for other_idx, positions in enumerate(snapshots):
                if other_idx != idx:
                    external_positions.extend(positions)
            sim.step(self._dt, stamp, external_positions=external_positions)
            snapshots[idx] = sim.positions()
        obs = self._session.observe()
        self._pose_pub.publish(self._session.agent_pose(self._cfg.agent_pose_frame, stamp))
        self._odom.publish(stamp, self._session, timed_out)
        self._cam.publish(stamp, obs, topdown_hfov_deg=self._session.topdown_hfov_deg())

    def destroy_node(self) -> bool:
        self._session.close()
        return super().destroy_node()
