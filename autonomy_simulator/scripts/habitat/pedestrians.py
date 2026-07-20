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

"""Multi-pedestrian oracle navigation on Habitat navmesh (evt_bench style)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import habitat_sim
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from habitat.config import Config
from habitat.coords import xy
from habitat.spawn import map_dist, pick_dispersed_navigable_points
from habitat.sim import Session

try:
    from pedsim_msgs.msg import TrackedPerson, TrackedPersons
except ImportError:  # pragma: no cover - optional at edit time
    TrackedPerson = None  # type: ignore[misc, assignment]
    TrackedPersons = None  # type: ignore[misc, assignment]


def _map_to_habitat(map_x: float, map_y: float, floor: float) -> np.ndarray:
    return np.array([map_x, floor, -map_y], dtype=np.float32)


def _habitat_to_map(pos: np.ndarray) -> tuple[float, float]:
    return xy(pos)


def _quat_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _quat_multiply(
    q1: tuple[float, float, float, float],
    q2: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _rotate_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return (x * cy - y * sy, x * sy + y * cy)


@dataclass
class _Pedestrian:
    track_id: int
    map_x: float
    map_y: float
    yaw: float
    goals: list[np.ndarray]
    avatar: str
    radius: float
    height: float
    goal_idx: int = 0
    old_map_xy: tuple[float, float] | None = None
    lin_speed: float = 1.0
    stall_steps: int = 0
    sidestep_sign: float = 1.0
    nav_rel_map: np.ndarray | None = None
    robot_far: bool = False


class PedestrianSim:
    """Spawn and step one dynamic-agent group; publish tracks + RViz markers."""

    _COLORS = (
        (0.9, 0.25, 0.2),
        (0.2, 0.55, 0.95),
        (0.95, 0.75, 0.15),
        (0.55, 0.2, 0.85),
        (0.2, 0.85, 0.55),
        (0.85, 0.45, 0.2),
        (0.35, 0.85, 0.85),
        (0.85, 0.35, 0.55),
    )

    def __init__(
        self,
        node: Node,
        cfg: Config,
        session: Session,
        *,
        actor_kind: str,
        agent_count: int,
        goal_count: int,
        linear_speed: float,
        seed: int,
        tracked_topic: str,
        viz_topic: str,
        track_id_offset: int = 0,
        blocked_positions: list[tuple[float, float, float]] | None = None,
    ) -> None:
        if TrackedPersons is None:
            raise RuntimeError('pedsim_msgs is required when pedestrians_enabled=true')
        self._node = node
        self._cfg = cfg
        self._session = session
        self._logger = node.get_logger()
        self._actor_kind = str(actor_kind).strip().lower() or 'humanoid'
        self._agent_count = max(0, int(agent_count))
        self._goal_count = max(1, int(goal_count))
        self._linear_speed = float(linear_speed)
        self._seed = int(seed)
        self._track_id_offset = int(track_id_offset)
        self._blocked_positions = list(blocked_positions or [])
        self._robot_debug_mesh_viz = bool(getattr(cfg, 'robot_agents_debug_mesh_viz', False))
        qos = qos_profile_system_default
        self._publish_tracks = self._actor_kind != 'robot'
        self._tracks_pub = (
            node.create_publisher(TrackedPersons, tracked_topic, qos)
            if self._publish_tracks else None
        )
        self._viz_pub = node.create_publisher(
            MarkerArray, viz_topic, qos,
        )
        self._people: list[_Pedestrian] = []
        self._birth = node.get_clock().now()
        self._robot_model = None
        if self._agent_count > 0:
            if self._actor_kind == 'robot':
                from habitat.robot.manager import RobotMeshManager
                from habitat.robot.model_publisher import RobotModelPublisher

                self._humanoids = RobotMeshManager(cfg, session, self._logger)
                self._robot_model = RobotModelPublisher(node, cfg)
                asset_hint = 'robot URDF assets (robot_asset_root or autonomy_simulator/urdf)'
            else:
                from habitat.humanoid.manager import HumanoidMeshManager

                self._humanoids = HumanoidMeshManager(cfg, session, self._logger)
                asset_hint = (
                    'humanoid URDF assets (humanoid_data_root / '
                    'AUTONOMY_HUMANOID_DATA_ROOT)'
                )
            if not self._humanoids.active:
                raise RuntimeError(
                    f'pedestrians_enabled requires {asset_hint}. '
                    'See autonomy_simulator/param/habitat.yaml and '
                    'autonomy_lerobot/config/data_paths.yaml',
                )
        else:
            self._humanoids = None
        self.reset()

    def reset(self) -> None:
        cfg = self._cfg
        session = self._session
        pf = session.pathfinder
        if pf is None or not pf.is_loaded:
            self._logger.warning(f'Navmesh unavailable; {self._actor_kind} agents disabled')
            self._people = []
            return

        floor = session.floor_height
        robot_x, robot_y, _, _ = session.map_pose()
        seed = self._seed or (hash((cfg.scene_id, self._actor_kind)) & 0xFFFFFFFF)

        def sample() -> np.ndarray:
            return session._snap(pf.get_random_navigable_point())  # noqa: SLF001

        spawn_pts = pick_dispersed_navigable_points(
            sample,
            self._agent_count + 1,
            seed=seed,
        )
        avatar_plan = (
            self._humanoids.pick_avatars(self._agent_count, seed)
            if self._humanoids is not None and self._humanoids.active
            else []
        )
        self._people = []
        rng = np.random.default_rng(seed + 17)
        pid = 1 + self._track_id_offset
        for point in spawn_pts:
            mx, my = _habitat_to_map(point)
            if math.hypot(mx - robot_x, my - robot_y) < cfg.pedestrian_spawn_min_robot_dist:
                continue
            actor_radius = cfg.pedestrian_radius
            actor_height = cfg.pedestrian_height
            avatar = (
                avatar_plan[len(self._people)]
                if len(self._people) < len(avatar_plan)
                else avatar_plan[-1] if avatar_plan else cfg.humanoid_avatar
            )
            if (
                self._actor_kind == 'robot'
                and self._humanoids is not None
                and self._robot_debug_mesh_viz
            ):
                actor_radius = self._humanoids.actor_radius(avatar)
                actor_height = self._humanoids.actor_height(avatar)
            blocked = False
            for bx, by, br in self._blocked_positions:
                if math.hypot(mx - bx, my - by) < max(cfg.pedestrian_avoid_dist * 0.6, actor_radius + br):
                    blocked = True
                    break
            if blocked:
                continue
            goals = self._sample_goals(point, floor, pf, rng)
            speed = self._linear_speed * float(rng.uniform(0.8, 1.2))
            self._people.append(_Pedestrian(
                track_id=pid,
                map_x=mx,
                map_y=my,
                yaw=float(rng.uniform(-math.pi, math.pi)),
                goals=goals,
                avatar=avatar,
                radius=actor_radius,
                height=actor_height,
                lin_speed=speed,
                sidestep_sign=1.0 if pid % 2 == 0 else -1.0,
            ))
            pid += 1
            if len(self._people) >= self._agent_count:
                break

        avatar_summary = ', '.join(p.avatar for p in self._people)
        self._logger.info(
            f'Spawned {len(self._people)} dynamic actors kind={self._actor_kind} '
            f'(goals={self._goal_count}, seed={seed}, assets={avatar_summary})',
        )
        if self._robot_model is not None and self._people:
            self._robot_model.set_assets([ped.avatar for ped in self._people])
        if self._humanoids is not None and self._humanoids.active:
            self._humanoids.respawn(self._people, floor)

    def positions(self) -> list[tuple[float, float, float]]:
        return [(p.map_x, p.map_y, p.radius) for p in self._people]

    def step(
        self,
        dt: float,
        stamp: rclpy.time.Time,
        *,
        external_positions: list[tuple[float, float, float]] | None = None,
    ) -> None:
        if not self._people:
            return
        cfg = self._cfg
        floor = self._session.floor_height
        pf = self._session.pathfinder
        robot_x, robot_y, _, _ = self._session.map_pose()
        robot_xy = (robot_x, robot_y)

        positions = self.positions()
        if external_positions:
            positions.extend(external_positions)
        for idx, ped in enumerate(self._people):
            ped.old_map_xy = (ped.map_x, ped.map_y)
            ped.nav_rel_map = None
            ped.robot_far = False
            self._step_one(ped, dt, robot_xy, positions, floor, pf, cfg)
            if self._humanoids is not None and self._humanoids.active:
                rel = ped.nav_rel_map if ped.nav_rel_map is not None else np.zeros(2)
                self._humanoids.set_nav_rel(idx, rel)

        if self._humanoids is not None and self._humanoids.active:
            robot_far = [p.robot_far for p in self._people]
            self._humanoids.sync(self._people, floor, robot_far)
        if self._robot_model is not None and self._people:
            joint_states = None
            if self._actor_kind == 'robot' and self._humanoids is not None:
                joint_states = self._humanoids.joint_state_maps()
            self._robot_model.publish_poses(
                stamp,
                [(ped.map_x, ped.map_y, floor, ped.yaw) for ped in self._people],
                joint_states=joint_states,
            )

        self._publish(stamp, dt)

    def _sample_goals(
        self,
        start: np.ndarray,
        floor: float,
        pf: Any,
        rng: np.random.Generator,
    ) -> list[np.ndarray]:
        cfg = self._cfg
        goals: list[np.ndarray] = []
        for _ in range(self._goal_count):
            for _try in range(200):
                candidate = pf.get_random_navigable_point()
                if goals and map_dist(candidate, goals[-1]) < cfg.pedestrian_waypoint_min_dist:
                    continue
                if map_dist(candidate, start) < cfg.pedestrian_waypoint_min_dist:
                    continue
                goals.append(candidate)
                break
        if not goals:
            goals.append(start)
        return goals

    def _shortest_path(self, start: np.ndarray, end: np.ndarray, pf: Any) -> list[np.ndarray]:
        path = habitat_sim.ShortestPath()
        path.requested_start = start
        path.requested_end = end
        if pf.find_path(path) and path.points:
            return list(path.points)
        return [start, end]

    @staticmethod
    def _nav_direction(
        rel_targ: np.ndarray,
        self_xy: tuple[float, float],
        others: list[tuple[float, float, float]],
        cfg: Config,
        self_radius: float,
        sidestep_sign: float,
    ) -> np.ndarray:
        """Goal attraction + pairwise repulsion; tangential sidestep if blocked."""
        targ_norm = float(np.linalg.norm(rel_targ))
        if targ_norm > 1e-4:
            direction = rel_targ / targ_norm
        else:
            direction = np.array([1.0, 0.0], dtype=np.float64)

        repulse = np.zeros(2, dtype=np.float64)
        personal = self_radius * 2.2
        for ox, oy, other_radius in others:
            off = np.array([self_xy[0] - ox, self_xy[1] - oy], dtype=np.float64)
            dist = float(np.linalg.norm(off))
            avoid_dist = max(cfg.pedestrian_avoid_dist, self_radius + other_radius)
            if dist > avoid_dist:
                continue
            if dist < 1e-3:
                off = np.array([sidestep_sign, 0.3 * sidestep_sign], dtype=np.float64)
                dist = float(np.linalg.norm(off))
            pair_personal = max(personal, (self_radius + other_radius) * 1.2)
            repulse += off / dist * max((pair_personal - dist) / pair_personal, 0.15)

        combined = direction + 1.8 * repulse
        comb_norm = float(np.linalg.norm(combined))
        if comb_norm < 0.2:
            tangent = np.array([-direction[1], direction[0]], dtype=np.float64)
            combined = direction * 0.2 + tangent * sidestep_sign
            comb_norm = float(np.linalg.norm(combined))
        if comb_norm < 1e-4:
            return direction
        return combined / comb_norm

    def _refresh_goals(self, ped: _Pedestrian, floor: float, pf: Any) -> None:
        hab = _map_to_habitat(ped.map_x, ped.map_y, floor)
        ped.goals = self._sample_goals(hab, floor, pf, np.random.default_rng())
        ped.goal_idx = 0
        ped.stall_steps = 0

    def _step_one(
        self,
        ped: _Pedestrian,
        dt: float,
        robot_xy: tuple[float, float],
        all_xy: list[tuple[float, float, float]],
        floor: float,
        pf: Any,
        cfg: Config,
    ) -> None:
        self_xy = (ped.map_x, ped.map_y)
        robot_dist = math.hypot(robot_xy[0] - self_xy[0], robot_xy[1] - self_xy[1])
        if (
            cfg.pedestrian_robot_activate_dist > 0.0
            and robot_dist > cfg.pedestrian_robot_activate_dist
        ):
            ped.robot_far = True
            ped.nav_rel_map = np.zeros(2, dtype=np.float64)
            return

        hab_pos = _map_to_habitat(ped.map_x, ped.map_y, floor)
        goal = ped.goals[ped.goal_idx]
        path = self._shortest_path(hab_pos, goal, pf)
        next_hab = path[1] if len(path) > 1 else goal
        next_x, next_y = _habitat_to_map(next_hab)
        rel = np.array([next_x - ped.map_x, next_y - ped.map_y], dtype=np.float64)

        others = [p for p in all_xy if (p[0], p[1]) != self_xy]
        rel = self._nav_direction(rel, self_xy, others, cfg, ped.radius, ped.sidestep_sign)
        ped.nav_rel_map = rel.copy()

        goal_x, goal_y = _habitat_to_map(goal)
        dist_goal = math.hypot(goal_x - ped.map_x, goal_y - ped.map_y)
        if dist_goal < cfg.pedestrian_dist_thresh:
            ped.goal_idx = (ped.goal_idx + 1) % len(ped.goals)
            if ped.goal_idx == 0:
                self._refresh_goals(ped, floor, pf)
            return

        step = ped.lin_speed * dt
        prev_x, prev_y = ped.map_x, ped.map_y
        ped.map_x += float(rel[0]) * step
        ped.map_y += float(rel[1]) * step
        snapped = self._session._snap(  # noqa: SLF001
            _map_to_habitat(ped.map_x, ped.map_y, floor),
        )
        ped.map_x, ped.map_y = _habitat_to_map(snapped)
        ped.yaw = math.atan2(rel[1], rel[0])

        moved = math.hypot(ped.map_x - prev_x, ped.map_y - prev_y)
        if moved < 0.02 * step / max(dt, 1e-3):
            ped.stall_steps += 1
        else:
            ped.stall_steps = 0

        if ped.stall_steps > 30:
            ped.sidestep_sign *= -1.0
            ped.yaw += ped.sidestep_sign * 0.8
            self._refresh_goals(ped, floor, pf)

    def _publish(self, stamp: rclpy.time.Time, dt: float) -> None:
        cfg = self._cfg
        floor = self._session.floor_height
        header = Header(stamp=stamp.to_msg(), frame_id=cfg.map_frame)
        tracks = TrackedPersons() if self._publish_tracks else None
        if tracks is not None:
            tracks.header = header
        markers = MarkerArray()

        age_ns = max(0, (stamp - self._birth).nanoseconds)
        age = Duration(sec=age_ns // 1_000_000_000, nanosec=age_ns % 1_000_000_000)

        for idx, ped in enumerate(self._people):
            track = TrackedPerson()
            track.track_id = ped.track_id
            track.is_occluded = False
            track.is_matched = True
            track.age = age
            track.pose.pose.position.x = ped.map_x
            track.pose.pose.position.y = ped.map_y
            track.pose.pose.position.z = floor

            vx, vy = 0.0, 0.0
            if ped.old_map_xy is not None and dt > 0.0:
                vx = (ped.map_x - ped.old_map_xy[0]) / dt
                vy = (ped.map_y - ped.old_map_xy[1]) / dt
            speed = math.hypot(vx, vy)
            yaw = math.atan2(vy, vx) if speed > 0.05 else ped.yaw
            half = yaw * 0.5
            track.pose.pose.orientation.z = math.sin(half)
            track.pose.pose.orientation.w = math.cos(half)
            # Finite covariances: mark xy position and yaw as valid for the RViz plugin.
            track.pose.covariance[0] = 0.25
            track.pose.covariance[7] = 0.25
            track.pose.covariance[14] = 1.0e6
            track.pose.covariance[35] = 0.05

            track.twist.twist.linear.x = vx
            track.twist.twist.linear.y = vy
            if tracks is not None:
                tracks.tracks.append(track)

            rgb = self._COLORS[idx % len(self._COLORS)]
            if self._actor_kind == 'robot' and self._humanoids is not None:
                spec = self._humanoids.actor_marker_spec(ped.avatar)
                if spec is not None:
                    parts = spec.get('parts', [spec])
                    for part in parts[:1]:
                        mesh = Marker()
                        mesh.header = header
                        mesh.ns = 'robot_meshes'
                        mesh.id = ped.track_id
                        mesh.action = Marker.ADD
                        offx, offy, offz = part['offset_xyz']
                        rotx, roty = _rotate_xy(float(offx), float(offy), yaw)
                        mesh.pose.position.x = ped.map_x + rotx
                        mesh.pose.position.y = ped.map_y + roty
                        mesh.pose.position.z = floor + float(offz)
                        q_yaw = _quat_from_euler(0.0, 0.0, yaw)
                        q_off = _quat_from_euler(*part['offset_rpy'])
                        # Apply the mesh's local corrective rotation first, then
                        # rotate the assembled robot in the map frame by yaw.
                        qx, qy, qz, qw = _quat_multiply(q_off, q_yaw)
                        mesh.pose.orientation.x = qx
                        mesh.pose.orientation.y = qy
                        mesh.pose.orientation.z = qz
                        mesh.pose.orientation.w = qw
                        rgba = part.get('color', (rgb[0], rgb[1], rgb[2], 1.0))
                        mesh.color.r = float(rgba[0])
                        mesh.color.g = float(rgba[1])
                        mesh.color.b = float(rgba[2])
                        mesh.color.a = float(rgba[3])
                        part_type = str(part.get('type', 'mesh'))
                        if 'mesh' in part:
                            mesh.type = Marker.MESH_RESOURCE
                            sx, sy, sz = part.get('scale', (1.0, 1.0, 1.0))
                            mesh.scale.x = float(sx)
                            mesh.scale.y = float(sy)
                            mesh.scale.z = float(sz)
                            mesh.mesh_resource = f'file://{part["mesh"]}'
                            mesh.mesh_use_embedded_materials = True
                        elif part_type == 'box':
                            mesh.type = Marker.CUBE
                            sx, sy, sz = part['size']
                            mesh.scale.x = float(sx)
                            mesh.scale.y = float(sy)
                            mesh.scale.z = float(sz)
                        elif part_type == 'cylinder':
                            mesh.type = Marker.CYLINDER
                            sx, sy, sz = part['size']
                            mesh.scale.x = float(sx)
                            mesh.scale.y = float(sy)
                            mesh.scale.z = float(sz)
                        else:
                            continue
                        markers.markers.append(mesh)
                    continue
            body = Marker()
            body.header = header
            body.ns = 'pedestrians'
            body.id = ped.track_id
            body.type = Marker.CYLINDER
            body.action = Marker.ADD
            body.pose.position.x = ped.map_x
            body.pose.position.y = ped.map_y
            body.pose.position.z = floor + ped.height * 0.5
            body.pose.orientation.w = 1.0
            body.scale.x = body.scale.y = ped.radius * 2.0
            body.scale.z = ped.height
            body.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=0.85)
            markers.markers.append(body)

            head = Marker()
            head.header = header
            head.ns = 'pedestrian_heads'
            head.id = ped.track_id
            head.type = Marker.SPHERE
            head.action = Marker.ADD
            head.pose.position.x = ped.map_x
            head.pose.position.y = ped.map_y
            head.pose.position.z = floor + ped.height + ped.radius * 0.6
            head.pose.orientation.w = 1.0
            head.scale.x = head.scale.y = head.scale.z = ped.radius * 1.2
            head.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=0.9)
            markers.markers.append(head)

        if tracks is not None and self._tracks_pub is not None:
            self._tracks_pub.publish(tracks)
        self._viz_pub.publish(markers)

    def nearest_robot_distance(self) -> float | None:
        if not self._people:
            return None
        rx, ry, _, _ = self._session.map_pose()
        return min(math.hypot(p.map_x - rx, p.map_y - ry) for p in self._people)
