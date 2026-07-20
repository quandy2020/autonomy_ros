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

"""Habitat-Sim scene loading and agent session."""

from __future__ import annotations

import math
import os
import shutil
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped

import habitat_sim
from habitat_sim.agent import AgentConfiguration, SixDOFPose
from habitat_sim.sensor import CameraSensorSpec, SensorSubType, SensorType
from habitat.config import Config
from habitat.coords import (
    from_pose,
    map_forward_xy,
    quat,
    quat_look_at,
    quat_look_down,
    ros_yaw_from_quat,
    to_odom_pose,
    xy,
)
from habitat.spawn import pick_dispersed_navigable_points


class Session:
    """Owns a Habitat-Sim instance and exposes agent pose plus sensor observations."""

    def __init__(self, cfg: Config, logger: Any) -> None:
        self._cfg = cfg
        self._logger = logger
        self._sim = self._open()
        self._agent = self._sim.initialize_agent(0)
        self._linear = 0.0
        self._angular = 0.0
        # Odom is aligned with map (identity map→odom TF).
        self._floor_height = 0.0
        self._room_topdown_cache: tuple[np.ndarray, np.quaternion] | None = None
        self._room_bounds_cache: tuple[float, float, float, float] | None = None
        self._interactive_topdown: tuple[np.ndarray, np.quaternion, float] | None = None
        self._spawn()

    def close(self) -> None:
        sim = getattr(self, '_sim', None)
        if sim is not None:
            sim.close()

    def set_velocity(self, linear: float, angular: float) -> None:
        self._linear = float(linear)
        self._angular = float(angular)

    def velocity(self) -> tuple[float, float]:
        return self._linear, self._angular

    def step(self, dt: float, timed_out: bool) -> dict[str, Any]:
        self.advance(dt, timed_out)
        return self.observe()

    def advance(self, dt: float, timed_out: bool) -> None:
        """Integrate robot motion without rendering (humanoid sync runs before observe)."""
        if timed_out:
            self._linear = 0.0
            self._angular = 0.0

        x, y, _, yaw_val = self.map_pose()
        new_yaw = yaw_val + self._angular * dt
        # Prefer translation only when the step stays on navmesh; otherwise yaw-only
        # so we do not scrape walls / jitter against snap_point.
        if abs(self._linear) > 1e-4:
            nx = x + self._linear * math.cos(yaw_val) * dt
            ny = y + self._linear * math.sin(yaw_val) * dt
            if not self._try_move_map(nx, ny, new_yaw):
                self._move_map(x, y, new_yaw)
                return
            return
        self._move_map(x, y, new_yaw)

    def observe(self) -> dict[str, Any]:
        if self._cfg.topdown_enabled:
            self._update_topdown_sensor()
        return self._sim.get_sensor_observations()

    @property
    def sim(self) -> habitat_sim.Simulator:
        return self._sim

    @staticmethod
    def _needs_physics(cfg: Config) -> bool:
        if not (cfg.pedestrians_enabled and cfg.pedestrian_count > 0):
            return False
        if str(cfg.dynamic_actor_kind).strip().lower() == 'robot':
            from habitat.robot.paths import robot_assets_available

            ok, _ = robot_assets_available(cfg)
            return ok
        from habitat.humanoid.paths import humanoid_assets_available

        ok, _ = humanoid_assets_available(cfg)
        return ok

    def agent_pose(self, frame_id: str, stamp: rclpy.time.Time) -> PoseStamped:
        x, y, z, yaw_val = self.map_pose()
        return to_odom_pose(x, y, z, yaw_val, frame_id, stamp)

    def map_pose(self) -> tuple[float, float, float, float]:
        state = self._agent.get_state()
        map_x, map_y = xy(state.position)
        return map_x, map_y, self._cfg.base_link_height, ros_yaw_from_quat(state.rotation)

    def color_sensor_pose(self) -> tuple[np.ndarray, np.quaternion]:
        """World-frame position and rotation of the RGB pinhole sensor."""
        return self._sensor_pose('color_sensor', self._cfg.sensor_height)

    def topdown_sensor_pose(self) -> tuple[np.ndarray, np.quaternion] | None:
        """World-frame pose of the topdown sensor, if enabled."""
        if not self._cfg.topdown_enabled:
            return None
        return self._sensor_pose('topdown_color_sensor', self._cfg.topdown_height_m)

    def _sensor_pose(
        self,
        uuid: str,
        fallback_height: float,
    ) -> tuple[np.ndarray, np.quaternion]:
        state = self._agent.get_state()
        sensor = state.sensor_states.get(uuid)
        if sensor is not None:
            return (
                np.array(sensor.position, dtype=np.float64),
                sensor.rotation,
            )
        position = np.array(state.position, dtype=np.float64)
        position[1] += float(fallback_height)
        return position, state.rotation

    @property
    def pathfinder(self):
        return self._sim.pathfinder

    @property
    def floor_height(self) -> float:
        return self._floor_height

    def odom_pose(self) -> tuple[float, float, float, float]:
        """Same as map_pose; map→odom TF is identity."""
        return self.map_pose()

    def set_pose(self, pose: PoseStamped) -> None:
        if pose.header.frame_id not in (
            self._cfg.map_frame,
            self._cfg.odom_frame,
        ):
            self._logger.warning(
                f'Ignoring set_pose in frame {pose.header.frame_id!r}; '
                f'use {self._cfg.map_frame!r} or {self._cfg.odom_frame!r}'
            )
            return
        position, rotation = from_pose(pose, self._floor_height)
        self._apply(self._snap(position), rotation)

    def _assets(self) -> None:
        cfg = self._cfg
        glb_path = os.path.join(cfg.scene_dir(), f'{cfg.scene_id}.glb')
        if not os.path.isfile(glb_path):
            raise FileNotFoundError(f'MP3D scene glb not found: {glb_path}')

        dataset_config = cfg.dataset_config()
        if os.path.isfile(dataset_config):
            return
        if not cfg.package_scene_dataset_config:
            raise FileNotFoundError(f'Scene dataset config not found: {dataset_config}')
        # First run: copy packaged mp3d.scene_dataset_config.json into mp3d_root.
        os.makedirs(cfg.mp3d_root, exist_ok=True)
        shutil.copy2(cfg.package_scene_dataset_config, dataset_config)
        self._logger.info(f'Installed MP3D scene dataset config to {dataset_config}')

    def _sensors(self) -> list[CameraSensorSpec]:
        """Pinhole RGB-D-Semantic rig (habitat-sim TUTORIALS/scripts/sim_utils.py)."""
        cfg = self._cfg
        resolution = [cfg.image_height, cfg.image_width]
        specs: list[CameraSensorSpec] = []

        for uuid, sensor_type in (
            ('color_sensor', SensorType.COLOR),
            ('depth_sensor', SensorType.DEPTH),
            ('semantic_sensor', SensorType.SEMANTIC),
        ):
            spec = CameraSensorSpec()
            spec.uuid = uuid
            spec.sensor_type = sensor_type
            spec.sensor_subtype = SensorSubType.PINHOLE
            spec.resolution = resolution
            spec.position = [0.0, cfg.sensor_height, 0.0]
            spec.orientation = [0.0, 0.0, 0.0]
            spec.hfov = cfg.camera_horizontal_fov_deg
            specs.append(spec)

        if cfg.topdown_enabled:
            spec = CameraSensorSpec()
            spec.uuid = 'topdown_color_sensor'
            spec.sensor_type = SensorType.COLOR
            spec.resolution = [cfg.topdown_height, cfg.topdown_width]
            spec.position = [0.0, cfg.topdown_height_m, 0.0]
            spec.orientation = [-math.pi / 2.0, 0.0, 0.0]
            if cfg.topdown_mode == 'room':
                spec.sensor_subtype = SensorSubType.ORTHOGRAPHIC
                spec.hfov = self._room_ortho_scale(cfg)
            else:
                spec.sensor_subtype = SensorSubType.PINHOLE
                spec.hfov = cfg.topdown_hfov_deg
            specs.append(spec)

        return specs

    @staticmethod
    def _navmesh_bounds_xz(cfg: Config) -> tuple[float, float, float, float] | None:
        """Load scene .navmesh without full sim; return lo_x, lo_z, hi_x, hi_z."""
        navmesh_path = os.path.join(cfg.scene_dir(), f'{cfg.scene_id}.navmesh')
        if not os.path.isfile(navmesh_path):
            return None
        pf = habitat_sim.PathFinder()
        if not pf.load_nav_mesh(navmesh_path):
            return None
        lo, hi = pf.get_bounds()
        return float(lo[0]), float(lo[2]), float(hi[0]), float(hi[2])

    def _room_ortho_scale(self, cfg: Config) -> float:
        """Orthographic width in meters (Habitat ORTHOGRAPHIC hfov)."""
        if cfg.topdown_ortho_scale > 0.0:
            return cfg.topdown_ortho_scale
        bounds = self._navmesh_bounds_xz(cfg)
        if bounds is None:
            return 40.0
        lo_x, lo_z, hi_x, hi_z = bounds
        span_x = hi_x - lo_x
        span_z = hi_z - lo_z
        aspect = cfg.topdown_height / max(cfg.topdown_width, 1)
        margin = cfg.topdown_room_margin
        # Fit both axes: hfov = ground width; height coverage = hfov * aspect.
        return max(span_x * margin, span_z * margin / max(aspect, 1e-3))

    def room_bounds_xz(self) -> tuple[float, float, float, float]:
        return self._room_bounds_xz()

    def ensure_interactive_topdown(self) -> tuple[np.ndarray, np.quaternion, float]:
        """Initial full-room pinhole pose for RViz interactive topdown."""
        if self._interactive_topdown is not None:
            return self._interactive_topdown
        pos, rot = self._overview_pinhole_pose()
        hfov = self._cfg.topdown_hfov_deg
        self._interactive_topdown = (pos, rot, hfov)
        return self._interactive_topdown

    def set_interactive_topdown(
        self,
        pos: np.ndarray,
        rot: np.quaternion,
        hfov: float | None = None,
    ) -> None:
        cur = self._interactive_topdown
        fov = hfov if hfov is not None else (cur[2] if cur else self._cfg.topdown_hfov_deg)
        fov = float(max(20.0, min(120.0, fov)))
        self._interactive_topdown = (pos.astype(np.float32), rot, fov)
        self._apply_topdown_hfov(fov)

    def topdown_hfov_deg(self) -> float:
        if self._interactive_topdown is not None:
            return self._interactive_topdown[2]
        return self._cfg.topdown_hfov_deg

    def _apply_topdown_hfov(self, hfov_deg: float) -> None:
        try:
            sensors = self._agent._sensors  # noqa: SLF001
            sensor = sensors.get('topdown_color_sensor')
            if sensor is not None:
                sensor.specification().hfov = float(hfov_deg)
        except Exception:
            pass

    def _overview_pinhole_pose(self) -> tuple[np.ndarray, np.quaternion]:
        """Pinhole camera height that frames the full scene (for interactive mode)."""
        cfg = self._cfg
        lo_x, lo_z, hi_x, hi_z = self._room_bounds_xz()
        span_x = hi_x - lo_x
        span_z = hi_z - lo_z
        cx = (lo_x + hi_x) * 0.5
        cz = (lo_z + hi_z) * 0.5
        center_floor = self._snap(
            np.array([cx, self._floor_height, cz], dtype=np.float32),
        )
        floor_y = float(center_floor[1])
        margin = cfg.topdown_room_margin
        hfov = math.radians(cfg.topdown_hfov_deg)
        tan_h = math.tan(hfov * 0.5)
        aspect = cfg.topdown_height / max(cfg.topdown_width, 1)
        tan_v = tan_h * aspect
        cam_h = max(
            (span_x * margin) / (2.0 * tan_h),
            (span_z * margin) / (2.0 * max(tan_v, 1e-6)),
            0.5 * math.hypot(span_x, span_z) * margin / min(tan_h, tan_v),
            cfg.topdown_height_m,
        )
        eye = np.array([cx, floor_y + cam_h, cz], dtype=np.float32)
        return eye, quat_look_down()

    def _room_bounds_xz(self) -> tuple[float, float, float, float]:
        """Union navmesh, pathfinder, and scene mesh bounds on the XZ plane."""
        if self._room_bounds_cache is not None:
            return self._room_bounds_cache

        lo_x = lo_z = math.inf
        hi_x = hi_z = -math.inf

        file_bounds = self._navmesh_bounds_xz(self._cfg)
        if file_bounds is not None:
            lo_x = min(lo_x, file_bounds[0])
            lo_z = min(lo_z, file_bounds[1])
            hi_x = max(hi_x, file_bounds[2])
            hi_z = max(hi_z, file_bounds[3])

        pf = self._sim.pathfinder
        if pf.is_loaded:
            plo, phi = pf.get_bounds()
            lo_x = min(lo_x, float(plo[0]))
            lo_z = min(lo_z, float(plo[2]))
            hi_x = max(hi_x, float(phi[0]))
            hi_z = max(hi_z, float(phi[2]))

        try:
            bb = self._sim.get_active_scene_graph().get_root_node().cumulative_bb
            lo_x = min(lo_x, float(bb.min.x))
            lo_z = min(lo_z, float(bb.min.z))
            hi_x = max(hi_x, float(bb.max.x))
            hi_z = max(hi_z, float(bb.max.z))
        except Exception:
            pass

        if not math.isfinite(lo_x):
            lo_x = lo_z = -10.0
            hi_x = hi_z = 10.0

        self._room_bounds_cache = (lo_x, lo_z, hi_x, hi_z)
        return self._room_bounds_cache

    def _room_topdown_pose(self) -> tuple[np.ndarray, np.quaternion]:
        """Fixed overhead camera framing the full scene."""
        if self._room_topdown_cache is not None:
            return self._room_topdown_cache

        cfg = self._cfg
        lo_x, lo_z, hi_x, hi_z = self._room_bounds_xz()
        span_x = hi_x - lo_x
        span_z = hi_z - lo_z
        cx = (lo_x + hi_x) * 0.5
        cz = (lo_z + hi_z) * 0.5
        center_floor = self._snap(
            np.array([cx, self._floor_height, cz], dtype=np.float32),
        )
        floor_y = float(center_floor[1])

        if cfg.topdown_mode == 'room':
            cam_h = max(span_x, span_z) * 0.5 + cfg.topdown_height_m
            eye = np.array([cx, floor_y + cam_h, cz], dtype=np.float32)
            rot = quat_look_down()
            ortho = self._room_ortho_scale(cfg)
            self._logger.info(
                f'Room topdown (ortho): scene {span_x:.1f}x{span_z:.1f} m, '
                f'ortho_scale={ortho:.1f} m, center=({cx:.1f}, {cz:.1f})',
            )
        else:
            margin = cfg.topdown_room_margin
            hfov = math.radians(cfg.topdown_hfov_deg)
            tan_h = math.tan(hfov * 0.5)
            aspect = cfg.topdown_height / max(cfg.topdown_width, 1)
            tan_v = tan_h * aspect
            cam_h = max(
                (span_x * margin) / (2.0 * tan_h),
                (span_z * margin) / (2.0 * max(tan_v, 1e-6)),
                0.5 * math.hypot(span_x, span_z) * margin / min(tan_h, tan_v),
                cfg.topdown_height_m,
            )
            eye = np.array([cx, floor_y + cam_h, cz], dtype=np.float32)
            rot = quat_look_down()
            self._logger.info(
                f'Room topdown: scene {span_x:.1f}x{span_z:.1f} m, '
                f'camera height {cam_h:.1f} m',
            )

        self._room_topdown_cache = (eye, rot)
        return self._room_topdown_cache

    def _update_topdown_sensor(self) -> None:
        """Overview camera: full room, oblique follow, or overhead follow."""
        cfg = self._cfg
        state = self._agent.get_state()
        map_x, map_y, _, yaw = self.map_pose()
        floor = self._floor_height

        if cfg.topdown_mode == 'interactive':
            pos, rot, _ = self.ensure_interactive_topdown()
        elif cfg.topdown_mode == 'room':
            pos, rot = self._room_topdown_pose()
        elif cfg.topdown_mode == 'overhead':
            height = floor + cfg.topdown_height_m
            pos = np.array([map_x, height, -map_y], dtype=np.float32)
            rot = quat_look_down()
        else:
            fx, fy = map_forward_xy(yaw)
            eye_x = map_x - cfg.topdown_distance_m * fx
            eye_y = map_y - cfg.topdown_distance_m * fy
            eye = np.array(
                [eye_x, floor + cfg.topdown_height_m, -eye_y], dtype=np.float32,
            )
            look_x = map_x + cfg.topdown_look_ahead_m * fx
            look_y = map_y + cfg.topdown_look_ahead_m * fy
            target = np.array(
                [
                    look_x,
                    floor + cfg.topdown_look_at_height_m,
                    -look_y,
                ],
                dtype=np.float32,
            )
            pos = eye
            rot = quat_look_at(eye, target)

        state.sensor_states['topdown_color_sensor'] = SixDOFPose(
            position=pos,
            rotation=rot,
        )
        self._agent.set_state(state, reset_sensors=False)

    def _open(self) -> habitat_sim.Simulator:
        self._assets()
        cfg = self._cfg

        sim_cfg = habitat_sim.SimulatorConfiguration()
        sim_cfg.scene_dataset_config_file = cfg.dataset_config()
        sim_cfg.scene_id = cfg.scene_asset()
        sim_cfg.enable_physics = self._needs_physics(cfg)
        sim_cfg.allow_sliding = True
        sim_cfg.requires_textures = True
        sim_cfg.load_semantic_mesh = True

        agent_cfg = AgentConfiguration()
        agent_cfg.sensor_specifications = self._sensors()

        self._logger.info(
            f'Loading MP3D scene {cfg.scene_asset()} from {cfg.mp3d_root}'
            + (' (physics on for humanoid meshes)' if self._needs_physics(cfg) else '')
        )
        return habitat_sim.Simulator(habitat_sim.Configuration(sim_cfg, [agent_cfg]))

    def _apply(self, position: np.ndarray, rotation: np.quaternion) -> None:
        state = self._agent.get_state()
        state.position = position
        state.rotation = rotation
        self._agent.set_state(state, reset_sensors=True)

    def _snap(self, position: np.ndarray) -> np.ndarray:
        if not self._sim.pathfinder.is_loaded:
            return position
        snapped = self._sim.pathfinder.snap_point(position)
        if self._sim.pathfinder.is_navigable(snapped):
            self._floor_height = float(snapped[1])
            return snapped
        return position

    def _try_move_map(self, map_x: float, map_y: float, map_yaw: float) -> bool:
        """Move if navmesh accepts the step; reject large sideways snaps (wall hits)."""
        position = np.array([map_x, self._floor_height, -map_y], dtype=np.float32)
        if not self._sim.pathfinder.is_loaded:
            self._apply(position, quat(map_yaw))
            return True
        snapped = self._sim.pathfinder.snap_point(position)
        if not self._sim.pathfinder.is_navigable(snapped):
            return False
        # Habitat snap can yank the agent sideways into a wall corridor — treat as blocked.
        dx = float(snapped[0] - position[0])
        dz = float(snapped[2] - position[2])
        if math.hypot(dx, dz) > 0.22:
            return False
        self._floor_height = float(snapped[1])
        self._apply(snapped, quat(map_yaw))
        return True

    def _move_map(self, map_x: float, map_y: float, map_yaw: float) -> None:
        position = np.array([map_x, self._floor_height, -map_y], dtype=np.float32)
        self._apply(self._snap(position), quat(map_yaw))

    def _spawn(self) -> None:
        """Place agent on navmesh (dispersed, random, or fixed pose)."""
        if not self._sim.pathfinder.is_loaded:
            self._logger.warning('Navmesh not loaded; using default agent spawn pose')
            self._floor_height = 0.0
            return

        mode = self._cfg.spawn_mode
        if mode == 'fixed':
            position = np.array(
                [self._cfg.spawn_x, self._floor_height, -self._cfg.spawn_y],
                dtype=np.float32,
            )
            point = self._snap(position)
            self._apply(point, quat(self._cfg.spawn_yaw))
            mx, my = xy(point)
            self._logger.info(
                f'Fixed spawn at map ({mx:.2f}, {my:.2f}) yaw={self._cfg.spawn_yaw:.2f}')
            return

        if mode == 'dispersed':
            seed = self._cfg.spawn_seed or (hash(self._cfg.scene_id) & 0xFFFFFFFF)
            points = pick_dispersed_navigable_points(
                lambda: self._snap(self._sim.pathfinder.get_random_navigable_point()),
                self._cfg.spawn_count,
                seed=seed,
            )
            if points:
                idx = min(max(self._cfg.spawn_index, 0), len(points) - 1)
                point = points[idx]
                yaw = (
                    idx * (2.0 * math.pi / self._cfg.spawn_count)
                    if self._cfg.spawn_count > 1 else 0.0
                )
                self._apply(point, quat(yaw))
                mx, my = xy(point)
                self._logger.info(
                    f'Dispersed spawn [{idx + 1}/{self._cfg.spawn_count}] '
                    f'at map ({mx:.2f}, {my:.2f}) yaw={yaw:.2f} seed={seed}')
                return

        point = self._snap(self._sim.pathfinder.get_random_navigable_point())
        self._apply(point, quat(0.0))
        mx, my = xy(point)
        self._logger.info(f'Random spawn at map ({mx:.2f}, {my:.2f})')
