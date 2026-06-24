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
from habitat_sim.agent import AgentConfiguration
from habitat_sim.sensor import CameraSensorSpec, SensorSubType, SensorType
from habitat.config import Config
from habitat.coords import from_pose, quat, ros_yaw_from_quat, to_odom_pose, xy
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
        if timed_out:
            self._linear = 0.0
            self._angular = 0.0

        x, y, _, yaw_val = self.map_pose()
        x += self._linear * math.cos(yaw_val) * dt
        y += self._linear * math.sin(yaw_val) * dt
        yaw_val += self._angular * dt
        self._move_map(x, y, yaw_val)
        return self._sim.get_sensor_observations()

    def agent_pose(self, frame_id: str, stamp: rclpy.time.Time) -> PoseStamped:
        x, y, z, yaw_val = self.map_pose()
        return to_odom_pose(x, y, z, yaw_val, frame_id, stamp)

    def map_pose(self) -> tuple[float, float, float, float]:
        state = self._agent.get_state()
        map_x, map_y = xy(state.position)
        return map_x, map_y, self._cfg.base_link_height, ros_yaw_from_quat(state.rotation)

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

        return specs

    def _open(self) -> habitat_sim.Simulator:
        self._assets()
        cfg = self._cfg

        sim_cfg = habitat_sim.SimulatorConfiguration()
        sim_cfg.scene_dataset_config_file = cfg.dataset_config()
        sim_cfg.scene_id = cfg.scene_asset()
        sim_cfg.enable_physics = False
        sim_cfg.allow_sliding = True
        sim_cfg.requires_textures = True
        sim_cfg.load_semantic_mesh = True

        agent_cfg = AgentConfiguration()
        agent_cfg.sensor_specifications = self._sensors()

        self._logger.info(
            f'Loading MP3D scene {cfg.scene_asset()} from {cfg.mp3d_root}'
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
