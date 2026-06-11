# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0

"""Habitat-Sim lifecycle: scene loading, agent pose, and sensor observations."""

from __future__ import annotations

import math
import os
import shutil
from typing import Any, Dict, List

import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, Quaternion

from habitat_bridge.bridge_config import BridgeConfig

try:
    import habitat_sim
    from habitat_sim.agent import AgentConfiguration
    from habitat_sim.sensor import CameraSensorSpec, SensorType
    from habitat_sim.utils.common import quat_from_angle_axis, quat_rotate_vector

    HABITAT_AVAILABLE = True
except ImportError as exc:  # pragma: no cover - optional dependency
    HABITAT_AVAILABLE = False
    _IMPORT_ERROR = exc
    quat_from_angle_axis = None  # type: ignore[assignment,misc]
    quat_rotate_vector = None  # type: ignore[assignment,misc]

_HABITAT_TO_ROS = np.array(
    [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]],
    dtype=np.float64,
)


def _require_habitat_sim() -> None:
    if not HABITAT_AVAILABLE:
        raise RuntimeError(f'habitat-sim is not installed: {_IMPORT_ERROR}')


def _yaw_from_habitat_quaternion(quaternion: np.quaternion) -> float:
    rotation = np.array(
        [
            [
                1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2),
                2.0 * (quaternion.x * quaternion.y - quaternion.z * quaternion.w),
                2.0 * (quaternion.x * quaternion.z + quaternion.y * quaternion.w),
            ],
            [
                2.0 * (quaternion.x * quaternion.y + quaternion.z * quaternion.w),
                1.0 - 2.0 * (quaternion.x ** 2 + quaternion.z ** 2),
                2.0 * (quaternion.y * quaternion.z - quaternion.x * quaternion.w),
            ],
            [
                2.0 * (quaternion.x * quaternion.z - quaternion.y * quaternion.w),
                2.0 * (quaternion.y * quaternion.z + quaternion.x * quaternion.w),
                1.0 - 2.0 * (quaternion.x ** 2 + quaternion.y ** 2),
            ],
        ]
    )
    return math.atan2(float(rotation[0, 2]), float(-rotation[2, 2]))


def _yaw_to_ros_quaternion(yaw: float) -> Quaternion:
    half_yaw = yaw * 0.5
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(half_yaw),
        w=math.cos(half_yaw),
    )


def _ros_yaw_from_quaternion(quaternion: Quaternion) -> float:
    return math.atan2(
        2.0
        * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0
        - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _ros_pose_to_habitat(
    pose: Pose,
    sensor_height: float,
) -> tuple[np.ndarray, np.quaternion]:
    height = float(pose.position.z) if pose.position.z != 0.0 else sensor_height
    position = np.array(
        [pose.position.x, height, pose.position.y],
        dtype=np.float32,
    )
    yaw = _ros_yaw_from_quaternion(pose.orientation)
    rotation = quat_from_angle_axis(yaw, np.array([0.0, 1.0, 0.0], dtype=np.float32))
    return position, rotation


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _ground_pose_ros(
    position: np.ndarray,
    rotation: np.quaternion,
) -> tuple[float, float, float, float]:
    """Map Habitat agent pose to ROS map/odom ground frame (x, y, z, yaw)."""
    position_ros = _HABITAT_TO_ROS @ np.asarray(position, dtype=np.float64)
    yaw = _yaw_from_habitat_quaternion(rotation)
    return (
        float(position_ros[0]),
        float(position_ros[1]),
        float(position_ros[2]),
        yaw,
    )


def _habitat_state_to_pose_stamped(
    position: np.ndarray,
    rotation: np.quaternion,
    frame_id: str,
    stamp: object,
) -> PoseStamped:
    message = PoseStamped()
    message.header.frame_id = frame_id
    message.header.stamp = stamp.to_msg() if hasattr(stamp, 'to_msg') else stamp
    position_ros = _HABITAT_TO_ROS @ np.asarray(position, dtype=np.float64)
    message.pose.position.x = float(position_ros[0])
    message.pose.position.y = float(position_ros[1])
    message.pose.position.z = float(position_ros[2])
    message.pose.orientation = _yaw_to_ros_quaternion(
        _yaw_from_habitat_quaternion(rotation)
    )
    return message


def _ensure_scene_assets(config: BridgeConfig, logger: Any) -> None:
    scene_dir = config.scene_directory()
    glb_path = os.path.join(scene_dir, f'{config.scene_id}.glb')
    if not os.path.isfile(glb_path):
        raise FileNotFoundError(f'MP3D scene glb not found: {glb_path}')

    dataset_config = config.resolved_scene_dataset_config()
    if os.path.isfile(dataset_config):
        return

    if not config.package_scene_dataset_config:
        raise FileNotFoundError(
            f'Scene dataset config not found: {dataset_config}'
        )
    os.makedirs(config.mp3d_root, exist_ok=True)
    shutil.copy2(config.package_scene_dataset_config, dataset_config)
    logger.info(f'Installed MP3D scene dataset config to {dataset_config}')


def _build_sensor_specs(config: BridgeConfig) -> List[CameraSensorSpec]:
    height = config.image_height
    width = config.image_width
    sensor_height = config.sensor_height
    sensor_specs: List[CameraSensorSpec] = []

    if config.enable_rgb or config.enable_semantic_colored:
        rgb_spec = CameraSensorSpec()
        rgb_spec.uuid = 'rgb'
        rgb_spec.sensor_type = SensorType.COLOR
        rgb_spec.resolution = [height, width]
        rgb_spec.position = [0.0, sensor_height, 0.0]
        sensor_specs.append(rgb_spec)

    if config.enable_depth:
        depth_spec = CameraSensorSpec()
        depth_spec.uuid = 'depth'
        depth_spec.sensor_type = SensorType.DEPTH
        depth_spec.resolution = [height, width]
        depth_spec.position = [0.0, sensor_height, 0.0]
        sensor_specs.append(depth_spec)

    if config.enable_semantic or config.enable_semantic_colored:
        semantic_spec = CameraSensorSpec()
        semantic_spec.uuid = 'semantic'
        semantic_spec.sensor_type = SensorType.SEMANTIC
        semantic_spec.resolution = [height, width]
        semantic_spec.position = [0.0, sensor_height, 0.0]
        sensor_specs.append(semantic_spec)

    return sensor_specs


def _create_simulator(config: BridgeConfig, logger: Any) -> habitat_sim.Simulator:
    _ensure_scene_assets(config, logger)

    sim_config = habitat_sim.SimulatorConfiguration()
    sim_config.scene_dataset_config_file = config.resolved_scene_dataset_config()
    sim_config.scene_id = config.resolved_scene_asset()
    sim_config.enable_physics = False
    sim_config.allow_sliding = True
    sim_config.requires_textures = True
    sim_config.load_semantic_mesh = True

    agent_config = AgentConfiguration()
    agent_config.sensor_specifications = _build_sensor_specs(config)

    logger.info(
        f'Loading MP3D scene {config.resolved_scene_asset()} '
        f'from {config.mp3d_root}'
    )
    return habitat_sim.Simulator(
        habitat_sim.Configuration(sim_config, [agent_config])
    )


class HabitatSession:
    """Owns a Habitat-Sim instance and exposes agent pose plus sensor observations."""

    def __init__(self, config: BridgeConfig, logger: Any) -> None:
        _require_habitat_sim()
        self._config = config
        self._logger = logger
        self._linear_cmd = 0.0
        self._angular_cmd = 0.0
        self._odom_origin = (0.0, 0.0, 0.0, 0.0)
        self._sim = _create_simulator(config, logger)
        self._agent = self._sim.get_agent(0)
        self._spawn_on_navmesh()
        self._record_odom_origin()

    def close(self) -> None:
        if hasattr(self, '_sim'):
            self._sim.close()

    def set_velocity_command(self, linear: float, angular: float) -> None:
        self._linear_cmd = linear
        self._angular_cmd = angular

    def active_velocity(self) -> tuple[float, float]:
        return self._linear_cmd, self._angular_cmd

    def step(self, dt: float, cmd_timed_out: bool) -> Dict[str, Any]:
        linear = self._linear_cmd
        angular = self._angular_cmd
        if cmd_timed_out:
            linear = 0.0
            angular = 0.0
        self._integrate_velocity(linear, angular, dt)
        return self._sim.get_sensor_observations()

    def get_observations(self) -> Dict[str, Any]:
        return self._sim.get_sensor_observations()

    def map_to_odom_pose(self) -> tuple[float, float, float, float]:
        return self._odom_origin

    def odom_to_base_footprint_pose(self) -> tuple[float, float, float, float]:
        state = self.agent_state()
        current = _ground_pose_ros(state.position, state.rotation)
        origin = self._odom_origin
        cos_yaw = math.cos(origin[3])
        sin_yaw = math.sin(origin[3])
        delta_x = current[0] - origin[0]
        delta_y = current[1] - origin[1]
        x_odom = cos_yaw * delta_x + sin_yaw * delta_y
        y_odom = -sin_yaw * delta_x + cos_yaw * delta_y
        z_odom = current[2] - origin[2]
        return x_odom, y_odom, z_odom, _normalize_angle(current[3] - origin[3])

    def agent_state(self) -> Any:
        """Return the current Habitat agent state."""
        return self._agent.get_state()

    def agent_pose_stamped(self, frame_id: str, stamp: object) -> PoseStamped:
        state = self.agent_state()
        return _habitat_state_to_pose_stamped(
            state.position,
            state.rotation,
            frame_id,
            stamp,
        )

    def set_agent_pose(self, pose: PoseStamped) -> None:
        position, rotation = _ros_pose_to_habitat(
            pose.pose,
            self._config.sensor_height,
        )
        state = self._agent.get_state()
        state.position = position
        state.rotation = rotation
        self._agent.set_state(state, reset_sensors=True)
        self._record_odom_origin()

    def _record_odom_origin(self) -> None:
        state = self.agent_state()
        self._odom_origin = _ground_pose_ros(state.position, state.rotation)

    def _integrate_velocity(self, linear: float, angular: float, dt: float) -> None:
        state = self._agent.get_state()
        yaw_axis = np.array([0.0, 1.0, 0.0], dtype=np.float32)
        forward = quat_rotate_vector(
            state.rotation,
            np.array([0.0, 0.0, -1.0], dtype=np.float32),
        )
        state.position = state.position + forward * linear * dt
        # Body-frame yaw rate (ROS twist.angular.z) is applied in the agent frame.
        state.rotation = state.rotation * quat_from_angle_axis(
            angular * dt, yaw_axis
        )
        self._agent.set_state(state, reset_sensors=False)

    def _spawn_on_navmesh(self) -> None:
        if not self._sim.pathfinder.is_loaded:
            self._logger.warning(
                'Navmesh not loaded; using default agent spawn pose'
            )
            return

        sample = self._sim.pathfinder.get_random_navigable_point()
        state = self._agent.get_state()
        state.position = sample
        state.rotation = quat_from_angle_axis(0.0, np.array([0.0, 1.0, 0.0]))
        self._agent.set_state(state, reset_sensors=True)
