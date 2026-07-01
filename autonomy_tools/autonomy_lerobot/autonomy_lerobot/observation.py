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

"""Assemble LeRobot dataset frames from cached ROS messages."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

from autonomy_lerobot.config import Config
from autonomy_lerobot.conversions import (
    camera_info_to_array,
    camera_info_to_intrinsic,
    cmd_vel_to_action,
    depth_to_video_rgb,
    grid_info_to_array,
    identity_extrinsic,
    image_to_numpy,
    occupancy_grid_to_array,
    odom_to_action_matrix,
    odom_to_state,
    path_to_array,
    pointcloud_to_array,
)

KEY_RGB = 'observation.images.rgb'
KEY_DEPTH = 'observation.images.depth'
KEY_SEMANTIC = 'observation.images.semantic'
KEY_STATE = 'observation.state'
KEY_CAMERA_INFO = 'observation.camera_info'
KEY_CAMERA_INTRINSIC = 'observation.camera_intrinsic'
KEY_CAMERA_EXTRINSIC = 'observation.camera_extrinsic'
KEY_MAP = 'observation.map'
KEY_MAP_INFO = 'observation.map_info'
KEY_POINTCLOUD = 'observation.pointcloud'
KEY_GLOBAL_PLAN = 'observation.global_plan'
KEY_GLOBAL_PLAN_LEN = 'observation.global_plan_len'
KEY_LOCAL_PLAN = 'observation.local_plan'
KEY_LOCAL_PLAN_LEN = 'observation.local_plan_len'
KEY_GLOBAL_COSTMAP = 'observation.global_costmap'
KEY_GLOBAL_COSTMAP_INFO = 'observation.global_costmap_info'
KEY_LOCAL_COSTMAP = 'observation.local_costmap'
KEY_LOCAL_COSTMAP_INFO = 'observation.local_costmap_info'
KEY_ACTION = 'action'
KEY_TASK = 'task'

FRAME_KEYS = (
    KEY_RGB, KEY_DEPTH, KEY_SEMANTIC, KEY_STATE, KEY_CAMERA_INFO,
    KEY_CAMERA_INTRINSIC, KEY_CAMERA_EXTRINSIC,
    KEY_MAP, KEY_MAP_INFO, KEY_POINTCLOUD,
    KEY_GLOBAL_PLAN, KEY_GLOBAL_PLAN_LEN, KEY_LOCAL_PLAN, KEY_LOCAL_PLAN_LEN,
    KEY_GLOBAL_COSTMAP, KEY_GLOBAL_COSTMAP_INFO,
    KEY_LOCAL_COSTMAP, KEY_LOCAL_COSTMAP_INFO, KEY_ACTION,
)

JDROBOT_FRAME_KEYS = (
    KEY_CAMERA_INTRINSIC,
    KEY_CAMERA_EXTRINSIC,
    KEY_ACTION,
    KEY_RGB,
    KEY_DEPTH,
)


@dataclass
class Latest:
    """Most recent ROS messages used to build a frame."""

    rgb: Image | None = None
    depth: Image | None = None
    semantic: Image | None = None
    odom: Odometry | None = None
    agent_pose: PoseStamped | None = None
    cmd_vel: Twist | None = None
    camera_info: CameraInfo | None = None
    map_grid: OccupancyGrid | None = None
    pointcloud: PointCloud2 | None = None
    global_plan: Path | None = None
    local_plan: Path | None = None
    global_costmap: OccupancyGrid | None = None
    local_costmap: OccupancyGrid | None = None

    def ready(self, cfg: Config) -> bool:
        if cfg.is_jdrobot:
            return (
                self.rgb is not None
                and self.depth is not None
                and self.camera_info is not None
                and self.odom is not None
            )
        return self.rgb is not None and self.odom is not None

    def missing_fields(self, cfg: Config) -> list[str]:
        missing: list[str] = []
        if self.rgb is None:
            missing.append('rgb')
        if cfg.is_jdrobot:
            if self.depth is None:
                missing.append('depth')
            if self.camera_info is None:
                missing.append('camera_info')
        if self.odom is None:
            missing.append('odom')
        return missing

    def clear(self) -> None:
        self.rgb = None
        self.depth = None
        self.semantic = None
        self.odom = None
        self.agent_pose = None
        self.cmd_vel = None
        self.camera_info = None
        self.map_grid = None
        self.pointcloud = None
        self.global_plan = None
        self.local_plan = None
        self.global_costmap = None
        self.local_costmap = None


def _empty_path(max_len: int) -> tuple[np.ndarray, np.ndarray]:
    return (
        np.zeros((max_len, 3), dtype=np.float32),
        np.array([0.0], dtype=np.float32),
    )


def _empty_grid(height: int, width: int) -> np.ndarray:
    return np.full((height, width, 1), -1, dtype=np.int8)


# Nav2 local costmap: 4 m window at 0.05 m/cell (nav2_habitat_params.yaml).
_DEFAULT_LOCAL_COSTMAP_CELLS = 80


def _build_habitat_frame(latest: Latest, cfg: Config) -> dict[str, Any]:
    cmd_vel = latest.cmd_vel or Twist()
    frame: dict[str, Any] = {
        KEY_RGB: image_to_numpy(latest.rgb),
        KEY_STATE: odom_to_state(latest.odom),
        KEY_ACTION: cmd_vel_to_action(cmd_vel),
        KEY_TASK: cfg.task,
    }
    if (cfg.record_depth or cfg.use_depth) and latest.depth is not None:
        frame[KEY_DEPTH] = image_to_numpy(latest.depth)
    if cfg.record_semantic and latest.semantic is not None:
        frame[KEY_SEMANTIC] = image_to_numpy(latest.semantic)
    if cfg.record_camera_info and latest.camera_info is not None:
        frame[KEY_CAMERA_INFO] = camera_info_to_array(latest.camera_info)
    if cfg.record_map and latest.map_grid is not None:
        frame[KEY_MAP] = occupancy_grid_to_array(latest.map_grid)
        frame[KEY_MAP_INFO] = grid_info_to_array(latest.map_grid)
    if cfg.record_pointcloud and latest.pointcloud is not None:
        frame[KEY_POINTCLOUD] = pointcloud_to_array(
            latest.pointcloud, cfg.max_pointcloud_points)
    if cfg.record_nav2:
        if latest.global_plan is not None:
            plan, length = path_to_array(latest.global_plan, cfg.max_path_waypoints)
        else:
            plan, length = _empty_path(cfg.max_path_waypoints)
        frame[KEY_GLOBAL_PLAN] = plan
        frame[KEY_GLOBAL_PLAN_LEN] = length

        if latest.local_plan is not None:
            plan, length = path_to_array(latest.local_plan, cfg.max_path_waypoints)
        else:
            plan, length = _empty_path(cfg.max_path_waypoints)
        frame[KEY_LOCAL_PLAN] = plan
        frame[KEY_LOCAL_PLAN_LEN] = length

        map_h = map_w = None
        if latest.map_grid is not None:
            map_h = latest.map_grid.info.height
            map_w = latest.map_grid.info.width

        if latest.global_costmap is not None:
            frame[KEY_GLOBAL_COSTMAP] = occupancy_grid_to_array(latest.global_costmap)
            frame[KEY_GLOBAL_COSTMAP_INFO] = grid_info_to_array(latest.global_costmap)
        elif map_h is not None and map_w is not None:
            frame[KEY_GLOBAL_COSTMAP] = _empty_grid(map_h, map_w)
            frame[KEY_GLOBAL_COSTMAP_INFO] = grid_info_to_array(latest.map_grid)

        if latest.local_costmap is not None:
            frame[KEY_LOCAL_COSTMAP] = occupancy_grid_to_array(latest.local_costmap)
            frame[KEY_LOCAL_COSTMAP_INFO] = grid_info_to_array(latest.local_costmap)
        else:
            cells = _DEFAULT_LOCAL_COSTMAP_CELLS
            frame[KEY_LOCAL_COSTMAP] = _empty_grid(cells, cells)
            frame[KEY_LOCAL_COSTMAP_INFO] = np.array(
                [0.05, float(cells), float(cells), 0.0, 0.0, 0.0],
                dtype=np.float32,
            )
    return frame


def _build_jdrobot_frame(latest: Latest, cfg: Config) -> dict[str, Any]:
    depth_m = image_to_numpy(latest.depth)
    return {
        KEY_CAMERA_INTRINSIC: camera_info_to_intrinsic(latest.camera_info),
        KEY_CAMERA_EXTRINSIC: identity_extrinsic(),
        KEY_ACTION: odom_to_action_matrix(latest.odom),
        KEY_RGB: image_to_numpy(latest.rgb),
        KEY_DEPTH: depth_to_video_rgb(depth_m, cfg.depth_min_m, cfg.depth_max_m),
        KEY_TASK: cfg.task,
    }


def build_frame(latest: Latest, cfg: Config) -> dict[str, Any]:
    """Build one dataset frame from the latest messages."""
    if not latest.ready(cfg):
        missing = ', '.join(latest.missing_fields(cfg))
        raise ValueError(f'required sensors missing: {missing}')

    if cfg.is_jdrobot:
        return _build_jdrobot_frame(latest, cfg)
    return _build_habitat_frame(latest, cfg)
