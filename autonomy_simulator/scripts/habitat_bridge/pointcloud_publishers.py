# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0

"""Publish MP3D semantic mesh as PointCloud2 and OccupancyGrid."""

from __future__ import annotations

import os
from typing import Any, Optional, Tuple

import numpy as np
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_system_default,
)
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from habitat_bridge.bridge_config import BridgeConfig

# nav_msgs/OccupancyGrid trinary encoding (matches map_server PGM trinary mode).
_OCC_UNKNOWN = -1
_OCC_FREE = 0
_OCC_OCCUPIED = 100

def _semantic_ply_path(config: BridgeConfig) -> str:
    if config.semantic_ply_path:
        return config.semantic_ply_path
    return os.path.join(
        config.scene_directory(),
        f'{config.scene_id}_semantic.ply',
    )


def _load_ply_vertices(path: str) -> Tuple[np.ndarray, np.ndarray]:
    """Load vertex xyz and rgb from an MP3D binary semantic PLY."""
    with open(path, 'rb') as file:
        vertex_count = 0
        while True:
            line = file.readline().decode('ascii').strip()
            if line.startswith('element vertex'):
                vertex_count = int(line.split()[-1])
            if line == 'end_header':
                break

        if vertex_count == 0:
            raise ValueError(f'No vertices found in PLY: {path}')

        dtype = np.dtype([
            ('x', '<f4'),
            ('y', '<f4'),
            ('z', '<f4'),
            ('red', 'u1'),
            ('green', 'u1'),
            ('blue', 'u1'),
        ])
        vertices = np.fromfile(file, dtype=dtype, count=vertex_count)

    # MP3D semantic PLY is Z-up on disk (matches map frame: XY floor, Z height).
    positions = np.column_stack(
        [vertices['x'], vertices['y'], vertices['z']]
    ).astype(np.float32)
    colors = np.column_stack(
        [vertices['red'], vertices['green'], vertices['blue']]
    ).astype(np.uint8)
    return positions, colors


def _load_semantic_scene(
    config: BridgeConfig,
    logger: Any,
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """Load semantic PLY vertices and colors once for cloud + map publishers."""
    ply_path = _semantic_ply_path(config)
    if not os.path.isfile(ply_path):
        logger.error(f'Semantic PLY not found: {ply_path}')
        return None

    positions, colors = _load_ply_vertices(ply_path)
    logger.info(
        f'Loaded semantic scene from {ply_path} ({positions.shape[0]} points)'
    )
    return positions, colors


def _downsample_positions(
    positions: np.ndarray,
    colors: Optional[np.ndarray],
    stride: int,
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    stride = max(1, int(stride))
    if stride <= 1:
        return positions, colors
    positions_ds = positions[::stride]
    colors_ds = colors[::stride] if colors is not None else None
    return positions_ds, colors_ds


def _to_pointcloud2(
    header: Header,
    positions: np.ndarray,
    colors: np.ndarray,
) -> PointCloud2:
    count = positions.shape[0]
    rgb_packed = (
        colors[:, 0].astype(np.uint32) << 16
        | colors[:, 1].astype(np.uint32) << 8
        | colors[:, 2].astype(np.uint32)
    )
    points = np.zeros(
        count,
        dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.uint32),
        ],
    )
    points['x'] = positions[:, 0]
    points['y'] = positions[:, 1]
    points['z'] = positions[:, 2]
    points['rgb'] = rgb_packed

    message = PointCloud2()
    message.header = header
    message.height = 1
    message.width = count
    message.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 16
    message.row_step = 16 * count
    message.data = points.tobytes()
    message.is_dense = True
    return message


def _grid_bounds_xy(
    positions: np.ndarray,
    padding: float,
) -> tuple[float, float, float, float]:
    """Tight XY envelope from obstacle-height points plus padding."""
    min_x = float(positions[:, 0].min())
    min_y = float(positions[:, 1].min())
    max_x = float(positions[:, 0].max())
    max_y = float(positions[:, 1].max())
    return min_x - padding, min_y - padding, max_x + padding, max_y + padding


def _dilate_mask(mask: np.ndarray, radius: int) -> np.ndarray:
    """Morphological dilation on a 2-D boolean mask."""
    if radius <= 0:
        return mask
    out = mask.copy()
    height, width = mask.shape
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx == 0 and dy == 0:
                continue
            y0 = max(0, dy)
            y1 = min(height, height + dy)
            x0 = max(0, dx)
            x1 = min(width, width + dx)
            sy0 = max(0, -dy)
            sx0 = max(0, -dx)
            sy1 = sy0 + (y1 - y0)
            sx1 = sx0 + (x1 - x0)
            out[y0:y1, x0:x1] |= mask[sy0:sy1, sx0:sx1]
    return out


def _project_points_to_grid(
    positions: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    width: int,
    height: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Project XY points to in-bounds grid indices (pcd2pgm style)."""
    grid_x = ((positions[:, 0] - origin_x) / resolution).astype(np.int32)
    grid_y = ((positions[:, 1] - origin_y) / resolution).astype(np.int32)
    valid = (
        (grid_x >= 0)
        & (grid_x < width)
        & (grid_y >= 0)
        & (grid_y < height)
    )
    return grid_x[valid], grid_y[valid]


def _make_occupancy_grid_message(
    frame_id: str,
    resolution: float,
    width: int,
    height: int,
    origin_x: float,
    origin_y: float,
    data: np.ndarray,
) -> OccupancyGrid:
    message = OccupancyGrid()
    message.header.frame_id = frame_id
    message.info.resolution = float(resolution)
    message.info.width = int(width)
    message.info.height = int(height)
    message.info.origin.position.x = origin_x
    message.info.origin.position.y = origin_y
    message.info.origin.position.z = 0.0
    message.info.origin.orientation.w = 1.0
    message.data = data.astype(np.int8).tolist()
    return message


def _build_occupancy_grid(
    positions: np.ndarray,
    config: BridgeConfig,
) -> OccupancyGrid:
    """pcd2pgm Z projection: height band -> occupied, else column hit -> free."""
    z_min = config.occupancy_grid_z_min
    z_max = config.occupancy_grid_z_max
    in_band = (positions[:, 2] >= z_min) & (positions[:, 2] <= z_max)
    wall_points = positions[in_band]
    if wall_points.shape[0] == 0:
        raise ValueError('No points in occupancy_grid_z range for occupancy grid')

    min_x, min_y, max_x, max_y = _grid_bounds_xy(
        wall_points, config.occupancy_grid_padding
    )
    resolution = config.occupancy_grid_resolution
    width = max(1, int((max_x - min_x) / resolution) + 1)
    height = max(1, int((max_y - min_y) / resolution) + 1)

    has_point = np.zeros((height, width), dtype=bool)
    all_x, all_y = _project_points_to_grid(
        positions, min_x, min_y, resolution, width, height
    )
    if all_x.size > 0:
        has_point[all_y, all_x] = True

    wall_cols = np.zeros((height, width), dtype=bool)
    wx, wy = _project_points_to_grid(
        wall_points, min_x, min_y, resolution, width, height
    )
    if wx.size > 0:
        wall_cols[wy, wx] = True

    occupied = wall_cols
    if config.occupancy_grid_wall_dilate > 0:
        occupied = _dilate_mask(occupied, config.occupancy_grid_wall_dilate)

    grid = np.full((height, width), _OCC_UNKNOWN, dtype=np.int8)
    grid[occupied] = _OCC_OCCUPIED
    grid[has_point & ~wall_cols & ~occupied] = _OCC_FREE

    return _make_occupancy_grid_message(
        config.occupancy_grid_frame,
        resolution,
        width,
        height,
        min_x,
        min_y,
        grid.ravel(),
    )


def _count_grid_values(data: np.ndarray) -> tuple[int, int, int]:
    free = int(np.sum(data == _OCC_FREE))
    occupied = int(np.sum(data == _OCC_OCCUPIED))
    unknown = int(np.sum(data == _OCC_UNKNOWN))
    return free, occupied, unknown


def _map_qos(rate_hz: float) -> QoSProfile:
    if rate_hz <= 0.0:
        return QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
    return qos_profile_system_default


class SemanticPointcloudPublisher:
    """Load scene semantic PLY once and publish as PointCloud2."""

    def __init__(
        self,
        node: Node,
        config: BridgeConfig,
        logger: Any,
        positions: Optional[np.ndarray] = None,
        colors: Optional[np.ndarray] = None,
    ) -> None:
        self._config = config
        self._logger = logger
        self._message: Optional[PointCloud2] = None
        self._publisher: Optional[Publisher] = None

        if not config.enable_semantic_pointcloud:
            return

        if positions is None or colors is None:
            scene = _load_semantic_scene(config, logger)
            if scene is None:
                return
            positions, colors = scene

        positions, colors = _downsample_positions(
            positions,
            colors,
            config.semantic_pointcloud_downsample,
        )

        header = Header()
        header.frame_id = config.semantic_pointcloud_frame
        self._message = _to_pointcloud2(header, positions, colors)

        rate_hz = float(config.semantic_pointcloud_rate_hz)
        self._publisher = node.create_publisher(
            PointCloud2,
            config.semantic_pointcloud_topic,
            _map_qos(rate_hz),
        )
        logger.info(
            f'Publishing semantic point cloud ({self._message.width} points) on '
            f'{config.semantic_pointcloud_topic}, rate={rate_hz} Hz'
        )
        if rate_hz <= 0.0:
            self.publish(node.get_clock().now())

    def publish(self, stamp: object) -> None:
        if self._publisher is None or self._message is None:
            return
        self._message.header.stamp = (
            stamp.to_msg() if hasattr(stamp, 'to_msg') else stamp
        )
        self._publisher.publish(self._message)


class SemanticOccupancyGridPublisher:
    """Build a 2D occupancy grid from semantic PLY and publish periodically."""

    def __init__(
        self,
        node: Node,
        config: BridgeConfig,
        logger: Any,
        positions: Optional[np.ndarray] = None,
    ) -> None:
        self._config = config
        self._message: Optional[OccupancyGrid] = None
        self._publisher: Optional[Publisher] = None

        if not config.enable_semantic_occupancy_grid:
            return

        if positions is None:
            scene = _load_semantic_scene(config, logger)
            if scene is None:
                return
            positions, _colors = scene

        positions, _ = _downsample_positions(
            positions,
            None,
            config.occupancy_grid_downsample,
        )

        try:
            self._message = _build_occupancy_grid(positions, config)
        except ValueError as exc:
            logger.error(str(exc))
            return

        rate_hz = float(config.occupancy_grid_rate_hz)
        self._publisher = node.create_publisher(
            OccupancyGrid,
            config.occupancy_grid_topic,
            _map_qos(rate_hz),
        )
        grid_data = np.asarray(self._message.data, dtype=np.int8)
        free, occupied, unknown = _count_grid_values(grid_data)
        origin = self._message.info.origin.position
        extent_x = self._message.info.width * self._message.info.resolution
        extent_y = self._message.info.height * self._message.info.resolution
        logger.info(
            f'Built occupancy grid {self._message.info.width}x'
            f'{self._message.info.height} '
            f'@ {config.occupancy_grid_resolution} m/cell (z_projection) '
            f'origin=({origin.x:.2f}, {origin.y:.2f}) '
            f'extent=({extent_x:.2f}, {extent_y:.2f}) m '
            f'free={free} occupied={occupied} unknown={unknown} on '
            f'{config.occupancy_grid_topic}, rate={rate_hz} Hz'
        )
        if rate_hz <= 0.0:
            self.publish(node.get_clock().now())

    def publish(self, stamp: object) -> None:
        if self._publisher is None or self._message is None:
            return
        self._message.header.stamp = (
            stamp.to_msg() if hasattr(stamp, 'to_msg') else stamp
        )
        self._publisher.publish(self._message)
