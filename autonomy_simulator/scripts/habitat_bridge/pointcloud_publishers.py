# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0

"""Publish MP3D semantic mesh vertices as sensor_msgs/PointCloud2."""

from __future__ import annotations

import os
from typing import Any, Optional

import numpy as np
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


def _semantic_ply_path(config: BridgeConfig) -> str:
    if config.semantic_ply_path:
        return config.semantic_ply_path
    return os.path.join(
        config.scene_directory(),
        f'{config.scene_id}_semantic.ply',
    )


def _load_ply_vertices(path: str) -> tuple[np.ndarray, np.ndarray]:
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


class SemanticPointcloudPublisher:
    """Load scene semantic PLY once and publish as PointCloud2."""

    def __init__(self, node: Node, config: BridgeConfig, logger: Any) -> None:
        self._config = config
        self._logger = logger
        self._message: Optional[PointCloud2] = None
        self._publisher: Optional[Publisher] = None

        if not config.enable_semantic_pointcloud:
            return

        ply_path = _semantic_ply_path(config)
        if not os.path.isfile(ply_path):
            logger.error(f'Semantic PLY not found: {ply_path}')
            return

        positions, colors = _load_ply_vertices(ply_path)
        stride = max(1, int(config.semantic_pointcloud_downsample))
        if stride > 1:
            positions = positions[::stride]
            colors = colors[::stride]

        header = Header()
        header.frame_id = config.semantic_pointcloud_frame
        self._message = _to_pointcloud2(header, positions, colors)

        rate_hz = float(config.semantic_pointcloud_rate_hz)
        if rate_hz <= 0.0:
            qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
        else:
            qos = qos_profile_system_default

        self._publisher = node.create_publisher(
            PointCloud2,
            config.semantic_pointcloud_topic,
            qos,
        )
        logger.info(
            f'Loaded semantic point cloud from {ply_path} '
            f'({self._message.width} points) on {config.semantic_pointcloud_topic}, '
            f'rate={rate_hz} Hz'
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
