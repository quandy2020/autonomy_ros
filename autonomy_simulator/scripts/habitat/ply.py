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

"""Load MP3D semantic PLY and publish PointCloud2 / OccupancyGrid."""

from __future__ import annotations

import os
from typing import Any

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_system_default,
)
from sensor_msgs.msg import PointCloud2, PointField

from habitat.config import Config


def _ply_to_map(xyz: np.ndarray) -> np.ndarray:
    # MP3D semantic PLY: (x, y, z) = (Habitat X, Habitat Z, height); map y = -Habitat Z = ply y.
    return np.column_stack([xyz[:, 0], xyz[:, 1], xyz[:, 2]]).astype(np.float32)


class PlyPublisher:
    """Load semantic PLY once; publish point cloud and occupancy grid."""

    def __init__(self, node: Node, cfg: Config, logger: Any) -> None:
        self._cfg = cfg
        self._logger = logger
        self._msgs: dict[str, object | None] = {'cloud': None, 'map': None}
        self._pubs: dict[str, object | None] = {'cloud': None, 'map': None}

        path = self._path()
        if not os.path.isfile(path):
            self._logger.error(f'Semantic PLY not found: {path}')
            return

        xyz, rgb = self._read(path)
        xyz = _ply_to_map(xyz)
        if cfg.semantic_pointcloud_rate_hz >= 0.0:
            self._init_cloud(node, xyz, rgb, path)
        if cfg.occupancy_grid_rate_hz >= 0.0:
            self._init_map(node, xyz, path)

    def publish_cloud(self, stamp: rclpy.time.Time) -> None:
        self._send('cloud', stamp)

    def publish_map(self, stamp: rclpy.time.Time) -> None:
        self._send('map', stamp)

    def _path(self) -> str:
        cfg = self._cfg
        if cfg.semantic_ply_path:
            return cfg.semantic_ply_path
        return os.path.join(cfg.scene_dir(), f'{cfg.scene_id}_semantic.ply')

    def _read(self, path: str) -> tuple[np.ndarray, np.ndarray]:
        with open(path, 'rb') as f:
            count = 0
            while True:
                line = f.readline().decode('ascii').strip()
                if line.startswith('element vertex'):
                    count = int(line.split()[-1])
                if line == 'end_header':
                    break
            if count == 0:
                raise ValueError(f'No vertices found in PLY: {path}')

            # MP3D semantic PLY: float32 xyz + uint8 rgb per vertex.
            dtype = np.dtype([
                ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                ('red', 'u1'), ('green', 'u1'), ('blue', 'u1'),
            ])
            verts = np.fromfile(f, dtype=dtype, count=count)

        xyz = np.column_stack([verts['x'], verts['y'], verts['z']]).astype(np.float32)
        rgb = np.column_stack([verts['red'], verts['green'], verts['blue']]).astype(np.uint8)
        return xyz, rgb

    @staticmethod
    def _qos(rate_hz: float) -> QoSProfile:
        if rate_hz > 0.0:
            return qos_profile_system_default
        # TRANSIENT_LOCAL keeps map/cloud for late RViz subscribers.
        return QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

    def _send(self, name: str, stamp: rclpy.time.Time) -> None:
        pub, msg = self._pubs[name], self._msgs[name]
        if pub is None or msg is None:
            return
        msg.header.stamp = stamp.to_msg()
        pub.publish(msg)

    def _cloud(self, frame_id: str, xyz: np.ndarray, rgb: np.ndarray) -> PointCloud2:
        n = xyz.shape[0]
        packed = (
            rgb[:, 0].astype(np.uint32) << 16
            | rgb[:, 1].astype(np.uint32) << 8
            | rgb[:, 2].astype(np.uint32)
        )
        points = np.zeros(
            n,
            dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.uint32)],
        )
        points['x'] = xyz[:, 0]
        points['y'] = xyz[:, 1]
        points['z'] = xyz[:, 2]
        points['rgb'] = packed

        cloud = PointCloud2()
        cloud.header.frame_id = frame_id
        cloud.height = 1
        cloud.width = n
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = 16 * n
        cloud.data = points.tobytes()
        cloud.is_dense = True
        return cloud

    def _grid(self, xyz: np.ndarray) -> OccupancyGrid:
        cfg = self._cfg
        z_min = cfg.occupancy_grid_z_min
        z_max = cfg.occupancy_grid_z_max
        res = cfg.occupancy_grid_resolution

        # Points are in map frame; column 2 is height (PLY Z-up).
        mask = (xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max)
        pts = xyz[mask]
        if pts.shape[0] == 0:
            raise ValueError(f'No points in z range [{z_min}, {z_max}] for occupancy grid')

        ox = float(pts[:, 0].min())
        oy = float(pts[:, 1].min())
        w = max(1, int(np.ceil((pts[:, 0].max() - ox) / res)))
        h = max(1, int(np.ceil((pts[:, 1].max() - oy) / res)))

        gx = np.clip(((pts[:, 0] - ox) / res).astype(np.int32), 0, w - 1)
        gy = np.clip(((pts[:, 1] - oy) / res).astype(np.int32), 0, h - 1)
        # ROS map: 0=free (white), 100=occupied (black), -1=unknown (gray).
        cells = np.zeros(w * h, dtype=np.int8)
        cells[np.unique(gy * w + gx)] = 100

        grid = OccupancyGrid()
        grid.header.frame_id = cfg.occupancy_grid_frame
        grid.info.resolution = float(res)
        grid.info.width = w
        grid.info.height = h
        grid.info.origin.position.x = ox
        grid.info.origin.position.y = oy
        grid.info.origin.orientation.w = 1.0
        grid.data = cells.tolist()
        return grid

    def _init_cloud(
        self,
        node: Node,
        xyz: np.ndarray,
        rgb: np.ndarray,
        path: str,
    ) -> None:
        cfg = self._cfg
        stride = max(1, int(cfg.semantic_pointcloud_downsample))
        if stride > 1:
            xyz, rgb = xyz[::stride], rgb[::stride]

        cloud = self._msgs['cloud'] = self._cloud(cfg.semantic_pointcloud_frame, xyz, rgb)
        rate = float(cfg.semantic_pointcloud_rate_hz)
        self._pubs['cloud'] = node.create_publisher(
            PointCloud2, cfg.semantic_pointcloud_topic, self._qos(rate)
        )
        self._logger.info(
            f'Loaded semantic point cloud from {path} '
            f'({cloud.width} points) on {cfg.semantic_pointcloud_topic}'
        )
        if rate <= 0.0:
            self.publish_cloud(node.get_clock().now())

    def _init_map(self, node: Node, xyz: np.ndarray, path: str) -> None:
        cfg = self._cfg
        stride = max(1, int(cfg.occupancy_grid_downsample))
        if stride > 1:
            xyz = xyz[::stride]

        try:
            grid = self._msgs['map'] = self._grid(xyz)
        except ValueError as exc:
            self._logger.error(str(exc))
            return

        rate = float(cfg.occupancy_grid_rate_hz)
        self._pubs['map'] = node.create_publisher(
            OccupancyGrid, cfg.occupancy_grid_topic, self._qos(rate)
        )
        self._logger.info(
            f'Built occupancy grid {grid.info.width}x{grid.info.height} '
            f'from {path} on {cfg.occupancy_grid_topic}'
        )
        if rate <= 0.0:
            self.publish_map(node.get_clock().now())
