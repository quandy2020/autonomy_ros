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

_WALL_Z_MIN = 0.1


def _ply_to_map(xyz: np.ndarray) -> np.ndarray:
    # MP3D semantic PLY: (x, y, z) = (Habitat X, Habitat Z, height); map y = -Habitat Z = ply y.
    return np.column_stack([xyz[:, 0], xyz[:, 1], xyz[:, 2]]).astype(np.float32)


def _semantic_ply_path(cfg: Config) -> str:
    if cfg.semantic_ply_path:
        return cfg.semantic_ply_path
    return os.path.join(cfg.scene_dir(), f'{cfg.scene_id}_semantic.ply')


def map_cell_value(grid: OccupancyGrid, x: float, y: float) -> int | None:
    """OccupancyGrid cell at map (x, y), or None if outside."""
    res = grid.info.resolution
    if res <= 0.0:
        return None
    col = int((x - grid.info.origin.position.x) / res)
    row = int((y - grid.info.origin.position.y) / res)
    if col < 0 or row < 0 or col >= grid.info.width or row >= grid.info.height:
        return None
    return int(grid.data[row * grid.info.width + col])


def is_navigable_map_cell(
    grid: OccupancyGrid,
    x: float,
    y: float,
    *,
    free_threshold: int = 0,
    occupied_threshold: int = 50,
) -> bool:
    cell = map_cell_value(grid, x, y)
    if cell is None:
        return False
    return free_threshold <= cell < occupied_threshold


def _occupancy_from_navmesh(
    navmesh_path: str,
    *,
    map_resolution: float,
    floor_height: float = 0.0,
    island_radius: float = 0.5,
) -> tuple[np.ndarray, float, float]:
    """Rasterize Habitat .navmesh to ROS OccupancyGrid cells + map-frame origin."""
    import habitat_sim

    pf = habitat_sim.PathFinder()
    if not pf.load_nav_mesh(navmesh_path):
        raise ValueError(f'failed to load navmesh: {navmesh_path}')

    if abs(floor_height) < 1e-6:
        floor_height = float(pf.get_random_navigable_point()[1])

    lo, hi = pf.get_bounds()
    origin_x = float(min(lo[0], hi[0]))
    origin_z = float(min(lo[2], hi[2]))
    navigable = pf.get_topdown_view(map_resolution, floor_height, island_radius)
    h, w = navigable.shape
    cells = np.where(navigable, 0, 100).astype(np.int8)[::-1, :]
    origin_y = -(origin_z + h * map_resolution)
    return cells, origin_x, origin_y


def _occupancy_from_ply_xyz(
    xyz: np.ndarray,
    *,
    resolution: float,
    z_min: float,
    z_max: float,
) -> tuple[np.ndarray, float, float]:
    """Rasterize semantic PLY points into a 2-D occupancy grid."""
    mask = (xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max)
    pts = xyz[mask]
    if pts.shape[0] == 0:
        raise ValueError(f'No points in z range [{z_min}, {z_max}] for occupancy grid')

    floor_pts = pts[pts[:, 2] <= 1e-6]
    ref = floor_pts if floor_pts.size else pts
    ox = float(ref[:, 0].min() - 0.5 * resolution)
    oy = float(ref[:, 1].min() - 0.5 * resolution)
    w = max(1, int(np.ceil((pts[:, 0].max() - ox) / resolution)))
    h = max(1, int(np.ceil((pts[:, 1].max() - oy) / resolution)))

    gx = np.clip(((pts[:, 0] - ox) / resolution).astype(np.int32), 0, w - 1)
    gy = np.clip(((pts[:, 1] - oy) / resolution).astype(np.int32), 0, h - 1)
    cells = np.full((h, w), -1, dtype=np.int8)
    for x, y, z in zip(gx, gy, pts[:, 2]):
        if z >= _WALL_Z_MIN - 1e-6:
            cells[y, x] = 100
    for x, y, z in zip(gx, gy, pts[:, 2]):
        if z <= 1e-6:
            cells[y, x] = 0
    cells[cells < 0] = 100
    return cells, ox, oy


class PlyPublisher:
    """Load semantic PLY once; publish point cloud and occupancy grid."""

    def __init__(
        self,
        node: Node,
        cfg: Config,
        logger: Any,
        *,
        map_floor_height: float | None = None,
    ) -> None:
        self._cfg = cfg
        self._logger = logger
        self._map_floor_height = map_floor_height
        self._msgs: dict[str, object | None] = {'cloud': None, 'map': None}
        self._pubs: dict[str, object | None] = {'cloud': None, 'map': None}

        path = _semantic_ply_path(cfg)
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

    @staticmethod
    def _vertex_dtype(header_lines: list[str]) -> np.dtype:
        """Build vertex dtype from PLY header (MP3D float32 or float64)."""
        props: list[tuple[str, str]] = []
        in_vertex = False
        for line in header_lines:
            if line.startswith('element vertex'):
                in_vertex = True
                continue
            if in_vertex and line.startswith('element '):
                break
            if not in_vertex or not line.startswith('property '):
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            ptype, pname = parts[1], parts[2]
            if ptype == 'float':
                props.append((pname, '<f4'))
            elif ptype == 'double':
                props.append((pname, '<f8'))
            elif ptype == 'uchar':
                props.append((pname, 'u1'))
        if not props:
            raise ValueError('PLY header has no vertex properties')
        return np.dtype(props)

    def _read(self, path: str) -> tuple[np.ndarray, np.ndarray]:
        with open(path, 'rb') as f:
            header_lines: list[str] = []
            count = 0
            while True:
                line = f.readline().decode('ascii').strip()
                header_lines.append(line)
                if line.startswith('element vertex'):
                    count = int(line.split()[-1])
                if line == 'end_header':
                    break
            if count == 0:
                raise ValueError(f'No vertices found in PLY: {path}')

            verts = np.fromfile(f, dtype=self._vertex_dtype(header_lines), count=count)

        xyz = np.column_stack([verts['x'], verts['y'], verts['z']]).astype(np.float32)
        rgb = np.column_stack([verts['red'], verts['green'], verts['blue']]).astype(np.uint8)
        return xyz, rgb

    @staticmethod
    def _qos(rate_hz: float) -> QoSProfile:
        if rate_hz > 0.0:
            return qos_profile_system_default
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

    @property
    def map_grid(self) -> OccupancyGrid | None:
        grid = self._msgs.get('map')
        return grid if isinstance(grid, OccupancyGrid) else None

    def _grid(self, xyz: np.ndarray) -> tuple[OccupancyGrid, str]:
        cfg = self._cfg
        res = float(cfg.occupancy_grid_resolution)
        ply_path = _semantic_ply_path(cfg)
        navmesh = os.path.join(cfg.scene_dir(), f'{cfg.scene_id}.navmesh')
        floor_h = 0.0 if self._map_floor_height is None else float(self._map_floor_height)
        island = float(cfg.navmesh_eps)
        if os.path.isfile(navmesh):
            try:
                cells, ox, oy = _occupancy_from_navmesh(
                    navmesh,
                    map_resolution=res,
                    floor_height=floor_h,
                    island_radius=island,
                )
                source = navmesh
            except Exception as exc:
                self._logger.warning(
                    f'Navmesh occupancy failed ({exc}); using semantic PLY slice',
                )
                cells, ox, oy = _occupancy_from_ply_xyz(
                    xyz,
                    resolution=res,
                    z_min=float(cfg.occupancy_grid_z_min),
                    z_max=float(cfg.occupancy_grid_z_max),
                )
                source = ply_path
        else:
            cells, ox, oy = _occupancy_from_ply_xyz(
                xyz,
                resolution=res,
                z_min=float(cfg.occupancy_grid_z_min),
                z_max=float(cfg.occupancy_grid_z_max),
            )
            source = ply_path

        h, w = cells.shape
        grid = OccupancyGrid()
        grid.header.frame_id = cfg.occupancy_grid_frame
        grid.info.resolution = res
        grid.info.width = w
        grid.info.height = h
        grid.info.origin.position.x = ox
        grid.info.origin.position.y = oy
        grid.info.origin.orientation.w = 1.0
        grid.data = cells.reshape(-1).tolist()
        return grid, source

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
            grid, map_source = self._grid(xyz)
            self._msgs['map'] = grid
        except ValueError as exc:
            self._logger.error(str(exc))
            return

        rate = float(cfg.occupancy_grid_rate_hz)
        self._pubs['map'] = node.create_publisher(
            OccupancyGrid, cfg.occupancy_grid_topic, self._qos(rate)
        )
        self._logger.info(
            f'Built occupancy grid {grid.info.width}x{grid.info.height} '
            f'from {map_source} on {cfg.occupancy_grid_topic}'
        )
        if rate <= 0.0:
            self.publish_map(node.get_clock().now())
