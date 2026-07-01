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
import sys
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


def _kujiale_module_dirs() -> list[str]:
    """Directories that may contain semantic_ply_to_kujiale.py (install or source)."""
    dirs: list[str] = []
    here = os.path.abspath(__file__)
    # .../lib/autonomy_simulator/habitat/ply.py or .../scripts/habitat/ply.py
    pkg_root = os.path.dirname(os.path.dirname(here))
    dirs.append(pkg_root)
    try:
        from ament_index_python.packages import get_package_prefix

        prefix = get_package_prefix('autonomy_simulator')
        dirs.append(os.path.join(prefix, 'lib', 'autonomy_simulator'))
        # colcon symlink-install: module lives in workspace source, not install/.
        ws = os.path.abspath(os.path.join(prefix, '..', '..'))
        src_scripts = os.path.join(
            ws, 'src', 'autonomy_ros', 'autonomy_simulator', 'scripts')
        if os.path.isfile(os.path.join(src_scripts, 'semantic_ply_to_kujiale.py')):
            dirs.append(src_scripts)
    except Exception:
        pass
    # colcon source tree: autonomy_simulator/scripts/
    for base in (pkg_root, os.path.dirname(pkg_root), os.path.dirname(os.path.dirname(here))):
        scripts = os.path.join(base, 'scripts')
        if os.path.isfile(os.path.join(scripts, 'semantic_ply_to_kujiale.py')):
            dirs.append(scripts)
    seen: set[str] = set()
    out: list[str] = []
    for d in dirs:
        d = os.path.abspath(d)
        if d not in seen:
            seen.add(d)
            out.append(d)
    return out


def _import_kujiale_converter():
    last_error: Exception | None = None
    for scripts in _kujiale_module_dirs():
        if scripts not in sys.path:
            sys.path.insert(0, scripts)
        try:
            import semantic_ply_to_kujiale as kujiale  # noqa: WPS433

            return kujiale
        except ImportError as exc:
            last_error = exc
    raise ImportError(
        'semantic_ply_to_kujiale not found; rebuild autonomy_simulator '
        f'({last_error})',
    )


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

        self._ensure_kujiale_pointcloud()
        path = self._path()
        if not os.path.isfile(path):
            self._logger.error(f'Point cloud PLY not found: {path}')
            return

        xyz, rgb = self._read(path)
        xyz = _ply_to_map(xyz)
        if cfg.semantic_pointcloud_rate_hz >= 0.0:
            self._init_cloud(node, xyz, rgb, path)
        if cfg.occupancy_grid_rate_hz >= 0.0:
            self._init_map(node, xyz, path)

    def _ensure_kujiale_pointcloud(self) -> None:
        cfg = self._cfg
        if cfg.semantic_ply_path or not cfg.kujiale_auto_convert:
            return
        try:
            kujiale = _import_kujiale_converter()
        except ImportError as exc:
            self._logger.warning(f'Kujiale converter unavailable: {exc}')
            return
        scene_dir = cfg.scene_dir()
        ref = str(cfg.kujiale_reference_ply).strip() or None
        style = kujiale.KujialeStyleConfig(
            map_resolution=float(cfg.occupancy_grid_resolution),
            map_connect_close_cells=int(cfg.kujiale_map_connect_close_cells),
            z_wall_ext_max=float(cfg.occupancy_grid_z_max),
        )
        try:
            out, info = kujiale.ensure_scene_pointcloud(
                scene_dir,
                scene_id=cfg.scene_id,
                reference_ply=ref,
                force=bool(cfg.kujiale_force_convert),
                cfg=style,
            )
        except Exception as exc:
            self._logger.error(f'Kujiale pointcloud conversion failed: {exc}')
            return
        status = info.get('status')
        if status == 'navmesh_missing':
            self._logger.warning(f'Navmesh missing in {scene_dir}; skip pointcloud build')
            return
        if status in ('generated', 'overwritten'):
            map_info = info.get('map', {})
            msg = (
                f"Kujiale pointcloud {status}: {info.get('points')} points "
                f"(floor={info.get('layers', {}).get('floor')}, "
                f"map_free={map_info.get('map_free_cells')}, "
                f"map_largest={map_info.get('map_largest_component')})"
            )
            ref_map = info.get('reference_map')
            if ref_map:
                msg += (
                    f" [ref map_free={ref_map.get('map_free_cells')}, "
                    f"largest={ref_map.get('map_largest_component')}]"
                )
            self._logger.info(msg)
        elif out is not None:
            self._logger.debug(f'Using cached kujiale pointcloud: {out}')

    def publish_cloud(self, stamp: rclpy.time.Time) -> None:
        self._send('cloud', stamp)

    def publish_map(self, stamp: rclpy.time.Time) -> None:
        self._send('map', stamp)

    def _path(self) -> str:
        cfg = self._cfg
        if cfg.semantic_ply_path:
            return cfg.semantic_ply_path
        return os.path.join(cfg.scene_dir(), 'pointcloud.ply')

    @staticmethod
    def _vertex_dtype(header_lines: list[str]) -> np.dtype:
        """Build vertex dtype from PLY header (MP3D float32 or kujiale float64)."""
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

        try:
            kujiale = _import_kujiale_converter()
            navmesh = os.path.join(
                cfg.scene_dir(), f'{cfg.scene_id}.navmesh')
            if os.path.isfile(navmesh):
                occ_grid = kujiale.load_occupancy_from_navmesh(
                    navmesh, map_resolution=float(res))
                cells, ox, oy = kujiale.occupancy_grid_to_ros(occ_grid)
            else:
                cells, ox, oy = kujiale.occupancy_grid_from_xyz(
                    xyz.astype(np.float64),
                    resolution=float(res),
                    z_min=float(z_min),
                    z_max=float(z_max),
                )
        except ImportError:
            return self._grid_legacy(xyz, z_min, z_max, res)

        h, w = cells.shape
        grid = OccupancyGrid()
        grid.header.frame_id = cfg.occupancy_grid_frame
        grid.info.resolution = float(res)
        grid.info.width = w
        grid.info.height = h
        grid.info.origin.position.x = ox
        grid.info.origin.position.y = oy
        grid.info.origin.orientation.w = 1.0
        grid.data = cells.reshape(-1).tolist()
        return grid

    def _grid_legacy(
        self,
        xyz: np.ndarray,
        z_min: float,
        z_max: float,
        res: float,
    ) -> OccupancyGrid:
        cfg = self._cfg
        # Points are in map frame; column 2 is height (PLY Z-up).
        mask = (xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max)
        pts = xyz[mask]
        if pts.shape[0] == 0:
            raise ValueError(f'No points in z range [{z_min}, {z_max}] for occupancy grid')

        floor_pts = pts[pts[:, 2] <= 1e-6]
        ref = floor_pts if floor_pts.size else pts
        ox = float(ref[:, 0].min() - 0.5 * res)
        oy = float(ref[:, 1].min() - 0.5 * res)
        w = max(1, int(np.ceil((pts[:, 0].max() - ox) / res)))
        h = max(1, int(np.ceil((pts[:, 1].max() - oy) / res)))

        gx = np.clip(((pts[:, 0] - ox) / res).astype(np.int32), 0, w - 1)
        gy = np.clip(((pts[:, 1] - oy) / res).astype(np.int32), 0, h - 1)
        flat = gy * w + gx
        cells = np.full(w * h, -1, dtype=np.int8)
        for idx, z in zip(flat, pts[:, 2]):
            if z >= 0.1 - 1e-6:
                cells[idx] = 100
        for idx, z in zip(flat, pts[:, 2]):
            if z <= 1e-6:
                cells[idx] = 0
        cells[cells < 0] = 100

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
