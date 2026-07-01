#!/usr/bin/env python3
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

"""Build kujiale-style pointcloud.ply from a 2-D occupancy grid (no semantic PLY).

Reference layout (kujiale_0003/pointcloud.ply, ~250k points):
  - floor  (distance-field RGB) @ z=0      on 2 cm cells (~23.6k)
  - low    (0, 0, 128)        @ z in (0, 0.1] on 5 cm cells (~41.9k)
  - wall   (102, 102, 102)    @ z in [0.1, 0.55] on 2 cm cells (~185k)
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import numpy as np

COLOR_FREE = np.array([128, 255, 128], dtype=np.uint8)
COLOR_NEAR = np.array([0, 0, 128], dtype=np.uint8)
COLOR_LOW = COLOR_NEAR
COLOR_WALL = np.array([102, 102, 102], dtype=np.uint8)

Z_WALL_MIN = 0.100003
Z_LOW_MAX = 0.1
Z_WALL_MAX = 0.45
Z_WALL_EXT_MAX = 0.55
FLOOR_RES = 0.02
LOW_RES = 0.05
WALL_RES = 0.02
MAP_RES = 0.05
DIST_NEAR_M = 0.2
DIST_FAR_M = 0.8
DEFAULT_TARGET_POINTS = 250_644
KUJIALE_LAYER_COUNTS = (23_572, 41_852, 185_220)
KUJIALE_WALL_MAIN = 142_543
KUJIALE_WALL_EXT = 42_677

OUTPUT_DTYPE = np.dtype([
    ('x', '<f8'), ('y', '<f8'), ('z', '<f8'),
    ('red', 'u1'), ('green', 'u1'), ('blue', 'u1'),
])

_LAYER_SHARES = tuple(c / sum(KUJIALE_LAYER_COUNTS) for c in KUJIALE_LAYER_COUNTS)


@dataclass(frozen=True)
class KujialeStyleConfig:
    map_resolution: float = MAP_RES
    floor_resolution: float = FLOOR_RES
    low_resolution: float = LOW_RES
    wall_resolution: float = WALL_RES
    z_low_max: float = Z_LOW_MAX
    z_wall_min: float = Z_WALL_MIN
    z_wall_max: float = Z_WALL_MAX
    z_wall_ext_max: float = Z_WALL_EXT_MAX
    encode_distance_field: bool = True
    dist_near_m: float = DIST_NEAR_M
    dist_far_m: float = DIST_FAR_M
    map_connect_close_cells: int = 0
    wall_column_max: int = 8
    wall_extension_max: int = 4
    low_column_max: int = 10
    target_points: int | None = DEFAULT_TARGET_POINTS
    random_seed: int = 0


@dataclass(frozen=True)
class OccupancyGrid2D:
    """2-D occupancy map: 0 = free, 100 = occupied."""

    cells: np.ndarray
    origin_x: float
    origin_y: float
    resolution: float
    origin_z: float = 0.0  # Habitat Z at navigable row 0 (before vertical flip).

    @property
    def height(self) -> int:
        return int(self.cells.shape[0])

    @property
    def width(self) -> int:
        return int(self.cells.shape[1])

    def cell_center(self, col: int, row: int) -> tuple[float, float]:
        cx = self.origin_x + (col + 0.5) * self.resolution
        cy = self.origin_y + (row + 0.5) * self.resolution
        return cx, cy


def load_occupancy_from_navmesh(
    navmesh_path: str | Path,
    *,
    map_resolution: float = MAP_RES,
    floor_height: float = 0.0,
    island_radius: float = 0.5,
) -> OccupancyGrid2D:
    """Rasterize a Habitat .navmesh into a 2-D occupancy grid (free / occupied)."""
    try:
        import habitat_sim
    except ImportError as exc:
        raise ImportError(
            'habitat_sim is required to load .navmesh files',
        ) from exc

    navmesh_path = Path(navmesh_path)
    pf = habitat_sim.PathFinder()
    if not pf.load_nav_mesh(str(navmesh_path)):
        raise ValueError(f'failed to load navmesh: {navmesh_path}')

    lo, hi = pf.get_bounds()
    origin_x = float(min(lo[0], hi[0]))
    origin_z = float(min(lo[2], hi[2]))
    navigable = pf.get_topdown_view(map_resolution, floor_height, island_radius)
    h, w = navigable.shape
    # Habitat row 0 = min habitat Z; map y = -habitat_z (see habitat/navmesh.py).
    # Flip vertically so internal row index grows with map +Y (ROS OccupancyGrid).
    cells = np.where(navigable, 0, 100).astype(np.int8)[::-1, :]
    origin_y = -(origin_z + h * map_resolution)
    return OccupancyGrid2D(cells, origin_x, origin_y, map_resolution, origin_z)


def load_occupancy_from_array(
    cells: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
) -> OccupancyGrid2D:
    grid = np.asarray(cells, dtype=np.int8)
    if grid.ndim != 2:
        raise ValueError('cells must be a 2-D array')
    return OccupancyGrid2D(grid, float(origin_x), float(origin_y), float(resolution))


def _dilate(keys: set[tuple[int, int]], radius: int) -> set[tuple[int, int]]:
    if radius <= 0 or not keys:
        return set(keys)
    out = set(keys)
    for c, r in keys:
        for dc in range(-radius, radius + 1):
            for dr in range(-radius, radius + 1):
                out.add((c + dc, r + dr))
    return out


def _erode(
    keys: set[tuple[int, int]],
    radius: int,
    bounds: tuple[int, int, int, int],
) -> set[tuple[int, int]]:
    c0, c1, r0, r1 = bounds
    if radius <= 0 or not keys:
        return set(keys)
    kept: set[tuple[int, int]] = set()
    for c, r in keys:
        ok = True
        for dc in range(-radius, radius + 1):
            for dr in range(-radius, radius + 1):
                if (c + dc, r + dr) not in keys:
                    ok = False
                    break
            if not ok:
                break
        if ok and c0 <= c <= c1 and r0 <= r <= r1:
            kept.add((c, r))
    return kept


def _close_free(
    free: set[tuple[int, int]],
    occ: set[tuple[int, int]],
    bounds: tuple[int, int, int, int],
    radius: int,
) -> set[tuple[int, int]]:
    if radius <= 0:
        return free
    expanded = _dilate(free, radius) - occ
    return _erode(expanded, radius, bounds)


def _occ_components(occ: set[tuple[int, int]]) -> list[set[tuple[int, int]]]:
    remaining = set(occ)
    comps: list[set[tuple[int, int]]] = []
    while remaining:
        seed = remaining.pop()
        comp = {seed}
        stack = [seed]
        while stack:
            c, r = stack.pop()
            for dc, dr in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                nb = (c + dc, r + dr)
                if nb in remaining:
                    remaining.remove(nb)
                    comp.add(nb)
                    stack.append(nb)
        comps.append(comp)
    return comps


def _filter_occ_speckle(
    occ: set[tuple[int, int]],
    *,
    min_cells: int = 2,
) -> set[tuple[int, int]]:
    """Drop tiny occupied islands (navmesh raster speckle)."""
    if min_cells <= 1:
        return occ
    kept: set[tuple[int, int]] = set()
    for comp in _occ_components(occ):
        if len(comp) >= min_cells:
            kept |= comp
    return kept


def _occ_adjacent_to_free(
    occ: set[tuple[int, int]],
    free: set[tuple[int, int]],
) -> set[tuple[int, int]]:
    """Occupied cells that border free space (actual walls, not bbox padding)."""
    walls: set[tuple[int, int]] = set()
    for c, r in occ:
        for dc, dr in ((0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            if (c + dc, r + dr) in free:
                walls.add((c, r))
                break
    return walls


def _map_keys(mask: np.ndarray) -> set[tuple[int, int]]:
    rows, cols = np.nonzero(mask)
    return {(int(c), int(r)) for r, c in zip(rows, cols)}


def _distance_field(
    occ: set[tuple[int, int]],
    bounds: tuple[int, int, int, int],
    resolution: float,
) -> dict[tuple[int, int], float]:
    c0, c1, r0, r1 = bounds
    w = c1 - c0 + 1
    h = r1 - r0 + 1
    wall = np.zeros((h, w), dtype=bool)
    for c, r in occ:
        wall[r - r0, c - c0] = True
    try:
        from scipy.ndimage import distance_transform_edt
        dist = distance_transform_edt(~wall) * resolution
    except ImportError:
        dist = np.full((h, w), np.inf, dtype=np.float64)
        q: list[tuple[int, int]] = []
        for c, r in occ:
            dist[r - r0, c - c0] = 0.0
            q.append((r - r0, c - c0))
        head = 0
        while head < len(q):
            ri, ci = q[head]
            head += 1
            nd = dist[ri, ci] + resolution
            for dri, dci in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                nr, nc = ri + dri, ci + dci
                if 0 <= nr < h and 0 <= nc < w and dist[nr, nc] > nd:
                    dist[nr, nc] = nd
                    q.append((nr, nc))
    out: dict[tuple[int, int], float] = {}
    for ri in range(h):
        for ci in range(w):
            out[(ci + c0, ri + r0)] = float(dist[ri, ci])
    return out


def _encode_distance_rgb(dist_m: float, cfg: KujialeStyleConfig) -> np.ndarray:
    t0 = cfg.dist_near_m
    t1 = max(cfg.dist_far_m, t0 + 1e-6)
    if dist_m <= t0:
        return COLOR_NEAR
    if dist_m >= t1:
        return COLOR_FREE
    t = (dist_m - t0) / (t1 - t0)
    rgb = np.array([round(t * 128.0), round(t * 255.0), 128.0], dtype=np.float64)
    return np.clip(rgb, 0.0, 255.0).astype(np.uint8)


def _layer_targets(cfg: KujialeStyleConfig) -> tuple[int, int, int]:
    total = cfg.target_points or sum(KUJIALE_LAYER_COUNTS)
    return tuple(max(1, int(round(total * s))) for s in _LAYER_SHARES)


def _layer_label(xyz: np.ndarray, rgb: np.ndarray) -> np.ndarray:
    labels = np.full(len(rgb), 2, dtype=np.int8)
    z = xyz[:, 2]
    is_wall = np.all(rgb == COLOR_WALL, axis=1)
    labels[is_wall] = 2
    labels[(z <= 1e-9) & ~is_wall] = 0
    labels[(z > 1e-9) & (z < Z_WALL_MIN) & ~is_wall] = 1
    return labels


def _sample_indices(
    idx: np.ndarray,
    n_keep: int,
    rng: np.random.Generator,
) -> np.ndarray:
    if idx.size == 0 or n_keep <= 0:
        return idx[:0]
    if idx.size <= n_keep:
        return idx
    return rng.choice(idx, size=n_keep, replace=False)


def _pack_layers(
    xyz: np.ndarray,
    rgb: np.ndarray,
    cfg: KujialeStyleConfig,
    *,
    floor_keep: int | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    if cfg.target_points is None:
        return xyz, rgb
    rng = np.random.default_rng(cfg.random_seed)
    labels = _layer_label(xyz, rgb)
    _, low_t, _ = _layer_targets(cfg)
    keep: list[np.ndarray] = []
    floor_idx = np.flatnonzero(labels == 0)
    if floor_idx.size:
        # Keep every free-cell floor point so /map free space is not eroded.
        n_floor = floor_idx.size if floor_keep is None else min(floor_idx.size, floor_keep)
        keep.append(_sample_indices(floor_idx, n_floor, rng))
    low_idx = np.flatnonzero(labels == 1)
    if low_idx.size:
        keep.append(_sample_indices(low_idx, low_t, rng))
    wall_idx = np.flatnonzero(labels == 2)
    if wall_idx.size:
        z = xyz[wall_idx, 2]
        ext = wall_idx[z > cfg.z_wall_max]
        main = wall_idx[z <= cfg.z_wall_max]
        if main.size:
            keep.append(_sample_indices(main, KUJIALE_WALL_MAIN, rng))
        if ext.size:
            keep.append(_sample_indices(ext, KUJIALE_WALL_EXT, rng))
    if not keep:
        return xyz, rgb
    idx = np.sort(np.concatenate(keep))
    return xyz[idx], rgb[idx]


def _wall_z_samples(cfg: KujialeStyleConfig, *, extension: bool) -> np.ndarray:
    if extension:
        if cfg.wall_extension_max <= 0:
            return np.empty(0, dtype=np.float64)
        z0 = cfg.z_wall_max + 1e-6
        n = max(1, cfg.wall_extension_max)
        return np.linspace(z0, cfg.z_wall_ext_max, n)
    z_top = cfg.z_wall_max - 1e-4
    n = max(1, cfg.wall_column_max)
    return np.linspace(cfg.z_wall_min, z_top, n)


def _low_z_samples(cfg: KujialeStyleConfig, n: int) -> np.ndarray:
    z_top = min(cfg.z_low_max, Z_WALL_MIN - 1e-4)
    if n <= 1:
        return np.array([(z_top + 1e-6) * 0.5], dtype=np.float64)
    return np.linspace(max(1e-6, 1e-9), z_top, max(1, n))


def occupancy_grid_to_kujiale(
    grid: OccupancyGrid2D,
    *,
    cfg: KujialeStyleConfig | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Synthesize a kujiale-style 2.5-D point cloud from a 2-D occupancy map."""
    cfg = cfg or KujialeStyleConfig()
    cells = grid.cells
    free_map = _map_keys(cells == 0)
    occ_raw = _map_keys(cells == 100)
    occ_map = occ_raw
    if not free_map and not occ_map:
        raise ValueError('occupancy grid has no labelled cells')

    c_vals = [c for c, _ in free_map | occ_map]
    r_vals = [r for _, r in free_map | occ_map]
    bounds = (min(c_vals), max(c_vals), min(r_vals), max(r_vals))
    if cfg.map_connect_close_cells > 0:
        free_map = _close_free(free_map, occ_map, bounds, cfg.map_connect_close_cells)
    occ_map = _filter_occ_speckle(occ_map)
    free_map |= occ_raw - occ_map
    wall_map = _occ_adjacent_to_free(occ_map, free_map)

    dist = (
        _distance_field(wall_map, bounds, grid.resolution)
        if cfg.encode_distance_field else {}
    )

    low_per_cell = 1 if cfg.target_points is not None else max(1, cfg.low_column_max)

    out_xyz: list[np.ndarray] = []
    out_rgb: list[np.ndarray] = []

    # --- wall @ boundary occupied cells only ---
    for c, r in sorted(wall_map):
        cx, cy = grid.cell_center(c, r)
        for zs in _wall_z_samples(cfg, extension=False):
            out_xyz.append(np.array([[cx, cy, zs]], dtype=np.float64))
            out_rgb.append(COLOR_WALL.reshape(1, 3))
        for zs in _wall_z_samples(cfg, extension=True):
            out_xyz.append(np.array([[cx, cy, zs]], dtype=np.float64))
            out_rgb.append(COLOR_WALL.reshape(1, 3))

    # --- low @ wall boundary (blue, z in (0, 0.1]) ---
    for c, r in sorted(wall_map):
        cx, cy = grid.cell_center(c, r)
        for z in _low_z_samples(cfg, low_per_cell):
            out_xyz.append(np.array([[cx, cy, z]], dtype=np.float64))
            out_rgb.append(COLOR_LOW.reshape(1, 3))

    # --- floor @ z=0 (one point per free cell, distance-field RGB) ---
    for c, r in sorted(free_map):
        cx, cy = grid.cell_center(c, r)
        if dist:
            rgb = _encode_distance_rgb(dist.get((c, r), cfg.dist_far_m), cfg)
        else:
            rgb = COLOR_FREE
        out_xyz.append(np.array([[cx, cy, 0.0]], dtype=np.float64))
        out_rgb.append(rgb.reshape(1, 3))

    xyz = np.vstack(out_xyz)
    rgb = np.vstack(out_rgb).astype(np.uint8)
    return _pack_layers(xyz, rgb, cfg, floor_keep=len(free_map))


def write_kujiale_ply(path: str | Path, xyz: np.ndarray, rgb: np.ndarray) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    n = xyz.shape[0]
    header = (
        'ply\n'
        'format binary_little_endian 1.0\n'
        'comment Created by Open3D\n'
        f'element vertex {n}\n'
        'property double x\n'
        'property double y\n'
        'property double z\n'
        'property uchar red\n'
        'property uchar green\n'
        'property uchar blue\n'
        'end_header\n'
    )
    packed = np.zeros(n, dtype=OUTPUT_DTYPE)
    packed['x'] = xyz[:, 0]
    packed['y'] = xyz[:, 1]
    packed['z'] = xyz[:, 2]
    packed['red'] = rgb[:, 0]
    packed['green'] = rgb[:, 1]
    packed['blue'] = rgb[:, 2]
    with path.open('wb') as f:
        f.write(header.encode('ascii'))
        packed.tofile(f)


def read_kujiale_ply(path: str | Path) -> tuple[np.ndarray, np.ndarray]:
    with Path(path).open('rb') as f:
        n = 0
        while True:
            line = f.readline().decode('ascii').strip()
            if line.startswith('element vertex'):
                n = int(line.split()[-1])
            if line == 'end_header':
                break
        verts = np.fromfile(f, dtype=OUTPUT_DTYPE, count=n)
    xyz = np.column_stack([verts['x'], verts['y'], verts['z']])
    rgb = np.column_stack([verts['red'], verts['green'], verts['blue']])
    return xyz, rgb


def occupancy_grid_from_xyz(
    xyz: np.ndarray,
    *,
    resolution: float = MAP_RES,
    z_min: float = 0.0,
    z_max: float = Z_WALL_EXT_MAX,
) -> tuple[np.ndarray, float, float]:
    """Rasterize a kujiale cloud into a 2-D occupancy grid (inverse of synthesis)."""
    mask = (xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max)
    pts = xyz[mask]
    if pts.size == 0:
        raise ValueError(f'no points in z=[{z_min}, {z_max}]')
    floor_pts = pts[pts[:, 2] <= 1e-6]
    ref = floor_pts if floor_pts.size else pts
    # Align to cell-corner origin so floor cell centers map back to unique cells.
    ox = float(ref[:, 0].min() - 0.5 * resolution)
    oy = float(ref[:, 1].min() - 0.5 * resolution)
    w = max(1, int(np.ceil((pts[:, 0].max() - ox) / resolution)))
    h = max(1, int(np.ceil((pts[:, 1].max() - oy) / resolution)))
    cells = np.full((h, w), -1, dtype=np.int8)
    gx = np.clip(((pts[:, 0] - ox) / resolution).astype(np.int32), 0, w - 1)
    gy = np.clip(((pts[:, 1] - oy) / resolution).astype(np.int32), 0, h - 1)
    for x, y, z in zip(gx, gy, pts[:, 2]):
        if z >= Z_WALL_MIN - 1e-6:
            cells[y, x] = 100
    for x, y, z in zip(gx, gy, pts[:, 2]):
        if z <= 1e-6:
            cells[y, x] = 0
    cells[cells < 0] = 100
    return cells, ox, oy


def occupancy_grid_to_ros(
    grid: OccupancyGrid2D,
) -> tuple[np.ndarray, float, float]:
    """Export OccupancyGrid2D as ROS OccupancyGrid cells + corner origin."""
    return grid.cells.copy(), grid.origin_x, grid.origin_y


def map_navigability_stats(
    xyz: np.ndarray,
    *,
    resolution: float = MAP_RES,
    z_max: float = Z_WALL_EXT_MAX,
) -> dict[str, int]:
    cells, _, _ = occupancy_grid_from_xyz(xyz, resolution=resolution, z_max=z_max)
    free = cells == 0
    visited = np.zeros_like(free, dtype=bool)
    comps: list[int] = []
    h, w = cells.shape
    for y in range(h):
        for x in range(w):
            if not free[y, x] or visited[y, x]:
                continue
            stack = [(y, x)]
            visited[y, x] = True
            n = 0
            while stack:
                cy, cx = stack.pop()
                n += 1
                for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                    ny, nx = cy + dy, cx + dx
                    if 0 <= ny < h and 0 <= nx < w and free[ny, nx] and not visited[ny, nx]:
                        visited[ny, nx] = True
                        stack.append((ny, nx))
            comps.append(n)
    comps.sort(reverse=True)
    return {
        'map_free_cells': int(free.sum()),
        'map_occupied_cells': int(np.sum(cells == 100)),
        'map_components': len(comps),
        'map_largest_component': comps[0] if comps else 0,
    }


def convert_navmesh_to_kujiale(
    navmesh_path: str | Path,
    output_path: str | Path,
    *,
    cfg: KujialeStyleConfig | None = None,
) -> tuple[int, dict[str, int]]:
    grid = load_occupancy_from_navmesh(navmesh_path, map_resolution=(cfg or KujialeStyleConfig()).map_resolution)
    xyz, rgb = occupancy_grid_to_kujiale(grid, cfg=cfg)
    write_kujiale_ply(output_path, xyz, rgb)
    labels = _layer_label(xyz, rgb)
    return xyz.shape[0], {
        'floor': int(np.sum(labels == 0)),
        'low': int(np.sum(labels == 1)),
        'wall': int(np.sum(labels == 2)),
    }


def ensure_scene_pointcloud(
    scene_dir: str | Path,
    *,
    scene_id: str | None = None,
    reference_ply: str | Path | None = None,
    force: bool = False,
    cfg: KujialeStyleConfig | None = None,
) -> tuple[Path | None, dict[str, object]]:
    """Build or refresh pointcloud.ply from scene .navmesh occupancy (no semantic PLY)."""
    scene_dir = Path(scene_dir)
    scene_id = scene_id or scene_dir.name
    navmesh = scene_dir / f'{scene_id}.navmesh'
    output = scene_dir / 'pointcloud.ply'
    info: dict[str, object] = {'navmesh': str(navmesh), 'output': str(output)}
    if not navmesh.is_file():
        info['status'] = 'navmesh_missing'
        return None, info
    stale = not output.is_file() or output.stat().st_mtime < navmesh.stat().st_mtime
    if output.is_file() and not stale and not force:
        info['status'] = 'cached'
        return output, info
    cfg = cfg or KujialeStyleConfig()
    count, layers = convert_navmesh_to_kujiale(navmesh, output, cfg=cfg)
    info.update(status='generated' if stale else 'overwritten', points=count, layers=layers)
    xyz, _ = read_kujiale_ply(output)
    info['map'] = map_navigability_stats(xyz, resolution=cfg.map_resolution, z_max=cfg.z_wall_ext_max)
    if reference_ply and Path(reference_ply).is_file():
        ref_xyz, _ = read_kujiale_ply(reference_ply)
        info['reference_map'] = map_navigability_stats(
            ref_xyz, resolution=cfg.map_resolution, z_max=cfg.z_wall_ext_max)
    return output, info


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Build kujiale-style pointcloud.ply from occupancy grid / navmesh')
    parser.add_argument('input', type=Path, nargs='?',
                        help='Scene dir or .navmesh path')
    parser.add_argument('-o', '--output', type=Path, default=None,
                        help='Output PLY (default: <scene>/pointcloud.ply)')
    parser.add_argument('--reference', type=Path, default=None,
                        help='Optional kujiale reference PLY for stats')
    parser.add_argument('--force', action='store_true', help='Overwrite existing output')
    parser.add_argument('--map-resolution', type=float, default=MAP_RES)
    parser.add_argument('--target-points', type=int, default=DEFAULT_TARGET_POINTS)
    args = parser.parse_args()

    if args.input is None:
        parser.error('input scene directory or .navmesh path is required')

    inp = args.input
    cfg = KujialeStyleConfig(
        map_resolution=args.map_resolution,
        target_points=args.target_points,
    )
    if inp.suffix == '.navmesh':
        out = args.output or inp.with_name('pointcloud.ply')
        count, layers = convert_navmesh_to_kujiale(inp, out, cfg=cfg)
    else:
        out, info = ensure_scene_pointcloud(
            inp, reference_ply=args.reference, force=args.force, cfg=cfg)
        if out is None:
            raise SystemExit(info.get('status', 'failed'))
        count = info.get('points', 0)
        layers = info.get('layers', {})
    print(f'Wrote {count} points to {out}')
    print(f'  floor: {layers.get("floor", 0)}  low: {layers.get("low", 0)}  '
          f'wall: {layers.get("wall", 0)}')


if __name__ == '__main__':
    main()
