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

import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

_SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(_SCRIPTS))

from semantic_ply_to_kujiale import (
    COLOR_NEAR,
    COLOR_WALL,
    KujialeStyleConfig,
    OccupancyGrid2D,
    _layer_label,
    convert_navmesh_to_kujiale,
    load_occupancy_from_array,
    occupancy_grid_to_kujiale,
    read_kujiale_ply,
    write_kujiale_ply,
)


class OccupancyToKujialeTest(unittest.TestCase):
    def _room_grid(self) -> OccupancyGrid2D:
        cells = np.full((20, 30), 100, dtype=np.int8)
        cells[2:18, 2:28] = 0
        cells[8:12, 12:18] = 100  # inner obstacle
        return load_occupancy_from_array(cells, origin_x=0.0, origin_y=0.0, resolution=0.05)

    def test_output_header_matches_kujiale(self) -> None:
        xyz = np.array([[0.0, 0.0, 0.0], [1.0, 2.0, 0.2]], dtype=np.float64)
        rgb = np.vstack([[128, 255, 128], COLOR_WALL])
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / 'pointcloud.ply'
            write_kujiale_ply(out, xyz, rgb)
            header = out.read_bytes().split(b'end_header\n', 1)[0].decode('ascii')
            self.assertIn('format binary_little_endian 1.0', header)
            self.assertIn('property double x', header)
            self.assertNotIn('element face', header)

    def test_sparse_layers_from_grid(self) -> None:
        cfg = KujialeStyleConfig(target_points=None, encode_distance_field=False)
        xyz, rgb = occupancy_grid_to_kujiale(self._room_grid(), cfg=cfg)
        self.assertGreater(xyz.shape[0], 10)
        labels = _layer_label(xyz, rgb)
        self.assertGreater(np.sum(labels == 0), 0)
        self.assertGreater(np.sum(labels == 1), 0)
        self.assertGreater(np.sum(labels == 2), 0)
        wall_z = xyz[labels == 2, 2]
        self.assertGreaterEqual(wall_z.min(), 0.1 - 1e-6)

    def test_distance_field_on_floor(self) -> None:
        cells = np.full((10, 20), 100, dtype=np.int8)
        cells[1:9, 1:19] = 0
        grid = load_occupancy_from_array(cells, 0.0, 0.0, 0.05)
        cfg = KujialeStyleConfig(encode_distance_field=True, target_points=None)
        xyz, rgb = occupancy_grid_to_kujiale(grid, cfg=cfg)
        floor = rgb[xyz[:, 2] <= 1e-9]
        self.assertGreater(floor.shape[0], 0)
        self.assertTrue(np.any(np.all(floor == COLOR_NEAR, axis=1)))

    def test_navmesh_scene_convert(self) -> None:
        navmesh = Path('/media/quandy/DATA4T/mp3d/17DRP5sb8fy/17DRP5sb8fy.navmesh')
        if not navmesh.is_file():
            self.skipTest('MP3D navmesh not available')
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / 'pointcloud.ply'
            try:
                count, layers = convert_navmesh_to_kujiale(navmesh, out)
            except ImportError:
                self.skipTest('habitat_sim not available')
            self.assertGreater(count, 1000)
            self.assertEqual(count, layers['floor'] + layers['low'] + layers['wall'])
            xyz, _ = read_kujiale_ply(out)
            self.assertAlmostEqual(xyz[:, 2].max(), 0.55, delta=0.01)

    def test_reference_stats_order_of_magnitude(self) -> None:
        ref = Path('/home/quandy/workspace/data/kujiale_0003/pointcloud.ply')
        if not ref.is_file():
            self.skipTest('kujiale reference not available')
        xyz, rgb = read_kujiale_ply(ref)
        self.assertEqual(xyz.shape[0], 250_644)
        labels = _layer_label(xyz, rgb)
        self.assertAlmostEqual(np.sum(labels == 0), 23_572, delta=500)
        self.assertAlmostEqual(np.sum(labels == 1), 41_852, delta=500)
        self.assertAlmostEqual(np.sum(labels == 2), 185_220, delta=500)


if __name__ == '__main__':
    unittest.main()
