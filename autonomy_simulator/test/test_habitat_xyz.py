"""Habitat position conversion tests."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

from habitat.coords import habitat_xyz


class _Vector3Like:
    def __init__(self, x: float, y: float, z: float) -> None:
        self.x = x
        self.y = y
        self.z = z


def test_habitat_xyz_from_magnum_like() -> None:
    point = habitat_xyz(_Vector3Like(1.5, -0.2, 3.0))
    assert point.dtype == np.float32
    assert point.shape == (3,)
    np.testing.assert_allclose(point, [1.5, -0.2, 3.0], rtol=0.0, atol=1e-6)
