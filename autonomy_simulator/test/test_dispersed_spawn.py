"""Dispersed spawn point selection tests."""

import math
import sys
from pathlib import Path

import numpy as np

# habitat package lives under scripts/
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

from habitat.spawn import pick_dispersed_navigable_points


def _grid_points() -> list[np.ndarray]:
    pts = []
    for x in range(5):
        for z in range(5):
            pts.append(np.array([float(x), 0.0, float(z)], dtype=np.float32))
    return pts


def test_dispersed_points_are_spread() -> None:
    pool = _grid_points()
    rng = np.random.default_rng(7)

    def sample() -> np.ndarray:
        return pool[int(rng.integers(len(pool)))]

    selected = pick_dispersed_navigable_points(sample, 3, seed=42, pool_size=25)
    assert len(selected) == 3
    d01 = math.hypot(float(selected[0][0] - selected[1][0]), float(selected[0][2] - selected[1][2]))
    d02 = math.hypot(float(selected[0][0] - selected[2][0]), float(selected[0][2] - selected[2][2]))
    assert d01 > 1.0
    assert d02 > 1.0


def test_dispersed_is_deterministic() -> None:
    pool = _grid_points()
    rng = np.random.default_rng(1)

    def sample() -> np.ndarray:
        return pool[int(rng.integers(len(pool)))]

    a = pick_dispersed_navigable_points(sample, 2, seed=99, pool_size=20)
    rng = np.random.default_rng(1)
    b = pick_dispersed_navigable_points(sample, 2, seed=99, pool_size=20)
    assert len(a) == len(b) == 2
    for p, q in zip(a, b):
        assert np.allclose(p, q)
