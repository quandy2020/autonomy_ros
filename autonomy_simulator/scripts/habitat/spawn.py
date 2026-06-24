"""Navmesh spawn point selection."""

from __future__ import annotations

import math
from typing import Callable

import numpy as np


def map_dist(a: np.ndarray, b: np.ndarray) -> float:
    ax, ay = float(a[0]), -float(a[2])
    bx, by = float(b[0]), -float(b[2])
    return math.hypot(ax - bx, ay - by)


def pick_dispersed_navigable_points(
    sample_point: Callable[[], np.ndarray],
    count: int,
    *,
    seed: int,
    pool_size: int | None = None,
) -> list[np.ndarray]:
    """Greedy farthest-point sampling on navigable points (deterministic for *seed*)."""
    count = max(1, count)
    rng = np.random.default_rng(seed)
    target_pool = pool_size if pool_size is not None else min(max(count * 40, 100), 500)
    pool: list[np.ndarray] = []
    for _ in range(target_pool * 3):
        if len(pool) >= target_pool:
            break
        pool.append(sample_point())
    if not pool:
        return []

    first = int(rng.integers(len(pool)))
    selected = [pool[first]]
    while len(selected) < count:
        best_idx = max(
            range(len(pool)),
            key=lambda i: min(map_dist(pool[i], s) for s in selected),
        )
        selected.append(pool[best_idx])
    return selected
