"""Procedural goal and obstacle sampling (numpy, backend-agnostic)."""

from __future__ import annotations

import numpy as np


def sample_goal_positions(
    num_envs: int,
    arena_size_m: float,
    min_distance_m: float,
    rng: np.random.Generator,
) -> np.ndarray:
    """Sample 2D goal positions inside a square arena."""
    half = arena_size_m * 0.5
    goals = np.zeros((num_envs, 2), dtype=np.float32)
    for index in range(num_envs):
        while True:
            candidate = rng.uniform(-half, half, size=2).astype(np.float32)
            if np.linalg.norm(candidate) >= min_distance_m:
                goals[index] = candidate
                break
    return goals


def sample_obstacle_positions(
    num_obstacles: int,
    arena_size_m: float,
    min_separation_m: float,
    rng: np.random.Generator,
) -> np.ndarray:
    """Sample static obstacle XY positions."""
    half = arena_size_m * 0.5
    obstacles: list[np.ndarray] = []
    attempts = 0
    while len(obstacles) < num_obstacles and attempts < num_obstacles * 20:
        attempts += 1
        candidate = rng.uniform(-half, half, size=2).astype(np.float32)
        if np.linalg.norm(candidate) < 1.0:
            continue
        if all(np.linalg.norm(candidate - existing) >= min_separation_m for existing in obstacles):
            obstacles.append(candidate)
    if len(obstacles) < num_obstacles:
        padding = num_obstacles - len(obstacles)
        extras = rng.uniform(-half, half, size=(padding, 2)).astype(np.float32)
        obstacles.extend(list(extras))
    return np.stack(obstacles, axis=0)
