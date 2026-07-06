# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

from typing import Any

import magnum as mn
import numpy as np

from habitat.config import Config
from habitat.humanoid.avatar import HumanoidAvatar
from habitat.humanoid.paths import (
    avatar_template,
    humanoid_assets_available,
    resolve_avatar_roster,
    resolve_humanoid_data_root,
    resolve_humanoid_infos_json,
)
from habitat.sim import Session


def _map_to_habitat(map_x: float, map_y: float, floor: float) -> np.ndarray:
    return np.array([map_x, floor, -map_y], dtype=np.float32)


class HumanoidMeshManager:
    """Spawns TrackVLA URDF humanoids into Habitat-Sim for native camera rendering."""

    def __init__(self, cfg: Config, session: Session, logger: Any) -> None:
        self._cfg = cfg
        self._session = session
        self._logger = logger
        self._avatars: list[HumanoidAvatar] = []
        self._active = False
        self._rel_hab: list[np.ndarray] = []
        self._roster: list[str] = []
        self._template_cache: dict[str, tuple[str, str, int]] = {}

        ok, detail = humanoid_assets_available(cfg)
        if not ok:
            self._logger.error(
                f'Humanoid assets unavailable ({detail}). '
                'Set humanoid_data_root or run TrackVLA/download_pedestrian_data.py',
            )
            return

        self._roster = resolve_avatar_roster(cfg)
        if not self._roster:
            self._logger.error('No humanoid avatars resolved from data root')
            return

        self._data_root = resolve_humanoid_data_root(cfg)
        self._infos_path = resolve_humanoid_infos_json(cfg)
        self._active = True
        self._logger.info(
            f'Humanoid mesh rendering: {len(self._roster)} avatar type(s) '
            f'at {self._data_root} (e.g. {", ".join(self._roster[:4])}'
            f'{", ..." if len(self._roster) > 4 else ""})',
        )

    @property
    def active(self) -> bool:
        return self._active

    def pick_avatars(self, count: int, seed: int) -> list[str]:
        """Assign distinct avatar types when possible (shuffled roster, round-robin)."""
        if count <= 0 or not self._roster:
            return []
        rng = np.random.default_rng(int(seed) + 31)
        pool = list(self._roster)
        rng.shuffle(pool)
        if count <= len(pool):
            return pool[:count]
        return [pool[i % len(pool)] for i in range(count)]

    def _template(self, avatar_name: str) -> tuple[str, str, int]:
        if avatar_name not in self._template_cache:
            self._template_cache[avatar_name] = avatar_template(
                self._cfg,
                self._data_root,
                avatar_name,
                self._infos_path,
            )
        return self._template_cache[avatar_name]

    def clear(self) -> None:
        for avatar in self._avatars:
            avatar.remove()
        self._avatars = []
        self._rel_hab = []

    def respawn(self, people: list[Any], floor: float) -> None:
        if not self._active:
            return
        self.clear()
        sim = self._session.sim
        for ped in people:
            avatar_name = str(getattr(ped, 'avatar', self._roster[0]))
            urdf, motion, semantic_id = self._template(avatar_name)
            avatar = HumanoidAvatar(sim, urdf, motion, semantic_id, self._logger)
            avatar.spawn()
            hab = _map_to_habitat(ped.map_x, ped.map_y, floor)
            avatar.set_base_from_habitat(mn.Vector3(hab), ped.yaw)
            avatar.set_rest_position()
            avatar.reset_controller()
            self._avatars.append(avatar)
        self._rel_hab = [np.zeros(2, dtype=np.float64) for _ in people]

    def set_nav_rel(self, index: int, rel_map: np.ndarray) -> None:
        """Store habitat XZ relative nav vector for walk animation."""
        if 0 <= index < len(self._rel_hab):
            self._rel_hab[index] = np.array(
                [float(rel_map[0]), float(-rel_map[1])], dtype=np.float64,
            )

    def sync(self, people: list[Any], floor: float, robot_far: list[bool]) -> None:
        if not self._active or len(self._avatars) != len(people):
            return
        for idx, (ped, avatar) in enumerate(zip(people, self._avatars)):
            hab = _map_to_habitat(ped.map_x, ped.map_y, floor)
            moving = not robot_far[idx]
            avatar.sync_walk(hab, self._rel_hab[idx], ped.yaw, moving)
