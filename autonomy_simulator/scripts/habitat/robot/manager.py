"""Spawn simple URDF robots as dynamic Habitat actors."""

from __future__ import annotations

from typing import Any

import numpy as np

from habitat.config import Config
from habitat.robot.avatar import RobotAvatar
from habitat.robot.paths import (
    official_robot_asset_names,
    resolve_robot_asset_roster,
    resolve_robot_asset_root,
    robot_asset_template,
    robot_assets_available,
    robot_type_metadata,
    robot_visual_marker_spec,
)
from habitat.sim import Session


def _map_to_habitat(map_x: float, map_y: float, floor: float) -> np.ndarray:
    return np.array([map_x, floor, -map_y], dtype=np.float32)


class RobotMeshManager:
    """Spawns kinematic robot URDFs into Habitat-Sim for native camera rendering."""

    def __init__(self, cfg: Config, session: Session, logger: Any) -> None:
        self._cfg = cfg
        self._session = session
        self._logger = logger
        self._robots: list[RobotAvatar] = []
        self._active = False
        self._roster: list[str] = []
        self._template_cache: dict[str, tuple[str, int]] = {}

        ok, detail = robot_assets_available(cfg)
        if not ok:
            self._logger.error(
                f'Robot assets unavailable ({detail}). '
                'Set robot_asset_root or use autonomy_simulator/urdf assets.',
            )
            return

        self._roster = resolve_robot_asset_roster(cfg)
        if not self._roster:
            self._logger.error('No robot asset types resolved from asset root')
            return

        self._asset_root = resolve_robot_asset_root(cfg)
        self._active = True
        self._logger.info(
            f'Robot mesh rendering: {len(self._roster)} asset type(s) '
            f'at {self._asset_root} (e.g. {", ".join(self._roster[:4])}'
            f'{", ..." if len(self._roster) > 4 else ""})',
        )

    @property
    def active(self) -> bool:
        return self._active

    def _parse_asset_counts(self) -> list[tuple[str, int]]:
        raw = str(getattr(self._cfg, 'robot_asset_counts', '')).strip()
        if not raw:
            return []
        counts: list[tuple[str, int]] = []
        for part in raw.split(','):
            item = part.strip()
            if not item or '=' not in item:
                continue
            name, count_raw = item.split('=', 1)
            asset_name = name.strip()
            try:
                count = max(0, int(count_raw.strip()))
            except Exception:
                continue
            if asset_name and count > 0:
                counts.append((asset_name, count))
        return counts

    def pick_avatars(self, count: int, seed: int) -> list[str]:
        """Assign robot asset types in roster priority order."""
        del seed
        if count <= 0 or not self._roster:
            return []
        explicit_counts = self._parse_asset_counts()
        if explicit_counts:
            expanded: list[str] = []
            roster_set = set(self._roster)
            for asset_name, asset_count in explicit_counts:
                if asset_name not in roster_set:
                    continue
                expanded.extend([asset_name] * asset_count)
            if expanded:
                if len(expanded) >= count:
                    return expanded[:count]
                fill_pool = [name for name in self._roster if name not in set(expanded)]
                if not fill_pool:
                    fill_pool = list(self._roster)
                return expanded + [fill_pool[i % len(fill_pool)] for i in range(count - len(expanded))]
        official = set(official_robot_asset_names())
        official_pool = [name for name in self._roster if name in official]
        if official_pool:
            return [official_pool[i % len(official_pool)] for i in range(count)]
        if count <= len(self._roster):
            return list(self._roster[:count])
        return [self._roster[i % len(self._roster)] for i in range(count)]

    def _template(self, asset_name: str) -> tuple[str, int]:
        if asset_name not in self._template_cache:
            self._template_cache[asset_name] = robot_asset_template(
                self._cfg,
                asset_name,
            )
        return self._template_cache[asset_name]

    def actor_semantic_id(self, asset_name: str) -> int:
        return int(robot_type_metadata(self._cfg, asset_name)['semantic_id'])

    def actor_radius(self, asset_name: str) -> float:
        return float(robot_type_metadata(self._cfg, asset_name)['radius'])

    def actor_height(self, asset_name: str) -> float:
        return float(robot_type_metadata(self._cfg, asset_name)['height'])

    def actor_marker_spec(self, asset_name: str) -> dict[str, object] | None:
        return robot_visual_marker_spec(self._cfg, asset_name)

    def joint_state_maps(self) -> list[dict[str, float]]:
        return [robot.joint_state_map() for robot in self._robots]

    def clear(self) -> None:
        for robot in self._robots:
            robot.remove()
        self._robots = []

    def respawn(self, people: list[Any], floor: float) -> None:
        if not self._active:
            return
        self.clear()
        sim = self._session.sim
        for ped in people:
            asset_name = str(getattr(ped, 'avatar', self._roster[0]))
            urdf, semantic_id = self._template(asset_name)
            self._logger.info(
                f'dynamic robot asset={asset_name} '
                f'radius={self.actor_radius(asset_name):.3f} '
                f'height={self.actor_height(asset_name):.3f} '
                f'semantic_id={self.actor_semantic_id(asset_name)} '
                f'urdf={urdf}'
            )
            robot = RobotAvatar(sim, urdf, semantic_id, asset_name, self._logger)
            robot.spawn()
            hab = _map_to_habitat(ped.map_x, ped.map_y, floor)
            robot.set_base_from_habitat(hab, ped.yaw)
            self._robots.append(robot)

    def set_nav_rel(self, index: int, rel_map: np.ndarray) -> None:
        del index, rel_map

    def sync(self, people: list[Any], floor: float, robot_far: list[bool]) -> None:
        if not self._active or len(self._robots) != len(people):
            return
        for ped, robot, is_far in zip(people, self._robots, robot_far):
            hab = _map_to_habitat(ped.map_x, ped.map_y, floor)
            rel = ped.nav_rel_map if ped.nav_rel_map is not None else np.zeros(2)
            robot.sync_drive(hab, rel, ped.yaw, moving=not is_far)
