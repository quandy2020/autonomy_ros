# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import json
import os
from typing import Any

from habitat.config import Config


def _workspace_root() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    for _ in range(8):
        parent = os.path.dirname(here)
        if os.path.isdir(os.path.join(parent, 'TrackVLA')):
            return parent
        if parent == here:
            break
        here = parent
    return os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(here))))


def resolve_trackvla_root(cfg: Config) -> str:
    if str(cfg.trackvla_root).strip():
        return str(cfg.trackvla_root).rstrip('/')
    env = os.environ.get('TRACKVLA_ROOT', '').strip()
    if env:
        return env.rstrip('/')
    return os.path.join(_workspace_root(), 'TrackVLA')


def resolve_humanoid_data_root(cfg: Config) -> str:
    if str(cfg.humanoid_data_root).strip():
        return str(cfg.humanoid_data_root).rstrip('/')
    env = os.environ.get('AUTONOMY_HUMANOID_DATA_ROOT', '').strip()
    if env:
        return env.rstrip('/')
    try:
        from autonomy_lerobot.data_paths import humanoid_data_root

        return str(humanoid_data_root())
    except Exception:
        pass
    return os.path.join(
        resolve_trackvla_root(cfg), 'data', 'humanoids', 'humanoid_data',
    )


def resolve_humanoid_infos_json(cfg: Config) -> str:
    if str(cfg.humanoid_infos_json).strip():
        return str(cfg.humanoid_infos_json)
    return os.path.join(resolve_trackvla_root(cfg), 'humanoid_infos.json')


def avatar_asset_paths(data_root: str, avatar: str) -> tuple[str, str]:
    folder = os.path.join(data_root, avatar)
    return (
        os.path.join(folder, f'{avatar}.urdf'),
        os.path.join(folder, f'{avatar}_motion_data_smplx.pkl'),
    )


def is_avatar_available(data_root: str, avatar: str) -> bool:
    urdf, motion = avatar_asset_paths(data_root, avatar)
    return os.path.isfile(urdf) and os.path.isfile(motion)


def list_available_avatars(data_root: str) -> list[str]:
    """Discover avatar folder names with URDF + SMPL-X motion on disk."""
    if not os.path.isdir(data_root):
        return []
    names: list[str] = []
    for entry in sorted(os.listdir(data_root)):
        if os.path.isdir(os.path.join(data_root, entry)) and is_avatar_available(
            data_root, entry,
        ):
            names.append(entry)
    return names


def resolve_avatar_roster(cfg: Config) -> list[str]:
    """Avatar names to cycle across pedestrians (explicit list or auto-discover)."""
    root = resolve_humanoid_data_root(cfg)
    requested = [
        part.strip()
        for part in str(cfg.humanoid_avatars).split(',')
        if part.strip()
    ]
    if requested:
        roster = [name for name in requested if is_avatar_available(root, name)]
        if roster:
            return roster
    discovered = list_available_avatars(root)
    if discovered:
        return discovered
    fallback = str(cfg.humanoid_avatar).strip()
    if fallback and is_avatar_available(root, fallback):
        return [fallback]
    return []


def avatar_template(
    cfg: Config,
    data_root: str,
    avatar: str,
    infos_path: str,
) -> tuple[str, str, int]:
    urdf, motion = avatar_asset_paths(data_root, avatar)
    semantic_id = load_semantic_id(infos_path, avatar, cfg.pedestrian_semantic_id)
    return urdf, motion, semantic_id


def humanoid_assets_available(cfg: Config) -> tuple[bool, str]:
    """Return (ok, message) when at least one avatar URDF + motion exists."""
    root = resolve_humanoid_data_root(cfg)
    roster = resolve_avatar_roster(cfg)
    if not roster:
        return False, f'no valid humanoid avatars under {root}'
    return True, root


def load_semantic_id(infos_path: str, avatar: str, fallback: int) -> int:
    if not os.path.isfile(infos_path):
        return fallback
    with open(infos_path, encoding='utf-8') as f:
        entries: list[dict[str, Any]] = json.load(f)
    for entry in entries:
        if entry.get('name') == avatar:
            return int(entry['semantic_id'])
    return fallback
