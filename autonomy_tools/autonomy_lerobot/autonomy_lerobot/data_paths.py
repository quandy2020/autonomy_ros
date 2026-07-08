"""On-disk data paths — single source: config/data_paths.yaml."""

from __future__ import annotations

import os
from functools import lru_cache
from pathlib import Path

import yaml


def _config_file() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory('autonomy_lerobot')
        return Path(share) / 'config' / 'data_paths.yaml'
    except Exception:
        return Path(__file__).resolve().parent.parent / 'config' / 'data_paths.yaml'


@lru_cache(maxsize=1)
def _raw() -> dict:
    path = _config_file()
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError(f'data_paths.yaml must be a mapping: {path}')
    return data


def volume_root() -> Path:
    override = os.environ.get('AUTONOMY_VOLUME_ROOT', '').strip()
    if override:
        return Path(override).expanduser().resolve()
    return Path(str(_raw()['volume_root'])).expanduser().resolve()


def data_root() -> Path:
    if 'data_root' in _raw():
        return Path(str(_raw()['data_root'])).expanduser().resolve()
    return volume_root() / 'data'


def mp3d_root() -> Path:
    if 'mp3d_root' in _raw():
        return Path(str(_raw()['mp3d_root'])).expanduser().resolve()
    return volume_root() / 'Datasets' / 'mp3d'


def mp3d_default_scene_id() -> str:
    return str(_raw().get('mp3d_default_scene_id', '17DRP5sb8fy'))


def default_mp3d_scene() -> Path:
    """MP3D scene directory; ``AUTONOMY_MP3D_SCENE`` overrides yaml."""
    override = os.environ.get('AUTONOMY_MP3D_SCENE', '').strip().rstrip('/')
    if override:
        return Path(override).expanduser().resolve()
    return mp3d_root() / mp3d_default_scene_id()


def humanoid_data_root() -> Path:
    """TrackVLA-style humanoid URDF root; ``AUTONOMY_HUMANOID_DATA_ROOT`` overrides yaml."""
    override = os.environ.get('AUTONOMY_HUMANOID_DATA_ROOT', '').strip().rstrip('/')
    if override:
        return Path(override).expanduser().resolve()
    if 'humanoid_data_root' in _raw():
        return Path(str(_raw()['humanoid_data_root'])).expanduser().resolve()
    return volume_root() / 'Datasets' / 'humanoids' / 'humanoids' / 'humanoid_data'


def lerobot_root() -> Path:
    return data_root() / 'lerobot'


def lerobot_collection_root() -> Path:
    """Multi-robot collection dataset root (per-robot subdirs: robot1/, robot2/, …)."""
    return lerobot_root() / 'collection'


def list_collection_robots(collection_root: Path | None = None) -> dict[str, Path]:
    """Return ``{robot_name: dataset_dir}`` for visualizable local datasets."""
    root = (collection_root or lerobot_collection_root()).expanduser().resolve()
    if not root.is_dir():
        return {}
    from autonomy_lerobot.dataset_health import has_episodes_meta

    return {
        child.name: child
        for child in sorted(root.iterdir())
        if child.is_dir()
        and (child / 'meta' / 'info.json').is_file()
        and has_episodes_meta(child)
    }


def lerobot_habitat_nav2_root() -> Path:
    """Single-robot Habitat + Nav2 recording root."""
    return lerobot_root() / 'habitat_nav2'


def collection_state_file() -> Path:
    return data_root() / 'collection' / 'state.json'


def collection_repo_id() -> str:
    return str(_raw().get('collection_repo_id', 'local/habitat_collection'))


def nav2_repo_id() -> str:
    return str(_raw().get('nav2_repo_id', 'local/habitat_nav2'))
