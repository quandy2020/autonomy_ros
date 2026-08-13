"""Configuration loading utilities."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    merged = dict(base)
    for key, value in override.items():
        if key == '_base':
            continue
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def load_yaml_config(path: str | Path, profile: str | None = None) -> dict[str, Any]:
    """Load YAML with optional ``_base`` inheritance and named ``profiles``."""
    config_path = Path(path).expanduser().resolve()
    if not config_path.is_file():
        raise FileNotFoundError(f'Config not found: {config_path}')
    with config_path.open('r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f'Expected mapping at root of {config_path}')

    base_name = data.pop('_base', None)
    profiles = data.pop('profiles', None)
    default_profile = data.pop('default_profile', None)

    if base_name:
        base_path = (config_path.parent / str(base_name)).resolve()
        base_data = load_yaml_config(base_path)
        data = _deep_merge(base_data, data)

    profile_name = profile or default_profile
    if profiles is not None:
        if not profile_name:
            available = sorted(profiles)
            raise ValueError(
                f'Config {config_path} defines profiles {available} '
                'but no profile was selected (use --profile or default_profile).'
            )
        if profile_name not in profiles:
            raise ValueError(
                f'Unknown profile {profile_name!r} in {config_path}; '
                f'available: {sorted(profiles)}'
            )
        override = profiles[profile_name]
        if not isinstance(override, dict):
            raise ValueError(f'Profile {profile_name!r} must be a mapping')
        data = _deep_merge(data, override)

    return data
