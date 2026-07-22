# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Resolve rosidl Python message paths for launch-time PYTHONPATH."""

from __future__ import annotations

import os
import sys

from ament_index_python.packages import PackageNotFoundError, get_package_prefix


def _python_subdir() -> str:
    return f'python{sys.version_info.major}.{sys.version_info.minor}'


def _paths_under_prefix(prefix: str) -> list[str]:
    subdir = _python_subdir()
    candidates = (
        os.path.join(prefix, 'local', 'lib', subdir, 'dist-packages'),
        os.path.join(prefix, 'lib', subdir, 'site-packages'),
        os.path.join(prefix, 'lib', subdir, 'dist-packages'),
    )
    return [path for path in candidates if os.path.isdir(path)]


def _install_root_fallback() -> str | None:
    try:
        return os.path.dirname(get_package_prefix('autonomy_ros'))
    except PackageNotFoundError:
        return None


def rosidl_python_paths(*package_names: str) -> list[str]:
    """Return existing rosidl Python directories for the given packages."""
    paths: list[str] = []
    install_root = _install_root_fallback()

    for name in package_names:
        prefix: str | None = None
        try:
            prefix = get_package_prefix(name)
        except PackageNotFoundError:
            if install_root is not None:
                candidate = os.path.join(install_root, name)
                if os.path.isdir(candidate):
                    prefix = candidate
        if prefix is None:
            continue
        for path in _paths_under_prefix(prefix):
            if path not in paths:
                paths.append(path)
    return paths


def pythonpath_env(*package_names: str) -> dict[str, str]:
    """Build a PYTHONPATH env dict, preserving the current value."""
    extra = rosidl_python_paths(*package_names)
    current = os.environ.get('PYTHONPATH', '')
    parts = extra + ([current] if current else [])
    return {'PYTHONPATH': ':'.join(parts)}
