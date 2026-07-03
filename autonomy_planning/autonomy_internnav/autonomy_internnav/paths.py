# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Resolve checkpoints and baseline config paths."""

from __future__ import annotations

import importlib.util
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

DEFAULT_CHECKPOINT = 'navdp-cross-modal.ckpt'
_SRC_WEIGHTS_REL = os.path.join(
    'autonomy_ros', 'autonomy_planning', 'autonomy_internnav', 'weights',
)


def package_weights_dirs() -> list[str]:
    """Return candidate weights directories, most specific first."""
    seen: set[str] = set()
    dirs: list[str] = []

    def add(path: str) -> None:
        norm = os.path.abspath(path)
        if norm not in seen:
            seen.add(norm)
            dirs.append(norm)

    env_weights = os.environ.get('AUTONOMY_INTERNNAV_WEIGHTS', '').strip()
    if env_weights:
        add(env_weights)
    add(os.path.join(os.path.dirname(os.path.dirname(__file__)), 'weights'))

    cur = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
    for _ in range(12):
        add(os.path.join(cur, 'weights'))
        src = os.path.join(cur, 'src')
        if os.path.isdir(src):
            add(os.path.join(src, _SRC_WEIGHTS_REL))
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent

    try:
        share = get_package_share_directory('autonomy_internnav')
        install_pkg = os.path.dirname(os.path.dirname(share))
        ws_root = os.path.dirname(os.path.dirname(install_pkg))
        add(os.path.join(ws_root, 'src', _SRC_WEIGHTS_REL))
        add(os.path.join(share, 'weights'))
    except PackageNotFoundError:
        pass
    return dirs


def resolve_asset(path: str, *, default_name: str = '') -> str:
    """Resolve a checkpoint or other weight file to an absolute path."""
    candidate = path.strip() if path else default_name
    if not candidate:
        raise FileNotFoundError('Asset path is empty')
    basename = os.path.basename(candidate)
    checked: list[str] = []

    env_ckpt = os.environ.get('NAVDP_CHECKPOINT', '').strip()
    if env_ckpt and (not path or path == default_name):
        checked.append(env_ckpt)
        if os.path.isfile(env_ckpt):
            return os.path.abspath(env_ckpt)
    if os.path.isfile(candidate):
        return os.path.abspath(candidate)
    for weights_dir in package_weights_dirs():
        resolved = os.path.join(weights_dir, basename)
        checked.append(resolved)
        if os.path.isfile(resolved):
            return os.path.abspath(resolved)
    raise FileNotFoundError(
        f'Asset not found: {candidate!r}. Place {basename} under weights/ '
        f'or set NAVDP_CHECKPOINT. Checked: {checked}',
    )


def resolve_checkpoint(path: str) -> str:
    return resolve_asset(path, default_name=DEFAULT_CHECKPOINT)


def baseline_package_dir(policy: str) -> str:
    spec = importlib.util.find_spec(f'autonomy_internnav.baselines.{policy}')
    if spec is None or not spec.origin:
        raise FileNotFoundError(f'Unknown baseline package: {policy}')
    return os.path.dirname(spec.origin)


def baseline_config_path(policy: str, filename: str) -> str:
    path = os.path.join(baseline_package_dir(policy), 'configs', filename)
    if not os.path.isfile(path):
        raise FileNotFoundError(f'Baseline config not found: {path}')
    return path
