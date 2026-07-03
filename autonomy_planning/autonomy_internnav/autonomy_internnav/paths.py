"""Resolve checkpoints and baseline config paths."""

from __future__ import annotations

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory

DEFAULT_CHECKPOINT = 'navdp-cross-modal.ckpt'
_WEIGHTS_ENV = 'AUTONOMY_INTERNNAV_WEIGHTS'
_CHECKPOINT_ENV = 'NAVDP_CHECKPOINT'

_SRC_WEIGHTS_REL = os.path.join(
    'autonomy_ros', 'autonomy_planning', 'autonomy_internnav', 'weights')


def _dedupe(paths: list[str]) -> list[str]:
    unique: list[str] = []
    for path in paths:
        norm = os.path.abspath(path)
        if norm not in unique:
            unique.append(norm)
    return unique


def package_weights_dirs() -> list[str]:
    """Candidate weights directories, most specific first."""
    dirs: list[str] = []

    weights_env = os.environ.get(_WEIGHTS_ENV, '').strip()
    if weights_env:
        dirs.append(weights_env)

    dirs.append(os.path.join(os.path.dirname(os.path.dirname(__file__)), 'weights'))

    cur = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
    for _ in range(12):
        dirs.append(os.path.join(cur, 'weights'))
        src_root = os.path.join(cur, 'src')
        if os.path.isdir(src_root):
            dirs.append(os.path.join(src_root, _SRC_WEIGHTS_REL))
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent

    try:
        share = get_package_share_directory('autonomy_internnav')
        install_pkg = os.path.dirname(os.path.dirname(share))
        ws_root = os.path.dirname(os.path.dirname(install_pkg))
        dirs.append(os.path.join(ws_root, 'src', _SRC_WEIGHTS_REL))
        dirs.append(os.path.join(share, 'weights'))
    except Exception:
        pass

    return _dedupe(dirs)


def resolve_asset(path: str, *, default_name: str = '') -> str:
    """Resolve a checkpoint or other weight file."""
    candidate = path.strip() if path else default_name
    if not candidate:
        raise FileNotFoundError('Asset path is empty')
    basename = os.path.basename(candidate)
    checked: list[str] = []

    env_checkpoint = os.environ.get(_CHECKPOINT_ENV, '').strip()
    if env_checkpoint and (not path or path == default_name):
        checked.append(env_checkpoint)
        if os.path.isfile(env_checkpoint):
            return os.path.abspath(env_checkpoint)

    if os.path.isfile(candidate):
        return os.path.abspath(candidate)

    for weights_dir in package_weights_dirs():
        resolved = os.path.join(weights_dir, basename)
        checked.append(resolved)
        if os.path.isfile(resolved):
            return os.path.abspath(resolved)

    raise FileNotFoundError(
        f'Asset not found: {candidate!r}. Place {basename} under weights/ '
        f'or set {_CHECKPOINT_ENV}. Checked: {checked}')


def resolve_checkpoint(path: str) -> str:
    """Resolve NavDP-style checkpoint."""
    return resolve_asset(path, default_name=DEFAULT_CHECKPOINT)


def baseline_package_dir(policy: str) -> str:
    """Directory of ``autonomy_internnav.baselines.<policy>``."""
    module_name = f'autonomy_internnav.baselines.{policy}'
    spec = importlib.util.find_spec(module_name)
    if spec is None or not spec.origin:
        raise FileNotFoundError(f'Unknown baseline package: {policy}')
    return os.path.dirname(spec.origin)


def baseline_config_path(policy: str, filename: str) -> str:
    """Resolve a YAML config shipped with a baseline."""
    path = os.path.join(baseline_package_dir(policy), 'configs', filename)
    if not os.path.isfile(path):
        raise FileNotFoundError(f'Baseline config not found: {path}')
    return path
