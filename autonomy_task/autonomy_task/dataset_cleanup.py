"""Cleanup orphaned LeRobot encoder temp directories under dataset roots."""

from __future__ import annotations

import shutil
from pathlib import Path


def _is_orphan_tmp_dir(path: Path) -> bool:
    """True for LeRobot streaming-encoder leftovers (tmpXXXX at dataset root)."""
    if not path.is_dir() or not path.name.startswith('tmp'):
        return False
    parent = path.parent
    return (parent / 'meta').is_dir() or (parent / 'data').is_dir()


def _dir_size(path: Path) -> int:
    return sum(f.stat().st_size for f in path.rglob('*') if f.is_file())


def cleanup_dataset_tree(root: Path) -> tuple[int, int]:
    """Remove orphan tmp* dirs under *root*. Returns (count, bytes_freed)."""
    if not root.is_dir():
        return 0, 0
    removed = 0
    freed = 0
    for child in root.iterdir():
        if not _is_orphan_tmp_dir(child):
            continue
        freed += _dir_size(child)
        shutil.rmtree(child, ignore_errors=True)
        removed += 1
    return removed, freed


def cleanup_dataset_roots(roots: list[Path]) -> dict[str, tuple[int, int]]:
    """Cleanup each dataset root; returns {path: (dirs_removed, bytes_freed)}."""
    out: dict[str, tuple[int, int]] = {}
    for root in roots:
        path = root.expanduser().resolve()
        out[str(path)] = cleanup_dataset_tree(path)
    return out
