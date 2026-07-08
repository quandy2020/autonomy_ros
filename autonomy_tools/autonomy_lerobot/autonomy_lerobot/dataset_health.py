# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0
"""Validate and repair on-disk LeRobot datasets before training / visualization."""

from __future__ import annotations

from pathlib import Path


def has_episodes_meta(root: Path) -> bool:
    episodes = root / 'meta' / 'episodes'
    return episodes.is_dir() and any(episodes.rglob('*.parquet'))


def has_frame_data(root: Path) -> bool:
    data = root / 'data'
    return data.is_dir() and any(data.rglob('*.parquet'))


def active_tmp_dirs(root: Path) -> list[Path]:
    if not root.is_dir():
        return []
    return sorted(
        p for p in root.iterdir()
        if p.is_dir() and p.name.startswith('tmp')
    )


def repair_dataset(root: Path) -> dict:
    """Rebuild missing meta/episodes and remove encoder tmp dirs."""
    from autonomy_lerobot.dataset_cleanup import cleanup_dataset

    return cleanup_dataset(root, min_frames=1, max_frames=10_000_000, dry_run=False)


def ensure_dataset_ready_for_viz(
    root: Path,
    *,
    auto_repair: bool = True,
) -> str | None:
    """Return an error message when *root* cannot be visualized, else None."""
    root = root.expanduser().resolve()
    info = root / 'meta' / 'info.json'
    if not info.is_file():
        return f'missing {info}'

    if has_episodes_meta(root):
        return None

    if not has_frame_data(root):
        return (
            f'{root} has meta/info.json but no frame parquet under data/. '
            'Start collection and save at least one episode first.'
        )

    if not auto_repair:
        return (
            f'{root} is missing meta/episodes parquet metadata. '
            'Wait for save_episode/finalize, or run: '
            'ros2 run autonomy_lerobot cleanup_lerobot_dataset --root <collection>'
        )

    report = repair_dataset(root)
    if report.get('skipped'):
        return (
            f'could not repair {root}: {report["skipped"]}. '
            'If recording is still active, call save_episode and retry.'
        )
    if not has_episodes_meta(root):
        return (
            f'repair finished but {root / "meta/episodes"} is still missing. '
            'Recording may still be in progress.'
        )
    return None
