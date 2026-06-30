"""Clean invalid / redundant frames from on-disk LeRobot v3 datasets."""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path
from typing import Any

from autonomy_lerobot.data_paths import lerobot_collection_root

DEFAULT_MIN_FRAMES = 5
DEFAULT_MAX_FRAMES = 600


def _episode_frame_counts(root: Path) -> dict[int, int]:
    import pyarrow.parquet as pq

    counts: dict[int, int] = {}
    data_dir = root / 'data'
    if not data_dir.is_dir():
        return counts
    for path in sorted(data_dir.rglob('*.parquet')):
        try:
            table = pq.read_table(path, columns=['episode_index'])
            for ep in table.column('episode_index').to_pylist():
                idx = int(ep)
                counts[idx] = counts.get(idx, 0) + 1
        except Exception:
            continue
    return counts


def _remove_tmp_dirs(root: Path, dry_run: bool) -> int:
    count = sum(1 for p in root.iterdir() if p.is_dir() and p.name.startswith('tmp'))
    if dry_run:
        if count:
            print(f'  [dry-run] remove {count} tmp dirs')
        return count
    for path in sorted(root.iterdir()):
        if path.is_dir() and path.name.startswith('tmp'):
            shutil.rmtree(path, ignore_errors=True)
    return count


def _load_tables(parquet_paths: list[Path]):
    import pyarrow as pa
    import pyarrow.parquet as pq

    tables = []
    for path in parquet_paths:
        try:
            tables.append(pq.read_table(path))
        except Exception as exc:
            print(f'  warn: skip {path}: {exc}')
    if not tables:
        return None
    return pa.concat_tables(tables, promote_options='default')


def _write_table(table, path: Path) -> None:
    import pyarrow.parquet as pq

    path.parent.mkdir(parents=True, exist_ok=True)
    pq.write_table(table, path)


def _remap_data(root: Path, old_to_new: dict[int, int], dry_run: bool) -> int:
    import pyarrow as pa

    data_dir = root / 'data'
    paths = sorted(data_dir.rglob('*.parquet'))
    if not paths:
        return 0

    table = _load_tables(paths)
    if table is None or table.num_rows == 0:
        return 0

    ep_col = table.column('episode_index').to_pylist()
    keep = [i for i, ep in enumerate(ep_col) if int(ep) in old_to_new]
    if not keep:
        return 0

    filtered = table.take(keep)
    new_eps = [old_to_new[int(ep)] for ep in filtered.column('episode_index').to_pylist()]

    if 'frame_index' in filtered.column_names:
        # Re-number frame_index within each new episode.
        frame_indices: list[int] = []
        per_ep: dict[int, int] = {}
        for ep in new_eps:
            frame_indices.append(per_ep.get(ep, 0))
            per_ep[ep] = per_ep.get(ep, 0) + 1
        filtered = filtered.set_column(
            filtered.schema.get_field_index('frame_index'),
            'frame_index',
            pa.array(frame_indices, type=filtered.schema.field('frame_index').type),
        )

    filtered = filtered.set_column(
        filtered.schema.get_field_index('episode_index'),
        'episode_index',
        pa.array(new_eps, type=filtered.schema.field('episode_index').type),
    )

    # Global row index 0..N-1 sorted by episode then frame.
    ep_list = filtered.column('episode_index').to_pylist()
    fi_list = (
        filtered.column('frame_index').to_pylist()
        if 'frame_index' in filtered.column_names else list(range(len(ep_list)))
    )
    order = sorted(range(len(ep_list)), key=lambda i: (ep_list[i], fi_list[i]))
    filtered = filtered.take(order)
    if 'index' in filtered.column_names:
        filtered = filtered.set_column(
            filtered.schema.get_field_index('index'),
            'index',
            pa.array(range(filtered.num_rows), type=filtered.schema.field('index').type),
        )

    if dry_run:
        print(f'  [dry-run] rewrite data: {filtered.num_rows} frames, {len(old_to_new)} episodes')
        return filtered.num_rows

    for path in paths:
        if path.is_file():
            path.unlink()
    for chunk in data_dir.iterdir():
        if chunk.is_dir():
            shutil.rmtree(chunk, ignore_errors=True)

    out = data_dir / 'chunk-000' / 'file-000.parquet'
    _write_table(filtered, out)
    return filtered.num_rows


def _video_keys_from_info(root: Path) -> list[str]:
    info_path = root / 'meta' / 'info.json'
    if not info_path.is_file():
        return []
    info = json.loads(info_path.read_text(encoding='utf-8'))
    return [
        key for key, spec in info.get('features', {}).items()
        if isinstance(spec, dict) and spec.get('dtype') == 'video'
    ]


def _episode_lengths_from_counts(
    counts: dict[int, int], old_to_new: dict[int, int],
) -> dict[int, int]:
    return {new: counts[old] for old, new in old_to_new.items()}


def _remove_corrupt_parquet(directory: Path, dry_run: bool) -> int:
    import pyarrow.parquet as pq

    removed = 0
    if not directory.is_dir():
        return 0
    for path in sorted(directory.rglob('*.parquet')):
        try:
            pq.read_schema(path)
        except Exception:
            if dry_run:
                print(f'  [dry-run] remove corrupt parquet {path}')
            else:
                path.unlink(missing_ok=True)
            removed += 1
    return removed


def _rebuild_meta_from_lengths(
    root: Path,
    episode_lengths: dict[int, int],
    *,
    fps: float,
    dry_run: bool,
) -> int:
    """Rebuild meta/episodes when parquet meta is missing or incomplete."""
    import pyarrow as pa

    if not episode_lengths:
        return 0

    video_keys = _video_keys_from_info(root)
    episodes_dir = root / 'meta' / 'episodes'
    rows: list[dict[str, Any]] = []
    cum_ts = 0.0

    for ep in sorted(episode_lengths):
        length = int(episode_lengths[ep])
        duration = length / fps if fps > 0 else 0.0
        row: dict[str, Any] = {
            'episode_index': ep,
            'tasks': ['navigate to goal'],
            'length': length,
            'data/chunk_index': 0,
            'data/file_index': 0,
        }
        for vk in video_keys:
            per_ep_mp4 = root / 'videos' / vk / 'chunk-000' / f'file-{ep:03d}.mp4'
            if per_ep_mp4.is_file():
                file_idx = ep
                from_ts, to_ts = 0.0, duration
            else:
                file_idx = 0
                from_ts, to_ts = cum_ts, cum_ts + duration
            row[f'videos/{vk}/chunk_index'] = 0
            row[f'videos/{vk}/file_index'] = file_idx
            row[f'videos/{vk}/from_timestamp'] = from_ts
            row[f'videos/{vk}/to_timestamp'] = to_ts
        if not all(
            (root / 'videos' / vk / 'chunk-000' / f'file-{ep:03d}.mp4').is_file()
            for vk in video_keys
        ):
            cum_ts += duration
        rows.append(row)

    if dry_run:
        print(f'  [dry-run] rebuild meta/episodes: {len(rows)} rows')
        return len(rows)

    _remove_corrupt_parquet(episodes_dir, dry_run=False)
    for chunk in episodes_dir.iterdir():
        if chunk.is_dir():
            shutil.rmtree(chunk, ignore_errors=True)
    for path in episodes_dir.rglob('*.parquet'):
        if path.is_file():
            path.unlink()

    table = pa.Table.from_pylist(rows)
    out = episodes_dir / 'chunk-000' / 'file-000.parquet'
    _write_table(table, out)
    return len(rows)


def _remap_meta_episodes(root: Path, old_to_new: dict[int, int], dry_run: bool) -> int:
    episodes_dir = root / 'meta' / 'episodes'
    _remove_corrupt_parquet(episodes_dir, dry_run)
    paths = sorted(episodes_dir.rglob('*.parquet'))
    if not paths:
        return 0

    table = _load_tables(paths)
    if table is None or table.num_rows == 0:
        return 0

    ep_col = table.column('episode_index').to_pylist()
    keep = [i for i, ep in enumerate(ep_col) if int(ep) in old_to_new]
    if not keep:
        return 0

    filtered = table.take(keep)
    new_eps = [old_to_new[int(ep)] for ep in filtered.column('episode_index').to_pylist()]

    import pyarrow as pa
    filtered = filtered.set_column(
        filtered.schema.get_field_index('episode_index'),
        'episode_index',
        pa.array(new_eps, type=filtered.schema.field('episode_index').type),
    )

    if dry_run:
        print(f'  [dry-run] rewrite meta/episodes: {filtered.num_rows} rows')
        return filtered.num_rows

    for path in paths:
        if path.is_file():
            path.unlink()
    for chunk in episodes_dir.iterdir():
        if chunk.is_dir():
            shutil.rmtree(chunk, ignore_errors=True)

    out = episodes_dir / 'chunk-000' / 'file-000.parquet'
    _write_table(filtered, out)
    return filtered.num_rows


def _referenced_videos(root: Path) -> set[tuple[str, int, int]]:
    import pyarrow.parquet as pq

    refs: set[tuple[str, int, int]] = set()
    episodes_dir = root / 'meta' / 'episodes'
    for path in episodes_dir.rglob('*.parquet'):
        try:
            table = pq.read_table(path)
        except Exception:
            continue
        for row in table.to_pylist():
            for key, value in row.items():
                if not key.startswith('videos/') or not key.endswith('/chunk_index'):
                    continue
                vid_key = key[len('videos/'):-len('/chunk_index')]
                chunk_idx = int(value)
                file_key = f'videos/{vid_key}/file_index'
                file_idx = int(row.get(file_key, 0))
                refs.add((vid_key, chunk_idx, file_idx))
    return refs


def _prune_orphan_videos(root: Path, dry_run: bool) -> int:
    refs = _referenced_videos(root)
    removed = 0
    videos_dir = root / 'videos'
    if not videos_dir.is_dir():
        return 0
    for path in sorted(videos_dir.rglob('*.mp4')):
        # path: videos/{key}/chunk-XXX/file-YYY.mp4
        try:
            vid_key = path.parent.parent.name
            chunk_idx = int(path.parent.name.split('-')[1])
            file_idx = int(path.stem.split('-')[1])
        except (IndexError, ValueError):
            continue
        if (vid_key, chunk_idx, file_idx) not in refs:
            if dry_run:
                print(f'  [dry-run] remove orphan video {path}')
            else:
                path.unlink(missing_ok=True)
            removed += 1
    return removed


def _update_info(root: Path, num_episodes: int, num_frames: int, dry_run: bool) -> None:
    info_path = root / 'meta' / 'info.json'
    if not info_path.is_file():
        return
    info = json.loads(info_path.read_text(encoding='utf-8'))
    info['total_episodes'] = num_episodes
    info['total_frames'] = num_frames
    info['splits'] = {'train': f'0:{num_episodes}'}
    if dry_run:
        print(f'  [dry-run] info.json -> episodes={num_episodes} frames={num_frames}')
        return
    info_path.write_text(json.dumps(info, indent=2) + '\n', encoding='utf-8')


def _remove_stats(root: Path, dry_run: bool) -> None:
    stats_path = root / 'meta' / 'stats.json'
    if stats_path.is_file():
        if dry_run:
            print(f'  [dry-run] remove {stats_path} (will regenerate on load)')
        else:
            stats_path.unlink(missing_ok=True)


def cleanup_dataset(
    root: str | Path,
    *,
    min_frames: int = DEFAULT_MIN_FRAMES,
    max_frames: int = DEFAULT_MAX_FRAMES,
    dry_run: bool = False,
) -> dict[str, Any]:
    """Remove tmp dirs, invalid episodes, orphans; reindex episodes 0..N-1."""
    root = Path(root)
    report: dict[str, Any] = {'root': str(root), 'dry_run': dry_run}

    if not (root / 'meta' / 'info.json').is_file():
        report['skipped'] = 'not a LeRobot dataset'
        return report

    report['tmp_dirs_removed'] = _remove_tmp_dirs(root, dry_run)

    counts = _episode_frame_counts(root)
    report['episodes_before'] = len(counts)
    report['frames_before'] = sum(counts.values())

    invalid = {
        ep: n for ep, n in counts.items()
        if n < min_frames or n > max_frames
    }
    report['invalid_episodes'] = {str(k): v for k, v in sorted(invalid.items())}

    valid_eps = sorted(ep for ep, n in counts.items() if min_frames <= n <= max_frames)
    old_to_new = {old: i for i, old in enumerate(valid_eps)}
    report['episodes_after'] = len(valid_eps)

    if not valid_eps:
        report['skipped'] = 'no valid episodes'
        return report

    frames_after = _remap_data(root, old_to_new, dry_run)
    meta_rows = _remap_meta_episodes(root, old_to_new, dry_run)
    meta_rebuilt = False

    info = json.loads((root / 'meta' / 'info.json').read_text(encoding='utf-8'))
    fps = float(info.get('fps', 10) or 10)
    ep_lengths = _episode_lengths_from_counts(counts, old_to_new)
    if meta_rows < len(valid_eps):
        meta_rows = _rebuild_meta_from_lengths(
            root, ep_lengths, fps=fps, dry_run=dry_run)
        meta_rebuilt = True

    report['frames_after'] = frames_after
    report['meta_rows'] = meta_rows
    report['meta_rebuilt'] = meta_rebuilt
    report['orphan_videos_removed'] = _prune_orphan_videos(root, dry_run)
    _update_info(root, len(valid_eps), frames_after, dry_run)
    _remove_stats(root, dry_run)

    return report


def cleanup_collection(
    collection_root: str | Path,
    *,
    min_frames: int = DEFAULT_MIN_FRAMES,
    max_frames: int = DEFAULT_MAX_FRAMES,
    dry_run: bool = False,
) -> list[dict[str, Any]]:
    root = Path(collection_root)
    reports: list[dict[str, Any]] = []
    if not root.is_dir():
        return reports
    for child in sorted(root.iterdir()):
        if child.is_dir() and (child / 'meta' / 'info.json').is_file():
            print(f'=== {child.name} ===')
            report = cleanup_dataset(
                child,
                min_frames=min_frames,
                max_frames=max_frames,
                dry_run=dry_run,
            )
            reports.append(report)
            print(json.dumps(report, indent=2, ensure_ascii=False))
    return reports


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Clean invalid LeRobot collection datasets')
    parser.add_argument(
        '--root', type=Path,
        default=lerobot_collection_root(),
        help='Collection root with robot1/ robot2/ ...',
    )
    parser.add_argument('--min-frames', type=int, default=DEFAULT_MIN_FRAMES)
    parser.add_argument('--max-frames', type=int, default=DEFAULT_MAX_FRAMES)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)

    cleanup_collection(
        args.root,
        min_frames=args.min_frames,
        max_frames=args.max_frames,
        dry_run=args.dry_run,
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
