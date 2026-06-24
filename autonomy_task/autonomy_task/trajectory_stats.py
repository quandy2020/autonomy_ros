"""Trajectory / episode statistics helpers."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Protocol


class CollectedRecord(Protocol):
    waypoint_id: str
    robot: str
    at: str
    x: float
    y: float
    yaw: float
    path_length_m: float
    straight_line_m: float
    duration_sec: float


def summarize_records(records: list[CollectedRecord]) -> dict[str, Any]:
    """Aggregate path length and duration over collected trajectory records."""
    episodes = [
        {
            'waypoint_id': r.waypoint_id,
            'robot': r.robot,
            'collected_at': r.at,
            'goal': {'x': r.x, 'y': r.y, 'yaw': r.yaw},
            'path_length_m': round(r.path_length_m, 3),
            'straight_line_m': round(r.straight_line_m, 3),
            'duration_sec': round(r.duration_sec, 2),
        }
        for r in records
    ]
    by_robot: dict[str, list[CollectedRecord]] = {}
    for r in records:
        by_robot.setdefault(r.robot, []).append(r)

    per_robot: dict[str, Any] = {}
    for robot, recs in sorted(by_robot.items()):
        paths = [x.path_length_m for x in recs]
        durs = [x.duration_sec for x in recs]
        per_robot[robot] = {
            'count': len(recs),
            'total_path_m': round(sum(paths), 3),
            'avg_path_m': round(sum(paths) / len(paths), 3) if recs else 0.0,
            'total_duration_sec': round(sum(durs), 2),
            'avg_duration_sec': round(sum(durs) / len(durs), 2) if recs else 0.0,
        }

    paths = [r.path_length_m for r in records]
    durs = [r.duration_sec for r in records]
    return {
        'episode_count': len(records),
        'total_path_m': round(sum(paths), 3),
        'avg_path_m': round(sum(paths) / len(records), 3) if records else 0.0,
        'total_duration_sec': round(sum(durs), 2),
        'avg_duration_sec': round(sum(durs) / len(records), 2) if records else 0.0,
        'per_robot': per_robot,
        'episodes': episodes,
    }


def lerobot_dataset_stats(dataset_root: str | Path) -> dict[str, Any] | None:
    """Read LeRobot meta/info.json and per-episode frame counts from parquet."""
    root = Path(dataset_root)
    info_path = root / 'meta' / 'info.json'
    if not info_path.is_file():
        return None
    try:
        info = json.loads(info_path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError):
        return None
    fps = float(info.get('fps', 0) or 0)
    total_frames = int(info.get('total_frames', 0) or 0)
    total_episodes = int(info.get('total_episodes', 0) or 0)
    episodes = _lerobot_episode_lengths(root, info, fps)
    return {
        'total_episodes': total_episodes,
        'total_frames': total_frames,
        'fps': fps,
        'duration_sec': round(total_frames / fps, 2) if fps > 0 else None,
        'episodes': episodes,
    }


def _lerobot_episode_lengths(
    root: Path, _info: dict[str, Any], fps: float,
) -> list[dict[str, Any]]:
    """Count frames per episode_index from parquet data files."""
    try:
        import pyarrow.parquet as pq
    except ImportError:
        return []

    parquet_files = sorted((root / 'data').rglob('*.parquet'))
    if not parquet_files:
        return []

    counts: dict[int, int] = {}
    for path in parquet_files:
        try:
            table = pq.read_table(path, columns=['episode_index'])
        except (OSError, KeyError):
            continue
        col = table.column('episode_index').to_pylist()
        for ep in col:
            idx = int(ep)
            counts[idx] = counts.get(idx, 0) + 1

    out: list[dict[str, Any]] = []
    for ep_idx in sorted(counts):
        frames = counts[ep_idx]
        out.append({
            'episode_index': ep_idx,
            'frames': frames,
            'duration_sec': round(frames / fps, 2) if fps > 0 else None,
        })
    return out


def format_report(
    state: Any,
    *,
    dataset_roots: dict[str, str] | None = None,
) -> str:
    """Human-readable trajectory statistics report."""
    traj = summarize_records(state.collected)
    lines = [
        f'采集轨迹: {traj["episode_count"]} 条',
        f'总路径长度: {traj["total_path_m"]:.2f} m  (平均 {traj["avg_path_m"]:.2f} m/条)',
        f'总耗时: {traj["total_duration_sec"]:.1f} s  (平均 {traj["avg_duration_sec"]:.1f} s/条)',
        '',
    ]
    for robot, stats in traj['per_robot'].items():
        lines.append(
            f'[{robot}] {stats["count"]} 条, '
            f'路径 {stats["total_path_m"]:.2f} m, '
            f'耗时 {stats["total_duration_sec"]:.1f} s')
    if traj['episodes']:
        lines.extend(['', '每条轨迹:'])
        for i, ep in enumerate(traj['episodes'], 1):
            lines.append(
                f'  {i:3d}. {ep["robot"]} {ep["waypoint_id"]}: '
                f'path={ep["path_length_m"]:.2f}m '
                f'straight={ep["straight_line_m"]:.2f}m '
                f'duration={ep["duration_sec"]:.1f}s')
    if dataset_roots:
        lines.extend(['', 'LeRobot 数据集:'])
        for robot, root in sorted(dataset_roots.items()):
            ds = lerobot_dataset_stats(root)
            if ds is None:
                lines.append(f'  {robot}: (无 meta/info.json) @ {root}')
                continue
            dur = ds['duration_sec']
            dur_s = f'{dur:.1f}s' if dur is not None else 'n/a'
            lines.append(
                f'  {robot}: {ds["total_episodes"]} episodes, '
                f'{ds["total_frames"]} frames, ~{dur_s} @ {root}')
            for ep in ds.get('episodes', []):
                ep_dur = ep.get('duration_sec')
                ep_dur_s = f'{ep_dur:.1f}s' if ep_dur is not None else 'n/a'
                lines.append(
                    f'      ep{ep["episode_index"]}: '
                    f'{ep["frames"]} frames (~{ep_dur_s})')
    return '\n'.join(lines)
