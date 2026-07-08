# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0
"""Visualize a local LeRobot collection dataset (offline, data_paths-aware)."""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path


def _format_dataset_error(
    robot: str,
    root: Path,
    collection_root: Path,
) -> str:
    from autonomy_lerobot.data_paths import collection_state_file, list_collection_robots

    lines = [
        f'error: dataset not found at {root}',
        f'  missing: {root / "meta" / "info.json"}',
        f'  collection root: {collection_root}',
    ]

    available = list_collection_robots(collection_root)
    if available:
        lines.append('  available robots:')
        for name, path in available.items():
            lines.append(f'    - {name}: {path}')
    else:
        lines.append('  available robots: (none — collection directory is empty)')

    state_path = collection_state_file()
    if state_path.is_file():
        try:
            state = json.loads(state_path.read_text(encoding='utf-8'))
            robots_in_state = sorted({
                str(item.get('robot', ''))
                for item in state.get('collected', [])
                if item.get('robot')
            })
            if robot in robots_in_state:
                lines.append(
                    f'  note: {state_path} records trajectories for {robot!r}, '
                    'but the LeRobot dataset directory was removed. '
                    'Re-run collection or restore the dataset from backup.'
                )
            elif robots_in_state:
                lines.append(
                    f'  note: {state_path} has trajectories for: '
                    f'{", ".join(robots_in_state)}'
                )
        except (OSError, json.JSONDecodeError, TypeError):
            pass

    lines.extend([
        '  record new data:',
        '    ros2 launch autonomy_task multi_robot_collection.launch.py',
        '  then visualize:',
        f'    ros2 run autonomy_lerobot lerobot_dataset_viz --robot <name> --episode-index 0',
        '  override path:',
        f'    ros2 run autonomy_lerobot lerobot_dataset_viz --root <dataset_dir> --episode-index 0',
    ])
    return '\n'.join(lines)


def _running_in_docker() -> bool:
    return Path('/.dockerenv').is_file()


def _default_viz_output_dir() -> Path:
    from autonomy_lerobot.data_paths import data_root

    return data_root() / 'lerobot' / 'viz'


def _parse_viz_options(extra: list[str]) -> dict:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--mode', default='local', choices=('local', 'distant'))
    parser.add_argument('--save-dir', default='')
    parser.add_argument('--output-dir', default='')
    parser.add_argument('--save', type=int, default=0)
    parser.add_argument('--num-workers', type=int, default=None)
    parser.add_argument('--batch-size', type=int, default=32)
    parser.add_argument('--grpc-port', type=int, default=9876)
    parser.add_argument('--web-port', type=int, default=9090)
    parser.add_argument('--tolerance-s', type=float, default=1e-4)
    parser.add_argument('--display-compressed-images', action='store_true')
    known, remaining = parser.parse_known_args(extra)
    if remaining:
        print(f'warning: ignoring unknown viz args: {" ".join(remaining)}', file=sys.stderr)
    return vars(known)


def _autosave_rrd(repo_id: str, episode_index: int, output_dir: Path) -> Path:
    import rerun as rr

    output_dir.mkdir(parents=True, exist_ok=True)
    rrd_path = output_dir / f'{repo_id.replace("/", "_")}_episode_{episode_index}.rrd'
    rr.save(rrd_path)
    return rrd_path


def _run_viz(
    repo_id: str,
    root: Path,
    episode_index: int,
    extra: list[str],
) -> int:
    from autonomy_lerobot.video_decode import patch_lerobot_video_decode

    patch_lerobot_video_decode()

    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.scripts.lerobot_dataset_viz import init_logging, visualize_dataset

    opts = _parse_viz_options(extra)

    if opts['num_workers'] is None:
        opts['num_workers'] = 0 if _running_in_docker() else 4

    output_dir = opts['output_dir'] or opts['save_dir']
    save = bool(opts['save']) or bool(output_dir)
    if save and not output_dir:
        output_dir = str(_default_viz_output_dir())

    init_logging()
    print(f'Loading dataset from {root}', file=sys.stderr)
    dataset = LeRobotDataset(
        repo_id,
        episodes=[episode_index],
        root=str(root),
        tolerance_s=opts['tolerance_s'],
        video_backend='pyav',
    )

    viz_kwargs = {
        'episode_index': episode_index,
        'batch_size': opts['batch_size'],
        'num_workers': opts['num_workers'],
        'mode': opts['mode'],
        'grpc_port': opts['grpc_port'],
        'web_port': opts['web_port'],
        'save': save,
        'output_dir': Path(output_dir) if output_dir else None,
        'display_compressed_images': opts['display_compressed_images'],
    }

    if opts['mode'] == 'distant':
        print(
            f'Rerun distant mode: connect from host with\n'
            f'  rerun rerun+http://<container-ip>:{opts["grpc_port"]}/proxy',
            file=sys.stderr,
        )

    visualize_dataset(dataset, **viz_kwargs)

    if (
        _running_in_docker()
        and opts['mode'] == 'local'
        and not save
    ):
        rrd_path = _autosave_rrd(repo_id, episode_index, _default_viz_output_dir())
        print(
            f'Docker: replay auto-saved to {rrd_path}\n'
            f'  open on host: rerun {rrd_path}\n'
            '  note: Rerun GUI "Save" is unavailable in Docker; use the path above.',
            file=sys.stderr,
        )
    elif save and output_dir:
        repo_id_str = repo_id.replace('/', '_')
        rrd_path = Path(output_dir) / f'{repo_id_str}_episode_{episode_index}.rrd'
        if rrd_path.is_file():
            print(f'saved replay to {rrd_path}', file=sys.stderr)

    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Visualize LeRobot dataset under data_paths.yaml collection root',
    )
    parser.add_argument(
        '--robot', '-r', default='robot1',
        help='robot subdir name (default: robot1)',
    )
    parser.add_argument(
        '--episode-index', '-e', type=int, default=0,
        help='episode index (default: 0)',
    )
    parser.add_argument(
        '--root', default='',
        help='override dataset directory (default: <collection_root>/<robot>)',
    )
    parser.add_argument(
        '--repo-id', default='',
        help='override LeRobot repo_id (default: per_robot_repo_id from data_paths)',
    )
    parser.add_argument(
        '--list', '-l', action='store_true',
        help='list available robot datasets under collection root and exit',
    )
    parser.add_argument(
        '--no-repair', action='store_true',
        help='do not auto-rebuild missing meta/episodes before visualization',
    )
    parser.add_argument(
        '--mode', choices=('local', 'distant'), default='local',
        help='local: spawn Rerun viewer; distant: gRPC server for host browser',
    )
    parser.add_argument(
        '--save-dir', default='',
        help='export .rrd to this directory (implies --save 1, no viewer spawn)',
    )
    parser.add_argument(
        '--num-workers', type=int, default=None,
        help='DataLoader workers (default: 0 in Docker, 4 otherwise)',
    )
    args, extra = parser.parse_known_args(argv)

    # Must be set before LeRobot imports huggingface_hub.
    os.environ['HF_HUB_OFFLINE'] = '1'

    from autonomy_lerobot.data_paths import (
        collection_repo_id,
        lerobot_collection_root,
        list_collection_robots,
    )
    from autonomy_lerobot.repo_id import per_robot_repo_id

    collection_root = lerobot_collection_root()

    if args.list:
        available = list_collection_robots(collection_root)
        if not available:
            print(f'no datasets under {collection_root}')
            return 1
        for name, path in available.items():
            print(f'{name}\t{path}')
        return 0

    root = Path(args.root).expanduser().resolve() if args.root else (
        collection_root / args.robot
    )
    repo_id = args.repo_id or per_robot_repo_id(collection_repo_id(), args.robot)
    info = root / 'meta' / 'info.json'

    if not info.is_file():
        print(
            _format_dataset_error(args.robot, root, collection_root),
            file=sys.stderr,
        )
        return 1

    from autonomy_lerobot.dataset_health import active_tmp_dirs, ensure_dataset_ready_for_viz

    repair_error = ensure_dataset_ready_for_viz(
        root, auto_repair=not args.no_repair,
    )
    if repair_error:
        print(f'error: {repair_error}', file=sys.stderr)
        return 1

    tmp_dirs = active_tmp_dirs(root)
    if tmp_dirs:
        print(
            f'warning: {root} still has active encoder tmp dirs '
            f'({", ".join(p.name for p in tmp_dirs)}); '
            'visualizing finalized episodes only',
            file=sys.stderr,
        )

    viz_extra = list(extra)
    if args.save_dir:
        viz_extra.extend(['--save', '1', '--output-dir', args.save_dir])
    if args.mode != 'local':
        viz_extra.extend(['--mode', args.mode])
    if args.num_workers is not None:
        viz_extra.extend(['--num-workers', str(args.num_workers)])

    return _run_viz(repo_id, root, args.episode_index, viz_extra)


if __name__ == '__main__':
    raise SystemExit(main())
