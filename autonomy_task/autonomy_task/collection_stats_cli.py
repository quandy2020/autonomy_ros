"""CLI: print trajectory collection statistics."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from autonomy_task.state import load
from autonomy_task.trajectory_stats import format_report, lerobot_dataset_stats


def _default_state_path() -> Path:
    return Path('/workspace/autonomy/data/collection/state.json')


def _default_dataset_root() -> Path:
    return Path('/workspace/autonomy/data/lerobot/collection')


def _discover_dataset_roots(root: Path) -> dict[str, str]:
    if not root.is_dir():
        return {}
    out: dict[str, str] = {}
    for child in sorted(root.iterdir()):
        if child.is_dir() and (child / 'meta' / 'info.json').is_file():
            out[child.name] = str(child)
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='统计采集轨迹条数、路径长度、耗时（state.json + LeRobot meta）')
    parser.add_argument(
        '--state', type=Path, default=_default_state_path(),
        help='collection state.json 路径')
    parser.add_argument(
        '--dataset-root', type=Path, default=_default_dataset_root(),
        help='LeRobot 多机数据集根目录（含 robot1/ robot2/ 子目录）')
    parser.add_argument(
        '--json', action='store_true', help='输出 JSON 而非可读报告')
    args = parser.parse_args(argv)

    state = load(args.state)
    dataset_roots = _discover_dataset_roots(args.dataset_root)

    if args.json:
        payload = {
            'state_file': str(args.state),
            'trajectories': state.trajectory_summary(),
            'lerobot': {
                robot: lerobot_dataset_stats(path)
                for robot, path in dataset_roots.items()
            },
        }
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return 0

    print(format_report(state, dataset_roots=dataset_roots or None))
    return 0


if __name__ == '__main__':
    sys.exit(main())
