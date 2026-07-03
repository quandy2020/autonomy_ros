# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0
"""Visualize a local LeRobot collection dataset (offline, data_paths-aware)."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path


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
    args, extra = parser.parse_known_args(argv)

    # Must be set before LeRobot imports huggingface_hub.
    os.environ['HF_HUB_OFFLINE'] = '1'

    from autonomy_lerobot.data_paths import collection_repo_id, lerobot_collection_root
    from autonomy_lerobot.repo_id import per_robot_repo_id

    root = Path(args.root).expanduser().resolve() if args.root else (
        lerobot_collection_root() / args.robot
    )
    repo_id = args.repo_id or per_robot_repo_id(collection_repo_id(), args.robot)
    info = root / 'meta' / 'info.json'

    if not info.is_file():
        print(
            f'error: dataset not found at {root}\n'
            f'  missing: {info}\n'
            f'  recordings are written to: {lerobot_collection_root()}/<robot>/\n'
            f'  (see config/data_paths.yaml volume_root; do not use workspace/data/ unless symlinked)',
            file=sys.stderr,
        )
        return 1

    viz_argv = [
        'lerobot-dataset-viz',
        '--repo-id', repo_id,
        '--root', str(root),
        '--episode-index', str(args.episode_index),
        *extra,
    ]
    sys.argv = viz_argv
    from lerobot.scripts.lerobot_dataset_viz import main as viz_main

    viz_main()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
