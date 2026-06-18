#!/usr/bin/env python3
"""Encode leftover LeRobot PNG episode folders to MP4.

LeRobot writes temporary PNG frames under images/ during recording. They are
normally converted to videos/.../*.mp4 when save_episode() runs. Use this tool
if that step was skipped or the bridge node exited before saving.
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path


def _fps_from_info(dataset_root: Path, default: int) -> int:
    info_path = dataset_root / 'meta' / 'info.json'
    if not info_path.is_file():
        return default
    info = json.loads(info_path.read_text(encoding='utf-8'))
    return int(info.get('fps', default))


def encode_pending_videos(
    dataset_root: Path,
    fps: int,
    vcodec: str,
    delete_png: bool,
) -> list[Path]:
    from lerobot.datasets.video_utils import encode_video_frames

    images_root = dataset_root / 'images'
    if not images_root.is_dir():
        raise FileNotFoundError(f'No images/ directory under {dataset_root}')

    written: list[Path] = []
    for video_key_dir in sorted(images_root.iterdir()):
        if not video_key_dir.is_dir():
            continue
        video_key = video_key_dir.name
        for ep_dir in sorted(video_key_dir.glob('episode-*')):
            pngs = list(ep_dir.glob('frame-*.png'))
            if not pngs:
                continue
            ep_idx = int(ep_dir.name.rsplit('-', 1)[-1])
            out = (
                dataset_root / 'videos' / video_key
                / 'chunk-000' / f'file-{ep_idx:03d}.mp4'
            )
            if out.is_file():
                print(f'skip existing {out}')
                continue
            print(f'encoding {len(pngs)} frames: {ep_dir} -> {out}')
            encode_video_frames(
                ep_dir, out, fps=fps, vcodec=vcodec, overwrite=True)
            written.append(out)
            if delete_png:
                shutil.rmtree(ep_dir)
                print(f'deleted {ep_dir}')
    return written


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Convert LeRobot images/*/episode-* PNG folders to MP4.')
    parser.add_argument(
        'dataset_root', type=Path,
        help='LeRobot dataset root (contains meta/info.json and images/)')
    parser.add_argument('--fps', type=int, default=None,
                        help='FPS (default: read from meta/info.json or 10)')
    parser.add_argument('--vcodec', default='h264',
                        help='Video codec (default: h264)')
    parser.add_argument(
        '--delete-png', action='store_true',
        help='Remove PNG episode folder after successful encoding')
    args = parser.parse_args(argv)

    dataset_root = args.dataset_root.expanduser().resolve()
    fps = args.fps if args.fps is not None else _fps_from_info(dataset_root, 10)

    try:
        written = encode_pending_videos(
            dataset_root, fps=fps, vcodec=args.vcodec, delete_png=args.delete_png)
    except Exception as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 1

    if not written:
        print('no PNG episodes found to encode')
        return 0

    print(f'wrote {len(written)} mp4 file(s)')
    for path in written:
        print(f'  {path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
