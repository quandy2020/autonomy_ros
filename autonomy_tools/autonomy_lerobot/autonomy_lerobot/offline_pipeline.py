#
# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

"""Stage-oriented offline pipeline, closer to bag2data."""

from __future__ import annotations

import argparse
import json
import logging
from pathlib import Path

from autonomy_lerobot.bag_postprocess import (
    BagPostprocessConfig,
    BagPostprocessor,
    BagPreviewSample,
)
from autonomy_lerobot.bag_to_lerobot import (
    TOPIC_CAMERA_INFO,
    TOPIC_DEPTH,
    TOPIC_RGB,
    _build_arg_parser,
    _config_from_args,
    BagMessageReader,
    iter_sampled_observations,
    clean_stationary_runs,
    run_conversion,
)
from autonomy_lerobot.collection_params import JDROBOT_EPISODE_TIMING, jdrobot_max_episode_seconds
from autonomy_lerobot.episode_split import EpisodeSplitConfig
from autonomy_lerobot.conversions import image_to_numpy
from autonomy_lerobot.sam3_segmenter import Sam3GroundSegmenter


def _build_pipeline_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Offline LeRobot pipeline with explicit convert/clean stages.'
    )
    subparsers = parser.add_subparsers(dest='command', required=True)

    convert_parser = subparsers.add_parser(
        'convert',
        help='convert one bag or a directory tree of bags',
    )
    base_parser = _build_arg_parser()
    for action in base_parser._actions:
        if action.dest == 'help':
            continue
        kwargs = {
            'default': action.default,
            'required': getattr(action, 'required', False),
            'help': action.help,
        }
        if action.option_strings:
            if getattr(action, 'type', None) is not None:
                kwargs['type'] = action.type
            if getattr(action, 'nargs', None) is not None:
                kwargs['nargs'] = action.nargs
            if getattr(action, 'const', None) is not None:
                kwargs['const'] = action.const
            if getattr(action, 'choices', None) is not None:
                kwargs['choices'] = action.choices
            if action.__class__.__name__ in {'_StoreTrueAction', '_StoreFalseAction'}:
                kwargs['action'] = 'store_true' if action.default is False else 'store_false'
                kwargs.pop('default', None)
                kwargs.pop('required', None)
                kwargs.pop('type', None)
                kwargs.pop('nargs', None)
                kwargs.pop('const', None)
            convert_parser.add_argument(*action.option_strings, **kwargs)

    clean_parser = subparsers.add_parser(
        'clean',
        help='remove stationary runs from an existing scene root',
    )
    clean_parser.add_argument('--scene-root', required=True, help='scene directory to clean')
    clean_parser.add_argument('--stationary-pos-threshold', type=float, default=0.01)
    clean_parser.add_argument('--stationary-yaw-threshold', type=float, default=0.01)
    clean_parser.add_argument('--dry-run', action='store_true')

    postprocess_parser = subparsers.add_parser(
        'postprocess',
        help='regenerate SAM3 and pointcloud preview artifacts for one scene',
    )
    postprocess_parser.add_argument('--scene-root', required=True, help='scene directory to postprocess')
    postprocess_parser.add_argument('--bag', default='', help='override bag path from meta/info.json')
    postprocess_parser.add_argument('--fps', type=float, default=0.0, help='override fps from scene meta')
    postprocess_parser.add_argument(
        '--episode-seconds',
        type=float,
        default=0.0,
        help='override episode duration from scene meta',
    )
    postprocess_parser.add_argument('--rgb-topic', default=TOPIC_RGB)
    postprocess_parser.add_argument('--depth-topic', default=TOPIC_DEPTH)
    postprocess_parser.add_argument('--camera-info-topic', default=TOPIC_CAMERA_INFO)
    postprocess_parser.add_argument('--pointcloud-stride', type=int, default=4)
    postprocess_parser.add_argument('--max-pointcloud-points', type=int, default=4096)
    postprocess_parser.add_argument('--depth-min-m', type=float, default=0.0)
    postprocess_parser.add_argument('--depth-max-m', type=float, default=10.0)
    postprocess_parser.add_argument('--sam3-device', default='cuda')
    postprocess_parser.add_argument('--sam3-confidence', type=float, default=0.3)
    postprocess_parser.add_argument('--sam3-checkpoint', default='')
    postprocess_parser.add_argument('--sam3-prompts', nargs='+', default=[])
    postprocess_parser.add_argument(
        '--no-ply',
        action='store_true',
        help='skip exporting filtered pointcloud/frame_xxxx.ply',
    )
    return parser


def _load_scene_info(scene_root: Path) -> dict:
    info_path = scene_root / 'meta' / 'info.json'
    if not info_path.is_file():
        raise FileNotFoundError(f'scene info not found: {info_path}')
    data = json.loads(info_path.read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError(f'invalid scene info: {info_path}')
    return data


def _build_segmenter(args: argparse.Namespace) -> Sam3GroundSegmenter:
    prompts = tuple(args.sam3_prompts) if args.sam3_prompts else Sam3GroundSegmenter.DEFAULT_PROMPTS
    return Sam3GroundSegmenter(
        device=args.sam3_device,
        confidence=args.sam3_confidence,
        prompts=prompts,
        checkpoint_path=args.sam3_checkpoint or None,
    )


def _run_postprocess(args: argparse.Namespace, logger: logging.Logger) -> None:
    scene_root = Path(args.scene_root).expanduser().resolve()
    scene_info = _load_scene_info(scene_root)
    bag_path_str = args.bag or str(scene_info.get('bag_path', '')).strip()
    if not bag_path_str or bag_path_str == 'None':
        raise ValueError('postprocess requires --bag or a valid meta/info.json bag_path')
    bag_path = Path(bag_path_str).expanduser().resolve()
    if not bag_path.is_file():
        raise FileNotFoundError(f'bag not found: {bag_path}')

    fps = float(args.fps or scene_info.get('fps', 0.0))
    episode_seconds = float(args.episode_seconds or scene_info.get('episode_seconds', 0.0))
    episode_split_mode = str(
        getattr(args, 'episode_split_mode', '') or scene_info.get('episode_split_mode', 'nav')
    ).strip() or 'nav'
    if fps <= 0.0:
        raise ValueError('postprocess requires positive fps')
    if episode_seconds <= 0.0:
        episode_seconds = jdrobot_max_episode_seconds()

    split_cfg = EpisodeSplitConfig(
        mode=episode_split_mode,
        record_before_sec=float(
            getattr(args, 'record_before_sec', 0.0)
            or scene_info.get('record_before_sec', JDROBOT_EPISODE_TIMING['record_before_sec'])
        ),
        record_after_sec=float(
            getattr(args, 'record_after_sec', 0.0)
            or scene_info.get('record_after_sec', JDROBOT_EPISODE_TIMING['record_after_sec'])
        ),
        max_nav_sec=float(
            getattr(args, 'max_nav_sec', 0.0)
            or scene_info.get('max_nav_sec', JDROBOT_EPISODE_TIMING['max_nav_sec'])
        ),
        stall_move_m=float(
            getattr(args, 'stall_move_m', 0.0)
            or scene_info.get('stall_move_m', JDROBOT_EPISODE_TIMING['stall_move_m'])
        ),
        episode_seconds=episode_seconds,
    )

    postprocessor = BagPostprocessor(
        BagPostprocessConfig(
            record_pointcloud=True,
            save_segmentation_viz=True,
            pointcloud_stride=args.pointcloud_stride,
            max_pointcloud_points=args.max_pointcloud_points,
            depth_min_m=args.depth_min_m,
            depth_max_m=args.depth_max_m,
        ),
        segmenter=_build_segmenter(args),
    )
    reader = BagMessageReader(
        bag_path,
        {args.rgb_topic, args.depth_topic, args.camera_info_topic},
    )
    current_episode_index: int | None = None
    frame_index_in_episode = 0
    processed_runs = 0
    # Collect every processed frame (bag2data style); collage picks first/mid/last.
    preview_samples: dict[int, list[BagPreviewSample]] = {}

    for observation in iter_sampled_observations(
        reader,
        rgb_topic=args.rgb_topic,
        depth_topic=args.depth_topic,
        camera_info_topic=args.camera_info_topic,
        odom_topic=None,
        odom_fallback_topic=None,
        fps=fps,
        split_cfg=split_cfg,
    ):
        if current_episode_index != observation.episode_index:
            current_episode_index = observation.episode_index
            frame_index_in_episode = 0
        run_index = observation.episode_index
        run_root = scene_root / f'run_{run_index}'
        if not run_root.is_dir():
            frame_index_in_episode += 1
            continue
        sample = postprocessor.build_preview_sample(
            rgb=image_to_numpy(observation.rgb_msg),
            depth=image_to_numpy(observation.depth_msg),
            camera_info=observation.camera_info_msg,
        )
        if not args.no_ply:
            postprocessor.write_pointcloud_ply(
                run_root,
                frame_index=frame_index_in_episode,
                sample=sample,
            )
        preview_samples.setdefault(run_index, []).append(sample)
        frame_index_in_episode += 1

    for run_index, samples in sorted(preview_samples.items()):
        run_root = scene_root / f'run_{run_index}'
        postprocessor.write_preview_samples(run_root, samples)
        n = len(samples)
        selected = (
            [0, n // 2, n - 1] if n >= 3 else list(range(n))
        )
        logger.info(
            'postprocess run=%s frames=%d collage_rows=%s',
            run_root.name,
            n,
            selected,
        )
        processed_runs += 1

    logger.info('postprocess summary: scene=%s processed_runs=%d', scene_root.name, processed_runs)


def main() -> None:
    parser = _build_pipeline_parser()
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')
    logger = logging.getLogger('autonomy_lerobot.offline_pipeline')

    if args.command == 'convert':
        args.defer_postprocess = True
        cfg = _config_from_args(args)
        run_conversion(cfg, logger)
        return
    if args.command == 'postprocess':
        _run_postprocess(args, logger)
        return

    scene_root = Path(args.scene_root).expanduser().resolve()
    summary = clean_stationary_runs(
        scene_root,
        pos_threshold=args.stationary_pos_threshold,
        yaw_threshold=args.stationary_yaw_threshold,
        dry_run=args.dry_run,
        logger=logger,
    )
    logger.info(
        'clean summary: total=%d kept=%d deleted=%d',
        summary['total_runs'],
        summary['kept_runs'],
        len(summary['deleted_runs']),
    )


if __name__ == '__main__':
    main()
