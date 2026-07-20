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

"""Convert ROS2 MCAP bags into LeRobot datasets."""

from __future__ import annotations

import argparse
import json
import logging
import shutil
from collections import defaultdict, deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from math import atan2
from pathlib import Path
from typing import Any, Iterable

import cv2
import numpy as np

from autonomy_lerobot.bag_postprocess import BagPostprocessConfig, BagPostprocessor
from autonomy_lerobot.collection_params import JDROBOT_COLLECTION_ROS_PARAMS
from autonomy_lerobot.conversions import (
    camera_info_to_intrinsic,
    depth_to_video_rgb,
    identity_extrinsic,
    image_to_numpy,
    odom_to_action_matrix,
)
from autonomy_lerobot.observation import (
    KEY_ACTION,
    KEY_CAMERA_EXTRINSIC,
    KEY_CAMERA_INTRINSIC,
    KEY_DEPTH,
    KEY_POINTCLOUD,
    KEY_RGB,
    KEY_SEMANTIC,
    KEY_TASK,
)
from autonomy_lerobot.recorder import DatasetRecorder
from autonomy_lerobot.sam3_segmenter import Sam3GroundSegmenter

TOPIC_RGB = '/camera/color/image_raw'
TOPIC_DEPTH = '/camera/depth/image_raw'
TOPIC_CAMERA_INFO = '/camera/depth/camera_info'
TOPIC_ODOM = '/odom'
TOPIC_ODOM_EKF = '/robot/state_cmd/ekf'


@dataclass(frozen=True)
class ConvertConfig:
    """Offline bag conversion configuration."""

    bag_path: Path | None
    input_dir: Path | None
    output_root: Path
    scene_name: str
    repo_id: str
    robot_type: str
    task: str
    fps: float
    episode_seconds: float
    video_vcodec: str
    overwrite: bool
    rgb_topic: str = TOPIC_RGB
    depth_topic: str = TOPIC_DEPTH
    camera_info_topic: str = TOPIC_CAMERA_INFO
    odom_topic: str = TOPIC_ODOM
    odom_fallback_topic: str = TOPIC_ODOM_EKF
    pointcloud_stride: int = 4
    max_pointcloud_points: int = 4096
    depth_min_m: float = 0.0
    depth_max_m: float = 10.0
    record_pointcloud: bool = True
    sam3_enable: bool = False
    sam3_device: str = 'cuda'
    sam3_confidence: float = 0.3
    sam3_checkpoint: str = ''
    sam3_prompts: tuple[str, ...] = ()
    save_segmentation_viz: bool = False
    inline_postprocess: bool = True
    skip_stationary_runs: bool = False
    stationary_pos_threshold: float = 0.01
    stationary_yaw_threshold: float = 0.01
    dry_run_clean: bool = False


@dataclass
class BufferedTopics:
    """Recent bag messages per topic."""

    messages: dict[str, deque[tuple[float, Any]]] = field(
        default_factory=lambda: defaultdict(lambda: deque(maxlen=32))
    )

    def append(self, topic: str, stamp: float, msg: Any) -> None:
        self.messages[topic].append((stamp, msg))

    def nearest(self, topic: str, stamp: float) -> Any | None:
        buf = self.messages.get(topic)
        if not buf:
            return None
        return min(buf, key=lambda item: abs(item[0] - stamp))[1]


@dataclass(frozen=True)
class SampledObservation:
    """Aligned observations for one sampled timestamp."""

    sample_ts: float
    episode_index: int
    rgb_msg: Any
    depth_msg: Any
    camera_info_msg: Any
    odom_msg: Any | None


class BagMessageReader:
    """Streaming ROS2 MCAP reader."""

    def __init__(self, bag_path: Path, topics: Iterable[str]) -> None:
        self._bag_path = bag_path
        self._topics = set(topics)

    def iter_messages(self) -> Iterable[tuple[str, float, Any]]:
        try:
            from mcap.reader import make_reader
            from mcap_ros2.decoder import DecoderFactory
        except ImportError as exc:  # pragma: no cover - runtime dependency
            raise RuntimeError(
                'mcap support missing. Run `pip3 install mcap mcap-ros2-support`.'
            ) from exc

        factory = DecoderFactory()
        with self._bag_path.open('rb') as handle:
            reader = make_reader(handle, decoder_factories=[factory])
            for schema, channel, message in reader.iter_messages():
                if channel.topic not in self._topics:
                    continue
                decoder = factory.decoder_for(channel.message_encoding, schema)
                if decoder is None:
                    continue
                try:
                    yield channel.topic, message.log_time / 1e9, decoder(message.data)
                except Exception:
                    continue


def iter_sampled_observations(
    reader: BagMessageReader,
    *,
    rgb_topic: str,
    depth_topic: str,
    camera_info_topic: str,
    odom_topic: str | None,
    odom_fallback_topic: str | None,
    fps: float,
    episode_seconds: float,
) -> Iterable[SampledObservation]:
    """Yield aligned observations sampled on the RGB timeline."""
    buffers = BufferedTopics()
    episode_start_ts: float | None = None
    next_sample_ts: float | None = None
    episode_index = 0
    frames_in_episode = 0
    sample_period = 1.0 / fps

    for topic, stamp, msg in reader.iter_messages():
        buffers.append(topic, stamp, msg)
        if topic != rgb_topic:
            continue
        if next_sample_ts is None:
            next_sample_ts = stamp
        while next_sample_ts is not None and stamp >= next_sample_ts:
            sample_ts = next_sample_ts
            next_sample_ts += sample_period
            rgb_msg = buffers.nearest(rgb_topic, sample_ts)
            depth_msg = buffers.nearest(depth_topic, sample_ts)
            camera_info_msg = buffers.nearest(camera_info_topic, sample_ts)
            odom_msg = None
            if odom_topic is not None:
                odom_msg = buffers.nearest(odom_topic, sample_ts)
                if odom_msg is None and odom_fallback_topic is not None:
                    odom_msg = buffers.nearest(odom_fallback_topic, sample_ts)
            if rgb_msg is None or depth_msg is None or camera_info_msg is None:
                continue
            if episode_start_ts is None:
                episode_start_ts = sample_ts
            if (
                episode_seconds > 0.0
                and episode_start_ts is not None
                and (sample_ts - episode_start_ts) >= episode_seconds
                and frames_in_episode > 0
            ):
                episode_index += 1
                episode_start_ts = sample_ts
                frames_in_episode = 0
            yield SampledObservation(
                sample_ts=sample_ts,
                episode_index=episode_index,
                rgb_msg=rgb_msg,
                depth_msg=depth_msg,
                camera_info_msg=camera_info_msg,
                odom_msg=odom_msg,
            )
            frames_in_episode += 1


def _yaw_from_pose_matrix(flatten_pose: np.ndarray) -> float:
    mat = flatten_pose.reshape(4, 4)
    return float(atan2(mat[1, 0], mat[0, 0]))


def _run_action_parquet(run_dir: Path) -> Path:
    return run_dir / 'data' / 'chunk-000' / 'file-000.parquet'


def detect_stationary_run(
    run_dir: Path,
    *,
    pos_threshold: float,
    yaw_threshold: float,
) -> dict[str, Any]:
    """Detect whether one run has almost no translation / yaw variation."""
    parquet_path = _run_action_parquet(run_dir)
    if not parquet_path.is_file():
        return {'run_dir': run_dir, 'is_stationary': False, 'error': 'parquet not found'}
    try:
        import pyarrow.parquet as pq
    except ImportError as exc:  # pragma: no cover - runtime dependency
        raise RuntimeError('pyarrow is required for stationary-run cleanup') from exc

    try:
        table = pq.read_table(parquet_path, columns=['action'])
        actions = np.asarray(table.column('action').to_pylist(), dtype=np.float32)
    except Exception as exc:
        return {'run_dir': run_dir, 'is_stationary': False, 'error': str(exc)}

    if actions.size == 0:
        return {
            'run_dir': run_dir,
            'is_stationary': True,
            'frame_count': 0,
            'pos_std': 0.0,
            'yaw_std': 0.0,
        }

    positions = actions[:, [3, 7, 11]]
    yaws = np.array([_yaw_from_pose_matrix(action) for action in actions], dtype=np.float32)
    pos_std = float(np.std(positions, axis=0).mean())
    yaw_std = float(np.std(yaws))
    return {
        'run_dir': run_dir,
        'is_stationary': pos_std < pos_threshold and yaw_std < yaw_threshold,
        'frame_count': int(actions.shape[0]),
        'pos_std': pos_std,
        'yaw_std': yaw_std,
    }


def clean_stationary_runs(
    scene_root: Path,
    *,
    pos_threshold: float,
    yaw_threshold: float,
    dry_run: bool,
    logger: logging.Logger,
) -> dict[str, Any]:
    """Remove stationary run_N directories under one scene."""
    results: list[dict[str, Any]] = []
    deleted_runs: list[str] = []
    for run_dir in sorted(scene_root.glob('run_*')):
        if not run_dir.is_dir():
            continue
        result = detect_stationary_run(
            run_dir,
            pos_threshold=pos_threshold,
            yaw_threshold=yaw_threshold,
        )
        results.append(result)
        if result.get('error'):
            logger.warning('skip %s: %s', run_dir.name, result['error'])
            continue
        if not result['is_stationary']:
            logger.info(
                'keep %s: pos_std=%.4f yaw_std=%.4f frames=%d',
                run_dir.name,
                result['pos_std'],
                result['yaw_std'],
                result['frame_count'],
            )
            continue
        logger.info(
            '%s stationary: pos_std=%.4f yaw_std=%.4f frames=%d',
            run_dir.name,
            result['pos_std'],
            result['yaw_std'],
            result['frame_count'],
        )
        if not dry_run:
            shutil.rmtree(run_dir)
            deleted_runs.append(run_dir.name)
    return {
        'total_runs': len(results),
        'deleted_runs': deleted_runs,
        'kept_runs': len(results) - len(deleted_runs),
        'results': results,
    }


class BagConverter:
    """Convert one MCAP bag into LeRobot episodes."""

    def __init__(self, cfg: ConvertConfig, logger: logging.Logger) -> None:
        self._cfg = cfg
        self._logger = logger
        self._postprocessor = self._build_postprocessor()
        self._scene_root = cfg.output_root / cfg.scene_name
        self._meta_root = self._scene_root / 'meta'
        self._meta_root.mkdir(parents=True, exist_ok=True)
        self._run_index = 0
        self._recorder = self._new_recorder()
        self._run_reports: list[dict[str, Any]] = []
        self._cleanup_summary: dict[str, Any] | None = None

    def run(self) -> None:
        topics = {
            self._cfg.rgb_topic,
            self._cfg.depth_topic,
            self._cfg.camera_info_topic,
            self._cfg.odom_topic,
            self._cfg.odom_fallback_topic,
        }
        reader = BagMessageReader(self._cfg.bag_path, topics)
        current_episode_index: int | None = None
        frames_in_episode = 0

        self._recorder.prepare_for_recording()
        for observation in iter_sampled_observations(
            reader,
            rgb_topic=self._cfg.rgb_topic,
            depth_topic=self._cfg.depth_topic,
            camera_info_topic=self._cfg.camera_info_topic,
            odom_topic=self._cfg.odom_topic,
            odom_fallback_topic=self._cfg.odom_fallback_topic,
            fps=self._cfg.fps,
            episode_seconds=self._cfg.episode_seconds,
        ):
            if observation.odom_msg is None:
                continue
            frame = self._build_frame(observation)
            if current_episode_index is None:
                current_episode_index = observation.episode_index
            if observation.episode_index != current_episode_index and frames_in_episode > 0:
                self._save_episode(frames_in_episode)
                self._recorder.prepare_for_recording()
                current_episode_index = observation.episode_index
                frames_in_episode = 0
            self._recorder.add_frame(frame)
            frames_in_episode += 1

        if frames_in_episode > 0:
            self._save_episode(frames_in_episode)
        self._write_scene_manifest()
        if self._cfg.skip_stationary_runs:
            self._cleanup_summary = clean_stationary_runs(
                self._scene_root,
                pos_threshold=self._cfg.stationary_pos_threshold,
                yaw_threshold=self._cfg.stationary_yaw_threshold,
                dry_run=self._cfg.dry_run_clean,
                logger=self._logger,
            )
            self._logger.info(
                'stationary cleanup: total=%d kept=%d deleted=%d',
                self._cleanup_summary['total_runs'],
                self._cleanup_summary['kept_runs'],
                len(self._cleanup_summary['deleted_runs']),
            )
            self._write_scene_manifest()

    def _save_episode(self, frames_in_episode: int) -> None:
        run_root = self._scene_root / f'run_{self._run_index}'
        result = self._recorder.save_episode()
        self._recorder.finalize()
        if self._postprocessor is not None:
            self._postprocessor.write_episode_artifacts(run_root)
        self._logger.info('saved episode (%d frames): %s', frames_in_episode, result)
        self._run_reports.append(
            {
                'run_name': f'run_{self._run_index}',
                'frames': frames_in_episode,
                'has_sam3': bool(self._cfg.sam3_enable and self._cfg.inline_postprocess),
                'saved_result': result,
            }
        )
        self._run_index += 1
        self._recorder = self._new_recorder()
        if self._postprocessor is not None:
            self._postprocessor.reset_episode()

    def _new_recorder(self) -> DatasetRecorder:
        run_root = self._scene_root / f'run_{self._run_index}'
        return DatasetRecorder(
            repo_id=self._cfg.repo_id,
            fps=self._cfg.fps,
            root=str(run_root),
            logger=self._logger,
            robot_type=self._cfg.robot_type,
            dataset_format='jdrobot',
            video_vcodec=self._cfg.video_vcodec,
            overwrite_dataset=self._cfg.overwrite,
        )

    def _write_scene_manifest(self) -> None:
        manifest = {
            'bag_path': str(self._cfg.bag_path),
            'scene_name': self._cfg.scene_name,
            'repo_id': self._cfg.repo_id,
            'fps': self._cfg.fps,
            'episode_seconds': self._cfg.episode_seconds,
            'total_runs': self._run_index,
            'sam3_enable': self._cfg.sam3_enable,
            'inline_postprocess': self._cfg.inline_postprocess,
            'generated_at': datetime.now(timezone.utc).isoformat(),
        }
        (self._meta_root / 'info.json').write_text(
            json.dumps(manifest, indent=2) + '\n',
            encoding='utf-8',
        )
        report = {
            **manifest,
            'runs': self._run_reports,
            'cleanup': self._cleanup_summary,
        }
        (self._meta_root / 'report.json').write_text(
            json.dumps(report, indent=2) + '\n',
            encoding='utf-8',
        )

    def _build_postprocessor(self) -> BagPostprocessor | None:
        if not self._cfg.inline_postprocess:
            return None
        segmenter = self._build_segmenter()
        if segmenter is None and not self._cfg.record_pointcloud:
            return None
        return BagPostprocessor(
            BagPostprocessConfig(
                record_pointcloud=self._cfg.record_pointcloud,
                save_segmentation_viz=self._cfg.save_segmentation_viz,
                pointcloud_stride=self._cfg.pointcloud_stride,
                max_pointcloud_points=self._cfg.max_pointcloud_points,
                depth_min_m=self._cfg.depth_min_m,
                depth_max_m=self._cfg.depth_max_m,
            ),
            segmenter=segmenter,
        )

    def _build_segmenter(self) -> Sam3GroundSegmenter | None:
        if not self._cfg.sam3_enable:
            return None
        prompts = self._cfg.sam3_prompts or (
            'floor',
            'ground',
            'stair tread',
            'stair step',
            'pavement',
            'walkway',
        )
        return Sam3GroundSegmenter(
            device=self._cfg.sam3_device,
            confidence=self._cfg.sam3_confidence,
            prompts=prompts,
            checkpoint_path=self._cfg.sam3_checkpoint or None,
        )

    def _build_frame(self, observation: SampledObservation) -> dict[str, Any]:
        rgb = image_to_numpy(observation.rgb_msg)
        depth = image_to_numpy(observation.depth_msg)
        frame: dict[str, Any] = {
            KEY_CAMERA_INTRINSIC: camera_info_to_intrinsic(observation.camera_info_msg),
            KEY_CAMERA_EXTRINSIC: identity_extrinsic(),
            KEY_ACTION: odom_to_action_matrix(observation.odom_msg),
            KEY_RGB: rgb,
            KEY_DEPTH: depth_to_video_rgb(depth, self._cfg.depth_min_m, self._cfg.depth_max_m),
            KEY_TASK: self._cfg.task,
        }

        if self._postprocessor is not None:
            self._postprocessor.process_frame(
                frame=frame,
                rgb=rgb,
                depth=depth,
                camera_info=observation.camera_info_msg,
            )
        return frame


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Convert ROS2 MCAP bag to LeRobot jdrobot dataset.'
    )
    parser.add_argument('--bag', default='', help='input .mcap bag path')
    parser.add_argument(
        '--input-dir',
        default='',
        help='convert all *.mcap under this directory (recursive)',
    )
    parser.add_argument('--output-root', required=True, help='output dataset parent directory')
    parser.add_argument('--scene-name', default='', help='scene directory name (default: bag stem)')
    parser.add_argument('--repo-id', default='local/habitat_collection_bag')
    parser.add_argument('--robot-type', default='jdrobot')
    parser.add_argument('--task', default='navigate to goal')
    parser.add_argument('--fps', type=float, default=JDROBOT_COLLECTION_ROS_PARAMS['record_fps'])
    parser.add_argument('--episode-seconds', type=float, default=20.0)
    parser.add_argument('--video-vcodec', default=JDROBOT_COLLECTION_ROS_PARAMS['video_vcodec'])
    parser.add_argument('--overwrite', action='store_true')
    parser.add_argument('--rgb-topic', default=TOPIC_RGB)
    parser.add_argument('--depth-topic', default=TOPIC_DEPTH)
    parser.add_argument('--camera-info-topic', default=TOPIC_CAMERA_INFO)
    parser.add_argument('--odom-topic', default=TOPIC_ODOM)
    parser.add_argument('--odom-fallback-topic', default=TOPIC_ODOM_EKF)
    parser.add_argument(
        '--pointcloud-stride',
        type=int,
        default=JDROBOT_COLLECTION_ROS_PARAMS['pointcloud_stride'],
    )
    parser.add_argument(
        '--max-pointcloud-points',
        type=int,
        default=JDROBOT_COLLECTION_ROS_PARAMS['max_pointcloud_points'],
    )
    parser.add_argument(
        '--depth-min-m', type=float, default=JDROBOT_COLLECTION_ROS_PARAMS['depth_min_m']
    )
    parser.add_argument(
        '--depth-max-m', type=float, default=JDROBOT_COLLECTION_ROS_PARAMS['depth_max_m']
    )
    parser.add_argument('--no-pointcloud', action='store_true')
    parser.add_argument('--sam3-enable', action='store_true')
    parser.add_argument('--sam3-device', default='cuda')
    parser.add_argument('--sam3-confidence', type=float, default=0.3)
    parser.add_argument('--sam3-checkpoint', default='')
    parser.add_argument('--save-segmentation-viz', action='store_true')
    parser.add_argument(
        '--defer-postprocess',
        action='store_true',
        help='skip inline SAM3/pointcloud generation; run later via offline pipeline',
    )
    parser.add_argument('--skip-stationary-runs', action='store_true')
    parser.add_argument('--stationary-pos-threshold', type=float, default=0.01)
    parser.add_argument('--stationary-yaw-threshold', type=float, default=0.01)
    parser.add_argument('--dry-run-clean', action='store_true')
    parser.add_argument(
        '--sam3-prompts',
        nargs='+',
        default=[],
        help='override default ground prompts',
    )
    return parser


def _config_from_args(args: argparse.Namespace) -> ConvertConfig:
    bag_path = Path(args.bag).expanduser().resolve() if args.bag else None
    input_dir = Path(args.input_dir).expanduser().resolve() if args.input_dir else None
    if bag_path is None and input_dir is None:
        raise ValueError('either --bag or --input-dir is required')
    default_scene_name = ''
    if args.scene_name.strip():
        default_scene_name = args.scene_name.strip()
    elif bag_path is not None:
        default_scene_name = bag_path.stem
    return ConvertConfig(
        bag_path=bag_path,
        input_dir=input_dir,
        output_root=Path(args.output_root).expanduser().resolve(),
        scene_name=default_scene_name,
        repo_id=args.repo_id,
        robot_type=args.robot_type,
        task=args.task,
        fps=args.fps,
        episode_seconds=args.episode_seconds,
        video_vcodec=args.video_vcodec,
        overwrite=args.overwrite,
        rgb_topic=args.rgb_topic,
        depth_topic=args.depth_topic,
        camera_info_topic=args.camera_info_topic,
        odom_topic=args.odom_topic,
        odom_fallback_topic=args.odom_fallback_topic,
        pointcloud_stride=args.pointcloud_stride,
        max_pointcloud_points=args.max_pointcloud_points,
        depth_min_m=args.depth_min_m,
        depth_max_m=args.depth_max_m,
        record_pointcloud=not args.no_pointcloud,
        sam3_enable=args.sam3_enable,
        sam3_device=args.sam3_device,
        sam3_confidence=args.sam3_confidence,
        sam3_checkpoint=args.sam3_checkpoint,
        sam3_prompts=tuple(args.sam3_prompts),
        save_segmentation_viz=args.save_segmentation_viz,
        inline_postprocess=not args.defer_postprocess,
        skip_stationary_runs=args.skip_stationary_runs,
        stationary_pos_threshold=args.stationary_pos_threshold,
        stationary_yaw_threshold=args.stationary_yaw_threshold,
        dry_run_clean=args.dry_run_clean,
    )


def _parse_args(argv: list[str] | None = None) -> ConvertConfig:
    parser = _build_arg_parser()
    args = parser.parse_args(argv)
    try:
        return _config_from_args(args)
    except ValueError as exc:
        parser.error(str(exc))
        raise


def _iter_bag_jobs(cfg: ConvertConfig) -> list[ConvertConfig]:
    if cfg.bag_path is not None:
        if not cfg.bag_path.is_file():
            raise FileNotFoundError(f'bag not found: {cfg.bag_path}')
        return [cfg]
    assert cfg.input_dir is not None
    bags = sorted(path for path in cfg.input_dir.rglob('*.mcap') if path.is_file())
    if not bags:
        raise FileNotFoundError(f'no .mcap files under {cfg.input_dir}')
    jobs: list[ConvertConfig] = []
    for bag_path in bags:
        scene_name = bag_path.stem
        if bag_path.parent != cfg.input_dir:
            scene_name = bag_path.parent.name
        jobs.append(
            ConvertConfig(
                bag_path=bag_path,
                input_dir=cfg.input_dir,
                output_root=cfg.output_root,
                scene_name=scene_name,
                repo_id=cfg.repo_id,
                robot_type=cfg.robot_type,
                task=cfg.task,
                fps=cfg.fps,
                episode_seconds=cfg.episode_seconds,
                video_vcodec=cfg.video_vcodec,
                overwrite=cfg.overwrite,
                rgb_topic=cfg.rgb_topic,
                depth_topic=cfg.depth_topic,
                camera_info_topic=cfg.camera_info_topic,
                odom_topic=cfg.odom_topic,
                odom_fallback_topic=cfg.odom_fallback_topic,
                pointcloud_stride=cfg.pointcloud_stride,
                max_pointcloud_points=cfg.max_pointcloud_points,
                depth_min_m=cfg.depth_min_m,
                depth_max_m=cfg.depth_max_m,
                record_pointcloud=cfg.record_pointcloud,
                sam3_enable=cfg.sam3_enable,
                sam3_device=cfg.sam3_device,
                sam3_confidence=cfg.sam3_confidence,
                sam3_checkpoint=cfg.sam3_checkpoint,
                sam3_prompts=cfg.sam3_prompts,
                save_segmentation_viz=cfg.save_segmentation_viz,
                inline_postprocess=cfg.inline_postprocess,
                skip_stationary_runs=cfg.skip_stationary_runs,
                stationary_pos_threshold=cfg.stationary_pos_threshold,
                stationary_yaw_threshold=cfg.stationary_yaw_threshold,
                dry_run_clean=cfg.dry_run_clean,
            )
        )
    return jobs


def run_conversion(cfg: ConvertConfig, logger: logging.Logger) -> None:
    """Run one or more bag conversion jobs."""
    for job in _iter_bag_jobs(cfg):
        logger.info('convert bag: %s -> %s/%s', job.bag_path, job.output_root, job.scene_name)
        converter = BagConverter(job, logger)
        converter.run()


def main() -> None:
    cfg = _parse_args()
    logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')
    logger = logging.getLogger('autonomy_lerobot.bag_to_lerobot')
    run_conversion(cfg, logger)


if __name__ == '__main__':
    main()
