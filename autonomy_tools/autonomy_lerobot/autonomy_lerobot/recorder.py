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

"""Record navigation episodes to LeRobot or local numpy files."""

from __future__ import annotations

import json
import os
import re
import shutil
import threading
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

from autonomy_lerobot.observation import KEY_TASK

from autonomy_lerobot.repo_id import sanitize_repo_id

_LEGACY_REPO_IDS = ('local/habitat_collection', 'local/habitat_nav2')

_JDROBOT_VECTOR_NAMES: dict[str, list[str]] = {
    'observation.camera_intrinsic': [f'K_{i}' for i in range(9)],
    'observation.camera_extrinsic': [f'E_{i}' for i in range(16)],
    'action': [f'A_{i}' for i in range(16)],
}

_JDROBOT_DATA_FEATURE_KEYS = frozenset({
    'observation.camera_intrinsic',
    'observation.camera_extrinsic',
    'action',
    'observation.images.rgb',
    'observation.images.depth',
})

_JDROBOT_OPTIONAL_FEATURE_KEYS = frozenset({
    'observation.pointcloud',
    'observation.images.semantic',
})

_META_FEATURE_KEYS = frozenset({
    'timestamp', 'frame_index', 'episode_index', 'index', 'task_index',
})

# LeRobot encoder API names (info.json may still use the config label, e.g. av1).
_LEROBOT_ENCODER_VCODEC_ALIASES = {
    'av1': 'libsvtav1',
}


def lerobot_encoder_vcodec(config_vcodec: str) -> str:
    """Map dataset metadata codec (kujiale: av1) to LeRobot encoder name."""
    return _LEROBOT_ENCODER_VCODEC_ALIASES.get(config_vcodec.strip().lower(), config_vcodec)


def _jdrobot_data_feature_keys(features: dict[str, Any]) -> set[str]:
    return set(features.keys()) - _META_FEATURE_KEYS


def dataset_schema_matches(
    info: dict[str, Any],
    *,
    dataset_format: str,
    robot_type: str,
    fps: int,
    video_vcodec: str,
) -> tuple[bool, str]:
    """Return whether on-disk info.json matches the requested recording schema."""
    if dataset_format == 'jdrobot':
        if info.get('robot_type') != robot_type:
            return False, (
                f"robot_type={info.get('robot_type')!r} (expected {robot_type!r})")
        if int(info.get('fps', -1)) != fps:
            return False, f"fps={info.get('fps')} (expected {fps})"
        features = info.get('features', {})
        data_keys = _jdrobot_data_feature_keys(features)
        if not _JDROBOT_DATA_FEATURE_KEYS.issubset(data_keys):
            missing = sorted(_JDROBOT_DATA_FEATURE_KEYS - data_keys)
            return False, f'features missing {missing}'
        extra = data_keys - _JDROBOT_DATA_FEATURE_KEYS - _JDROBOT_OPTIONAL_FEATURE_KEYS
        if extra:
            return False, f'features extra {sorted(extra)}'
        action = features.get('action', {})
        if list(action.get('shape', [])) != [16]:
            return False, f"action.shape={action.get('shape')} (expected [16])"
        for video_key in ('observation.images.rgb', 'observation.images.depth'):
            codec = features.get(video_key, {}).get('info', {}).get('video.codec')
            if codec != video_vcodec:
                return False, (
                    f'{video_key} codec={codec!r} (expected {video_vcodec!r})')
        return True, ''
    if info.get('robot_type') != robot_type:
        return False, (
            f"robot_type={info.get('robot_type')!r} (expected {robot_type!r})")
    return True, ''


def _load_dataset_info(path: Path) -> dict[str, Any] | None:
    info_path = path / 'meta' / 'info.json'
    if not info_path.is_file():
        return None
    return json.loads(info_path.read_text(encoding='utf-8'))


def _feature_for_array(key: str, arr: np.ndarray, *, dataset_format: str) -> dict[str, Any]:
    """Infer a LeRobot feature spec from a numpy array."""
    if arr.dtype == np.uint8 and arr.ndim == 3:
        h, w, c = arr.shape
        if c == 3:
            spec: dict[str, Any] = {
                'dtype': 'video',
                'shape': (h, w, c),
            }
            if dataset_format != 'jdrobot':
                spec['names'] = ['height', 'width', 'channel']
            return spec
    if arr.ndim == 3 and arr.shape[2] == 1:
        return {
            'dtype': 'float32',
            'shape': tuple(arr.shape),
            'names': ['height', 'width', 'channel'],
        }
    if arr.ndim == 2:
        return {
            'dtype': 'float32',
            'shape': tuple(arr.shape),
            'names': ['height', 'width'],
        }
    if arr.ndim == 1:
        if dataset_format == 'jdrobot' and key in _JDROBOT_VECTOR_NAMES:
            return {
                'dtype': 'float32',
                'shape': (arr.shape[0],),
                'names': _JDROBOT_VECTOR_NAMES[key],
            }
        stem = re.sub(r'^observation\.', '', key).replace('.', '_')
        return {
            'dtype': 'float32',
            'shape': (arr.shape[0],),
            'names': [f'{stem}_{i}' for i in range(arr.shape[0])],
        }
    return {
        'dtype': 'float32',
        'shape': tuple(arr.shape),
        'names': [f'dim{i}' for i in range(arr.ndim)],
    }


def _features_from_frame(frame: dict[str, Any], *, dataset_format: str = 'habitat_nav2') -> dict[str, Any]:
    features: dict[str, Any] = {}
    for key, value in frame.items():
        if key == KEY_TASK or not isinstance(value, np.ndarray):
            continue
        features[key] = _feature_for_array(key, value, dataset_format=dataset_format)
    return features


def _normalize_frame_for_lerobot(frame: dict[str, Any]) -> dict[str, Any]:
    """Ensure array shapes match LeRobot image feature expectations."""
    normalized: dict[str, Any] = {}
    for key, value in frame.items():
        if isinstance(value, np.ndarray) and value.ndim == 2:
            normalized[key] = np.ascontiguousarray(value[..., np.newaxis])
        elif isinstance(value, np.ndarray) and value.ndim == 3 and value.shape[2] == 1:
            normalized[key] = np.ascontiguousarray(value.astype(np.float32, copy=False))
        else:
            normalized[key] = value
    return normalized


class DatasetRecorder:
    """Write frames to LeRobot v3 datasets (default); npz fallback if import fails."""

    def __init__(
        self,
        repo_id: str,
        fps: float,
        root: str,
        logger,
        *,
        robot_type: str = 'habitat_diffdrive',
        dataset_format: str = 'habitat_nav2',
        video_vcodec: str = 'h264',
        streaming_encoding: bool = True,
        parallel_video_encoding: bool = True,
        overwrite_dataset: bool = False,
    ) -> None:
        self._repo_id = sanitize_repo_id(repo_id)
        self._fps = int(round(fps))
        self._root = Path(root).expanduser()
        self._lerobot_root: Path | None = None
        self._logger = logger
        self._robot_type = robot_type
        self._dataset_format = dataset_format
        self._episode_index = 0
        self._fallback_frames: list[dict[str, Any]] = []
        self._lerobot = None
        self._lerobot_available = self._check_lerobot()
        self._video_vcodec = video_vcodec
        self._lerobot_vcodec = lerobot_encoder_vcodec(video_vcodec)
        self._streaming_encoding = streaming_encoding
        self._parallel_video_encoding = parallel_video_encoding
        self._overwrite_dataset = overwrite_dataset
        self._buffered_frames = 0
        self._lock = threading.Lock()
        if self._overwrite_dataset:
            self._force_remove_dataset(self._root)
        else:
            self._discard_incomplete_dataset(self._root)
            self._reject_schema_mismatch(self._root)
        if self._lerobot_vcodec != self._video_vcodec:
            self._logger.info(
                f'video_vcodec {self._video_vcodec!r} uses lerobot encoder '
                f'{self._lerobot_vcodec!r}')

    def _sync_info_json_video_codecs(self, root: Path) -> None:
        """Keep meta info.json video.codec aligned with kujiale labels (e.g. av1)."""
        if self._lerobot_vcodec == self._video_vcodec:
            return
        info_path = root / 'meta' / 'info.json'
        if not info_path.is_file():
            return
        try:
            info = json.loads(info_path.read_text(encoding='utf-8'))
            features = info.get('features', {})
            changed = False
            for spec in features.values():
                if not isinstance(spec, dict) or spec.get('dtype') != 'video':
                    continue
                video_info = spec.setdefault('info', {})
                if video_info.get('video.codec') != self._video_vcodec:
                    video_info['video.codec'] = self._video_vcodec
                    changed = True
            if changed:
                info_path.write_text(
                    json.dumps(info, indent=4) + '\n', encoding='utf-8')
        except Exception as exc:
            self._logger.warning(f'could not sync video.codec in {info_path}: {exc}')

    @property
    def buffered_frames(self) -> int:
        return self._buffered_frames

    def _has_recorded_episodes(self, path: Path) -> bool:
        data_dir = path / 'data'
        if data_dir.is_dir() and any(data_dir.rglob('*.parquet')):
            return True
        videos_dir = path / 'videos'
        return videos_dir.is_dir() and any(videos_dir.rglob('*.mp4'))

    def _is_complete_lerobot_dataset(self, path: Path) -> bool:
        meta = path / 'meta'
        if not (meta / 'info.json').is_file():
            return False
        if self._has_recorded_episodes(path):
            return True
        return (meta / 'tasks.parquet').is_file()

    def _is_partial_lerobot_dataset(self, path: Path) -> bool:
        """Empty or interrupted create (info.json only, no frames on disk)."""
        meta = path / 'meta'
        if not (meta / 'info.json').is_file():
            return False
        if self._has_recorded_episodes(path):
            return False
        return not (meta / 'tasks.parquet').is_file()

    def _is_usable_empty_dir(self, path: Path) -> bool:
        return path.is_dir() and not any(path.iterdir())

    def _discard_incomplete_dataset(self, path: Path) -> None:
        """Remove only empty interrupted creates; never delete recorded data."""
        if not path.exists():
            return
        if self._is_complete_lerobot_dataset(path):
            return
        if self._is_usable_empty_dir(path):
            return
        if not self._is_partial_lerobot_dataset(path):
            return
        self._logger.warning(
            f'removing empty incomplete LeRobot dataset at {path} '
            '(no data/videos on disk)')
        shutil.rmtree(path)

    def _force_remove_dataset(self, path: Path) -> None:
        """Remove any dataset tree so create/open can start fresh at *path*."""
        if not path.exists():
            return
        self._logger.warning(f'removing LeRobot dataset at {path}')
        shutil.rmtree(path)
        self._lerobot_root = None
        self._lerobot = None

    def _schema_mismatch_reason(self, path: Path) -> str | None:
        info = _load_dataset_info(path)
        if info is None:
            return None
        ok, reason = dataset_schema_matches(
            info,
            dataset_format=self._dataset_format,
            robot_type=self._robot_type,
            fps=self._fps,
            video_vcodec=self._video_vcodec,
        )
        return None if ok else reason

    def _reject_schema_mismatch(self, path: Path) -> None:
        """Refuse to append when an existing dataset uses a different schema."""
        if not self._is_complete_lerobot_dataset(path):
            return
        reason = self._schema_mismatch_reason(path)
        if reason is None:
            return
        raise RuntimeError(
            f'existing dataset at {path} is incompatible with '
            f'dataset_format={self._dataset_format!r} '
            f'robot_type={self._robot_type!r} fps={self._fps} '
            f'vcodec={self._video_vcodec!r}: {reason}. '
            'Delete the directory or launch with clean_datasets_on_start:=true')

    def _episode_indices_from_parquet(
        self, root: Path, subdir: str,
    ) -> set[int] | None:
        """Collect unique episode_index values from parquet under *subdir*."""
        base = root / subdir
        if not base.is_dir():
            return set()
        try:
            import pyarrow.compute as pc
            import pyarrow.parquet as pq
        except ImportError:
            return None
        indices: set[int] = set()
        for path in base.rglob('*.parquet'):
            try:
                table = pq.read_table(path, columns=['episode_index'])
                col = table.column('episode_index')
                if hasattr(pc, 'unique'):
                    unique = pc.unique(col)
                    indices.update(int(x) for x in unique.to_pylist())
                else:
                    indices.update(int(x) for x in col.to_pylist())
            except Exception:
                continue
        return indices

    @staticmethod
    def _contiguous_episode_count(indices: set[int]) -> int:
        """Largest n with episodes 0..n-1 all present in *indices*."""
        count = 0
        while count in indices:
            count += 1
        return count

    def _count_data_frames(self, root: Path, max_episode: int) -> int | None:
        data_dir = root / 'data'
        if not data_dir.is_dir():
            return None
        try:
            import pyarrow.compute as pc
            import pyarrow.parquet as pq
        except ImportError:
            return None
        total = 0
        for path in data_dir.rglob('*.parquet'):
            try:
                table = pq.read_table(path, columns=['episode_index'])
                col = table.column('episode_index')
                mask = pc.less(col, max_episode)
                total += int(pc.sum(pc.cast(mask, 'int64')).as_py())
            except Exception:
                continue
        return total

    def _prune_meta_episodes(self, root: Path, keep_count: int) -> bool:
        """Drop meta/episodes rows with episode_index >= keep_count."""
        episodes_dir = root / 'meta' / 'episodes'
        if not episodes_dir.is_dir():
            return False
        try:
            import pyarrow.parquet as pq
        except ImportError:
            return False
        changed = False
        for path in episodes_dir.rglob('*.parquet'):
            try:
                table = pq.read_table(path)
                col = table.column('episode_index').to_pylist()
                keep = [i for i, ep in enumerate(col) if int(ep) < keep_count]
                if len(keep) == len(col):
                    continue
                if not keep:
                    path.unlink()
                else:
                    pq.write_table(table.take(keep), path)
                changed = True
            except Exception:
                continue
        return changed

    def _repair_info_episode_count(self, root: Path) -> bool:
        """Align info.json and meta/episodes with on-disk frame data."""
        meta_indices = self._episode_indices_from_parquet(root, 'meta/episodes')
        data_indices = self._episode_indices_from_parquet(root, 'data')
        if meta_indices is None or data_indices is None:
            return self._repair_info_episode_count_legacy(root)

        meta_count = self._contiguous_episode_count(meta_indices)
        data_count = self._contiguous_episode_count(data_indices)
        if meta_count > 0 and data_count > 0:
            count = min(meta_count, data_count)
        else:
            count = meta_count or data_count

        info_path = root / 'meta' / 'info.json'
        if not info_path.is_file():
            return False
        try:
            info = json.loads(info_path.read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError):
            return False

        changed = False
        old = int(info.get('total_episodes', -1))
        if old != count:
            info['total_episodes'] = count
            info['splits'] = {'train': f'0:{count}'}
            changed = True

        frames = self._count_data_frames(root, count)
        if frames is not None:
            old_frames = int(info.get('total_frames', -1))
            if old_frames != frames:
                info['total_frames'] = frames
                changed = True

        if changed:
            info_path.write_text(json.dumps(info, indent=2) + '\n', encoding='utf-8')
            self._logger.warning(
                f'repaired meta/info.json at {root}: '
                f'total_episodes {old} -> {count}'
                + (f', total_frames -> {frames}' if frames is not None else ''))

        if meta_count > count and self._prune_meta_episodes(root, count):
            changed = True
            self._logger.warning(
                f'pruned meta/episodes to episode_index < {count} at {root}')
        return changed

    def _repair_info_episode_count_legacy(self, root: Path) -> bool:
        """Fallback when pyarrow is unavailable: sync info with meta/episodes only."""
        episodes_dir = root / 'meta' / 'episodes'
        if not episodes_dir.is_dir():
            return False
        count = len(list(episodes_dir.rglob('*.parquet')))
        info_path = root / 'meta' / 'info.json'
        if not info_path.is_file() or count == 0:
            return False
        try:
            info = json.loads(info_path.read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError):
            return False
        old = int(info.get('total_episodes', -1))
        if old == count:
            return False
        info['total_episodes'] = count
        info['splits'] = {'train': f'0:{count}'}
        info_path.write_text(json.dumps(info, indent=2) + '\n', encoding='utf-8')
        self._logger.warning(
            f'repaired meta/info.json total_episodes {old} -> {count} at {root}')
        return True

    def _repo_id_from_info(self, root: Path) -> str | None:
        info_path = root / 'meta' / 'info.json'
        if not info_path.is_file():
            return None
        try:
            info = json.loads(info_path.read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError):
            return None
        repo_id = info.get('repo_id') or info.get('dataset_repo_id')
        return str(repo_id) if repo_id else None

    def _repo_ids_to_try(self, root: Path) -> list[str]:
        seen: set[str] = set()
        out: list[str] = []
        for candidate in (
            self._repo_id,
            self._repo_id_from_info(root),
            *_LEGACY_REPO_IDS,
            sanitize_repo_id(f'local/habitat_collection/{root.name}'),
        ):
            if not candidate:
                continue
            rid = sanitize_repo_id(candidate)
            if rid not in seen:
                seen.add(rid)
                out.append(rid)
        return out

    def _has_episodes_meta(self, path: Path) -> bool:
        episodes = path / 'meta' / 'episodes'
        return episodes.is_dir() and any(episodes.rglob('*.parquet'))

    def _prepare_root_for_create(self, root: Path) -> None:
        """LeRobot create() requires *root* not to exist (even as an empty dir)."""
        if self._is_usable_empty_dir(root):
            shutil.rmtree(root)

    def _open_lerobot_dataset(self, root: Path, repo_id: str | None = None):
        """Open an existing on-disk dataset for append (offline, streaming encoder)."""
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        rid = repo_id or self._repo_id
        old_offline = os.environ.get('HF_HUB_OFFLINE')
        os.environ['HF_HUB_OFFLINE'] = '1'
        try:
            return LeRobotDataset(
                repo_id=rid,
                root=str(root),
                vcodec=self._lerobot_vcodec,
                streaming_encoding=self._streaming_encoding,
            )
        finally:
            if old_offline is None:
                os.environ.pop('HF_HUB_OFFLINE', None)
            else:
                os.environ['HF_HUB_OFFLINE'] = old_offline

    def _resolve_lerobot_root(self) -> Path:
        """Pick a LeRobot dataset directory, avoiding clashes with .npz fallback data."""
        if self._lerobot_root is not None:
            return self._lerobot_root
        root = self._root
        if self._is_complete_lerobot_dataset(root):
            reason = self._schema_mismatch_reason(root)
            if reason is not None:
                if self._overwrite_dataset:
                    self._logger.warning(
                        f'replacing incompatible dataset at {root}: {reason}')
                    self._force_remove_dataset(root)
                else:
                    self._reject_schema_mismatch(root)
            self._lerobot_root = root
            return root
        if self._is_partial_lerobot_dataset(root):
            self._discard_incomplete_dataset(root)
            self._lerobot_root = root
            return root
        if self._is_usable_empty_dir(root):
            self._lerobot_root = root
            return root
        if root.exists():
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            alt = root.parent / f'{root.name}__lerobot_{stamp}'
            self._logger.warning(
                f'{root} already exists (not a LeRobot dataset); '
                f'writing new dataset to {alt}')
            self._lerobot_root = alt
            return alt
        self._lerobot_root = root
        return root

    def _looks_like_lerobot_dataset(self, path: Path) -> bool:
        meta = path / 'meta'
        if (meta / 'info.json').is_file():
            return True
        return (path / 'data').is_dir() or (path / 'videos').is_dir()

    def _create_lerobot_dataset(self, frame: dict[str, Any], root: Path):
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        self._prepare_root_for_create(root)
        try:
            return LeRobotDataset.create(
                repo_id=self._repo_id,
                fps=self._fps,
                features=_features_from_frame(frame, dataset_format=self._dataset_format),
                robot_type=self._robot_type,
                root=str(root),
                vcodec=self._lerobot_vcodec,
                streaming_encoding=self._streaming_encoding,
            )
        except FileExistsError:
            if self._try_open_lerobot_dataset(root):
                return self._lerobot
            if self._overwrite_dataset:
                self._logger.warning(
                    f'dataset path already exists at {root}; overwrite enabled, recreating')
                self._force_remove_dataset(root)
                self._prepare_root_for_create(root)
                self._lerobot_root = root
                return LeRobotDataset.create(
                    repo_id=self._repo_id,
                    fps=self._fps,
                    features=_features_from_frame(frame, dataset_format=self._dataset_format),
                    robot_type=self._robot_type,
                    root=str(root),
                    vcodec=self._lerobot_vcodec,
                    streaming_encoding=self._streaming_encoding,
                )
            raise RuntimeError(
                f'LeRobot dataset path exists at {root} but cannot be opened for append; '
                'set overwrite_dataset:=true to replace it')

    def _try_open_lerobot_dataset(self, root: Path) -> bool:
        if not self._looks_like_lerobot_dataset(root):
            return False

        if (
            self._has_recorded_episodes(root)
            and not self._has_episodes_meta(root)
        ):
            self._logger.warning(
                f'LeRobot dataset at {root} has data but missing meta/episodes; '
                'run finalize on previous session or set overwrite_dataset:=true')

        self._repair_info_episode_count(root)

        last_exc: Exception | None = None
        for repo_id in self._repo_ids_to_try(root):
            try:
                self._lerobot = self._open_lerobot_dataset(root, repo_id)
                self._repo_id = repo_id
                self._lerobot_root = root
                self._episode_index = self._lerobot.num_episodes
                self._logger.info(
                    f'Opened existing LeRobot dataset for append: '
                    f'repo_id={repo_id} root={root} episodes={self._episode_index}')
                return True
            except Exception as exc:
                last_exc = exc
                if 'out of range' in str(exc).lower() and self._repair_info_episode_count(root):
                    try:
                        self._lerobot = self._open_lerobot_dataset(root, repo_id)
                        self._repo_id = repo_id
                        self._lerobot_root = root
                        self._episode_index = self._lerobot.num_episodes
                        self._logger.info(
                            f'Opened LeRobot dataset after meta repair: '
                            f'repo_id={repo_id} root={root} episodes={self._episode_index}')
                        return True
                    except Exception as retry_exc:
                        last_exc = retry_exc
                self._logger.warning(
                    f'failed to open LeRobot dataset at {root} '
                    f'(repo_id={repo_id}): {exc}')
        if last_exc is not None:
            self._logger.warning(
                f'failed to open existing LeRobot dataset at {root}: {last_exc}')
        return False

    def _reset_episode_buffer(self) -> None:
        """Repair episode_buffer after a failed save or before a new recording."""
        if self._lerobot is None:
            return
        buf = self._lerobot.episode_buffer
        if buf is not None and 'size' in buf:
            return
        encoder = getattr(self._lerobot, '_streaming_encoder', None)
        if encoder is not None:
            try:
                encoder.cancel_episode()
            except Exception:
                pass
        self._lerobot.episode_buffer = self._lerobot.create_episode_buffer()

    def prepare_for_recording(self) -> None:
        """Reset in-memory episode state before starting a new recording session."""
        with self._lock:
            root = self._resolve_lerobot_root()
            if self._lerobot is None and self._lerobot_available:
                if self._is_complete_lerobot_dataset(root):
                    if not self._try_open_lerobot_dataset(root):
                        raise RuntimeError(
                            f'cannot open existing LeRobot dataset at {root} for append; '
                            'stop the bridge gracefully (finalize) or set overwrite_dataset:=true')
            if self._lerobot is None:
                return
            encoder = getattr(self._lerobot, '_streaming_encoder', None)
            if encoder is not None:
                try:
                    encoder.cancel_episode()
                except Exception:
                    pass
            self._lerobot.episode_buffer = self._lerobot.create_episode_buffer()
            self._buffered_frames = 0

    def _check_lerobot(self) -> bool:
        try:
            import lerobot.datasets.lerobot_dataset  # noqa: F401
            return True
        except ImportError as exc:
            self._logger.error(
                f'cannot import lerobot ({exc}); falling back to .npz under {self._root}. '
                'Docker image should include lerobot — check PYTHONPATH / venv.')
            return False

    def _archive_unopenable_dataset(self, root: Path) -> None:
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        backup = root.parent / f'{root.name}__broken_{stamp}'
        shutil.move(str(root), str(backup))
        self._lerobot_root = None
        self._logger.warning(
            f'archived unopenable LeRobot dataset to {backup} (will create a new one)')

    def _ensure_lerobot(self, frame: dict[str, Any]) -> None:
        if self._lerobot is not None or not self._lerobot_available:
            return

        root = self._resolve_lerobot_root()
        if self._is_complete_lerobot_dataset(root) and self._try_open_lerobot_dataset(root):
            return

        if self._is_complete_lerobot_dataset(root) and not self._overwrite_dataset:
            if (
                self._has_recorded_episodes(root)
                and not self._has_episodes_meta(root)
            ):
                self._archive_unopenable_dataset(root)
                root = self._root
                self._lerobot_root = None
            else:
                raise RuntimeError(
                    f'cannot open existing LeRobot dataset at {root} for append; '
                    'stop the bridge gracefully (finalize) or set overwrite_dataset:=true')

        self._prepare_root_for_create(root)
        self._lerobot = self._create_lerobot_dataset(frame, root)
        mode = 'streaming mp4' if self._streaming_encoding else 'png -> mp4 on save'
        actual_root = self._lerobot_root or root
        self._logger.info(
            f'LeRobot dataset: repo_id={self._repo_id} root={actual_root} video={mode}')

    def add_frame(self, frame: dict[str, Any]) -> None:
        frame = _normalize_frame_for_lerobot(frame)
        with self._lock:
            if self._lerobot_available:
                self._ensure_lerobot(frame)
            if self._lerobot is not None:
                self._reset_episode_buffer()
                self._lerobot.add_frame(dict(frame))
                self._buffered_frames += 1
                return
            self._fallback_frames.append(frame)
            self._buffered_frames += 1

    def save_episode(self) -> str:
        with self._lock:
            if self._buffered_frames == 0 and not self._fallback_frames:
                return 'no frames to save'
            if self._lerobot is not None:
                try:
                    self._lerobot.save_episode(
                        parallel_encoding=self._parallel_video_encoding)
                    self._episode_index += 1
                    self._buffered_frames = 0
                except Exception as exc:
                    self._logger.error(f'save_episode failed: {exc}')
                    self._reset_episode_buffer()
                    self._buffered_frames = 0
                    return f'save failed: {exc}'
                root = self._resolve_lerobot_root()
                self._sync_info_json_video_codecs(root)
                videos = sorted((root / 'videos').rglob('*.mp4')) if (root / 'videos').is_dir() else []
                if videos:
                    latest = videos[-1]
                    return f'lerobot:{self._repo_id} episode={self._episode_index} video={latest}'
                return f'lerobot:{self._repo_id} episode={self._episode_index}'

            if not self._fallback_frames:
                return 'no frames to save'

            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            episode_dir = self._root / 'episodes' / f'episode_{self._episode_index:04d}_{stamp}'
            episode_dir.mkdir(parents=True, exist_ok=True)

            arrays: dict[str, np.ndarray] = {}
            for key in self._fallback_frames[0]:
                if key == KEY_TASK:
                    continue
                values = [f[key] for f in self._fallback_frames]
                arrays[key.replace('.', '_')] = np.stack(values, axis=0)

            arrays['task'] = np.array(self._fallback_frames[0][KEY_TASK])
            arrays['fps'] = np.array(self._fps)
            np.savez_compressed(episode_dir / 'episode.npz', **arrays)

            (episode_dir / 'meta.json').write_text(
                json.dumps({
                    'repo_id': self._repo_id,
                    'fps': self._fps,
                    'num_frames': len(self._fallback_frames),
                    'keys': sorted(arrays.keys()),
                }, indent=2),
                encoding='utf-8',
            )
            self._fallback_frames.clear()
            self._episode_index += 1
            self._buffered_frames = 0
            return str(episode_dir)

    def finalize(self) -> None:
        if self._lerobot is not None:
            try:
                self._lerobot.finalize()
            except Exception as exc:
                self._logger.warning(f'LeRobot finalize: {exc}')
            self._sync_info_json_video_codecs(self._resolve_lerobot_root())
