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
import re
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

from autonomy_lerobot.observation import KEY_TASK


def _feature_for_array(key: str, arr: np.ndarray) -> dict[str, Any]:
    """Infer a LeRobot feature spec from a numpy array."""
    if arr.dtype == np.uint8 and arr.ndim == 3:
        h, w, c = arr.shape
        if c == 3:
            return {
                'dtype': 'video',
                'shape': (h, w, c),
                'names': ['height', 'width', 'channel'],
            }
    if arr.ndim == 3 and arr.shape[2] == 1:
        h, w, _ = arr.shape
        return {
            'dtype': 'image',
            'shape': (h, w, 1),
            'names': ['height', 'width', 'channel'],
        }
    if arr.ndim == 2:
        h, w = arr.shape
        return {
            'dtype': 'image',
            'shape': (h, w, 1),
            'names': ['height', 'width', 'channel'],
        }
    if arr.ndim == 1:
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


def _features_from_frame(frame: dict[str, Any]) -> dict[str, Any]:
    features: dict[str, Any] = {}
    for key, value in frame.items():
        if key == KEY_TASK or not isinstance(value, np.ndarray):
            continue
        features[key] = _feature_for_array(key, value)
    return features


def _normalize_frame_for_lerobot(frame: dict[str, Any]) -> dict[str, Any]:
    """Ensure array shapes match LeRobot image feature expectations."""
    normalized: dict[str, Any] = {}
    for key, value in frame.items():
        if isinstance(value, np.ndarray) and value.ndim == 2:
            normalized[key] = np.ascontiguousarray(value[..., np.newaxis])
        else:
            normalized[key] = value
    return normalized


class DatasetRecorder:
    """Write frames to LeRobot when installed, otherwise save .npz episodes."""

    def __init__(
        self,
        repo_id: str,
        fps: float,
        root: str,
        logger,
        *,
        video_vcodec: str = 'h264',
        streaming_encoding: bool = True,
        parallel_video_encoding: bool = True,
    ) -> None:
        self._repo_id = repo_id
        self._fps = int(round(fps))
        self._root = Path(root).expanduser()
        self._lerobot_root: Path | None = None
        self._logger = logger
        self._episode_index = 0
        self._fallback_frames: list[dict[str, Any]] = []
        self._lerobot = None
        self._lerobot_available = self._check_lerobot()
        self._video_vcodec = video_vcodec
        self._streaming_encoding = streaming_encoding
        self._parallel_video_encoding = parallel_video_encoding

    def _is_lerobot_dataset(self, path: Path) -> bool:
        return (path / 'meta' / 'info.json').is_file()

    def _resolve_lerobot_root(self) -> Path:
        """Pick a LeRobot dataset directory, avoiding clashes with .npz fallback data."""
        if self._lerobot_root is not None:
            return self._lerobot_root
        root = self._root
        if self._is_lerobot_dataset(root):
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

    def _check_lerobot(self) -> bool:
        try:
            import lerobot.datasets.lerobot_dataset  # noqa: F401
            return True
        except ImportError:
            self._logger.warning(
                f'lerobot not installed; saving episodes as .npz under {self._root}')
            return False

    def _ensure_lerobot(self, frame: dict[str, Any]) -> None:
        if self._lerobot is not None or not self._lerobot_available:
            return
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        root = self._resolve_lerobot_root()
        if self._is_lerobot_dataset(root):
            self._lerobot = LeRobotDataset(
                repo_id=self._repo_id,
                root=str(root),
            )
            self._episode_index = self._lerobot.num_episodes
            self._logger.info(
                f'Opened LeRobot dataset: repo_id={self._repo_id} root={root}')
            return

        try:
            self._lerobot = LeRobotDataset.create(
                repo_id=self._repo_id,
                fps=self._fps,
                features=_features_from_frame(frame),
                robot_type='habitat_diffdrive',
                root=str(root),
                vcodec=self._video_vcodec,
                streaming_encoding=self._streaming_encoding,
            )
        except FileExistsError:
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            root = root.parent / f'{root.name}__lerobot_{stamp}'
            self._lerobot_root = root
            self._logger.warning(
                f'Dataset path already exists; retrying at {root}')
            self._lerobot = LeRobotDataset.create(
                repo_id=self._repo_id,
                fps=self._fps,
                features=_features_from_frame(frame),
                robot_type='habitat_diffdrive',
                root=str(root),
                vcodec=self._video_vcodec,
                streaming_encoding=self._streaming_encoding,
            )
        mode = 'streaming mp4' if self._streaming_encoding else 'png -> mp4 on save'
        self._logger.info(
            f'LeRobot dataset: repo_id={self._repo_id} root={root} video={mode}')

    def add_frame(self, frame: dict[str, Any]) -> None:
        frame = _normalize_frame_for_lerobot(frame)
        if self._lerobot_available:
            self._ensure_lerobot(frame)
        if self._lerobot is not None:
            self._lerobot.add_frame(frame)
            return
        self._fallback_frames.append(frame)

    def save_episode(self) -> str:
        if self._lerobot is not None:
            self._lerobot.save_episode(
                parallel_encoding=self._parallel_video_encoding)
            self._episode_index += 1
            root = self._resolve_lerobot_root()
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
        return str(episode_dir)

    def finalize(self) -> None:
        if self._lerobot is not None:
            self._lerobot.finalize()
