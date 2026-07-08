# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0
"""LeRobot video decode workarounds for the autonomy Docker image."""

from __future__ import annotations

import importlib.util
import logging
from pathlib import Path

import av
import torch


def _torchcodec_usable() -> bool:
    if importlib.util.find_spec('torchcodec') is None:
        return False
    try:
        from torchcodec.decoders import VideoDecoder  # noqa: F401
    except (ImportError, OSError, RuntimeError):
        return False
    return True


def _decode_video_frames_pyav(
    video_path: Path | str,
    timestamps: list[float],
    tolerance_s: float,
    log_loaded_timestamps: bool = False,
) -> torch.Tensor:
    """Decode frames with PyAV (no torchvision.io.VideoReader)."""
    from lerobot.datasets.video_utils import FrameTimestampError

    video_path = str(video_path)
    first_ts = min(timestamps)
    last_ts = max(timestamps)

    loaded_frames: list[torch.Tensor] = []
    loaded_ts: list[float] = []

    with av.open(video_path) as container:
        stream = container.streams.video[0]
        time_base = float(stream.time_base)
        seek_pts = int(first_ts / time_base) if time_base > 0 else 0
        container.seek(seek_pts, stream=stream, backward=True)

        for frame in container.decode(stream):
            if frame.pts is None:
                continue
            current_ts = float(frame.pts * time_base)
            if log_loaded_timestamps:
                logging.info('frame loaded at timestamp=%.4f', current_ts)
            rgb = torch.from_numpy(frame.to_ndarray(format='rgb24')).permute(2, 0, 1)
            loaded_frames.append(rgb)
            loaded_ts.append(current_ts)
            if current_ts >= last_ts:
                break

    if not loaded_frames:
        raise FrameTimestampError(
            f'No frames decoded from {video_path} for timestamps {timestamps}'
        )

    query_ts = torch.tensor(timestamps)
    loaded_ts_t = torch.tensor(loaded_ts)
    dist = torch.cdist(query_ts[:, None], loaded_ts_t[:, None], p=1)
    min_, argmin_ = dist.min(1)

    is_within_tol = min_ < tolerance_s
    if not is_within_tol.all():
        raise FrameTimestampError(
            f'One or several query timestamps unexpectedly violate the tolerance '
            f'({min_[~is_within_tol]} > {tolerance_s=}).'
            f'\nqueried timestamps: {query_ts}'
            f'\nloaded timestamps: {loaded_ts_t}'
            f'\nvideo: {video_path}'
            f'\nbackend: pyav'
        )

    closest_frames = torch.stack([loaded_frames[idx] for idx in argmin_])
    closest_frames = closest_frames.type(torch.float32) / 255

    if len(timestamps) != len(closest_frames):
        raise FrameTimestampError(
            f'Number of retrieved frames ({len(closest_frames)}) does not match '
            f'number of queried timestamps ({len(timestamps)})'
        )
    return closest_frames


def patch_lerobot_video_decode() -> None:
    """Prefer torchcodec; otherwise use PyAV without torchvision VideoReader."""
    import lerobot.datasets.video_utils as video_utils

    original_decode = video_utils.decode_video_frames_torchvision

    def get_codec() -> str:
        if _torchcodec_usable():
            return 'torchcodec'
        logging.warning(
            'torchcodec unavailable in this environment; using pyav for video decode',
        )
        return 'pyav'

    def decode_video_frames_torchvision(
        video_path: Path | str,
        timestamps: list[float],
        tolerance_s: float,
        backend: str = 'pyav',
        log_loaded_timestamps: bool = False,
    ) -> torch.Tensor:
        if backend not in ('pyav', 'video_reader'):
            raise ValueError(f'Unsupported torchvision video backend: {backend}')
        import torchvision

        if hasattr(torchvision.io, 'VideoReader'):
            return original_decode(
                video_path,
                timestamps,
                tolerance_s,
                backend,
                log_loaded_timestamps,
            )
        return _decode_video_frames_pyav(
            video_path,
            timestamps,
            tolerance_s,
            log_loaded_timestamps,
        )

    video_utils.get_safe_default_codec = get_codec
    video_utils.decode_video_frames_torchvision = decode_video_frames_torchvision
