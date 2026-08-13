"""RGB-D preprocessing for policy input and collision checks."""

from __future__ import annotations

import numpy as np


def preprocess_rgb(
    rgb: np.ndarray,
    width: int,
    height: int,
) -> np.ndarray:
    """Resize and normalize RGB image to CHW float32 in [0, 1]."""
    import cv2

    resized = cv2.resize(rgb, (width, height), interpolation=cv2.INTER_AREA)
    if resized.ndim == 2:
        resized = np.stack([resized] * 3, axis=-1)
    if resized.shape[-1] == 4:
        resized = resized[..., :3]
    chw = np.transpose(resized.astype(np.float32) / 255.0, (2, 0, 1))
    return chw


def preprocess_depth(
    depth_m: np.ndarray,
    width: int,
    height: int,
    max_range_m: float,
) -> np.ndarray:
    """Resize and clip depth to single-channel CHW float32 in [0, 1]."""
    import cv2

    resized = cv2.resize(depth_m, (width, height), interpolation=cv2.INTER_NEAREST)
    clipped = np.clip(resized, 0.0, max_range_m) / max(max_range_m, 1e-6)
    return clipped.astype(np.float32)[None, ...]


def stack_rgbd(
    rgb: np.ndarray,
    depth_m: np.ndarray,
    width: int,
    height: int,
    max_range_m: float,
) -> np.ndarray:
    """Stack RGB and depth into a 4-channel CHW tensor."""
    rgb_chw = preprocess_rgb(rgb, width, height)
    depth_chw = preprocess_depth(depth_m, width, height, max_range_m)
    return np.concatenate([rgb_chw, depth_chw], axis=0)


def depth_collision_score(
    depth_m: np.ndarray,
    threshold_m: float,
    roi_fraction: float = 0.4,
) -> float:
    """Return fraction of central ROI pixels closer than threshold."""
    height, width = depth_m.shape[:2]
    row_start = int(height * (0.5 - roi_fraction / 2))
    row_end = int(height * (0.5 + roi_fraction / 2))
    col_start = int(width * (0.5 - roi_fraction / 2))
    col_end = int(width * (0.5 + roi_fraction / 2))
    roi = depth_m[row_start:row_end, col_start:col_end]
    valid = np.isfinite(roi) & (roi > 0.0)
    if not np.any(valid):
        return 0.0
    close = roi[valid] < threshold_m
    return float(np.mean(close))
