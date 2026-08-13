"""Capture Isaac Sim viewport frames for demo / batch video export."""

from __future__ import annotations

import numpy as np


def rgb_frame_to_bgr(frame: np.ndarray, *, scale: int = 1) -> np.ndarray:
    """Convert an RGB uint8 frame to BGR for OpenCV video writers."""
    import cv2

    if frame.ndim != 3 or frame.shape[2] < 3:
        raise ValueError(f'Expected HxWx3 RGB frame, got shape {frame.shape}')
    bgr = cv2.cvtColor(frame[:, :, :3], cv2.COLOR_RGB2BGR)
    if scale > 1:
        bgr = cv2.resize(
            bgr,
            (bgr.shape[1] * scale, bgr.shape[0] * scale),
            interpolation=cv2.INTER_LINEAR,
        )
    return bgr


def capture_viewport_bgr(env, *, scale: int = 1) -> np.ndarray | None:
    """Grab one viewport frame from an Isaac Lab DirectRLEnv adapter."""
    if not hasattr(env, 'capture_viewport_frame'):
        return None
    frame = env.capture_viewport_frame()
    if frame is None:
        return None
    return rgb_frame_to_bgr(frame, scale=scale)
