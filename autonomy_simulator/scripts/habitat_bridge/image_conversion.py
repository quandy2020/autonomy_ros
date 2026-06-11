# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0

"""Convert Habitat sensor arrays to ROS sensor_msgs using cv_bridge."""

from __future__ import annotations

import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

_BRIDGE = CvBridge()


def make_camera_info(
    header: Header,
    width: int,
    height: int,
    horizontal_fov_deg: float = 79.0,
) -> CameraInfo:
    """Create a pinhole CameraInfo message from image size and HFOV."""
    hfov = np.deg2rad(horizontal_fov_deg)
    fx = 0.5 * width / np.tan(0.5 * hfov)
    fy = fx
    cx = (width - 1) * 0.5
    cy = (height - 1) * 0.5

    info = CameraInfo()
    info.header = header
    info.width = width
    info.height = height
    info.distortion_model = 'plumb_bob'
    info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return info


def _stamp_image(message: Image, header: Header) -> Image:
    message.header = header
    return message


def numpy_rgb_to_image(header: Header, rgb: np.ndarray) -> Image:
    """Convert an RGB uint8 array to sensor_msgs/Image (bgr8) via cv_bridge."""
    if rgb.ndim == 3 and rgb.shape[2] == 4:
        rgb = rgb[:, :, :3]
    if rgb.dtype != np.uint8:
        rgb = np.clip(rgb * (255.0 if rgb.max() <= 1.0 else 1.0), 0, 255).astype(
            np.uint8
        )
    bgr = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2BGR)
    return _stamp_image(_BRIDGE.cv2_to_imgmsg(bgr, encoding='bgr8'), header)


def numpy_depth_to_image(header: Header, depth: np.ndarray) -> Image:
    """Convert a depth float array in meters to sensor_msgs/Image (32FC1)."""
    depth = np.ascontiguousarray(np.squeeze(depth).astype(np.float32))
    return _stamp_image(
        _BRIDGE.cv2_to_imgmsg(depth, encoding='32FC1'),
        header,
    )


def normalize_semantic_ids(semantic: np.ndarray) -> np.ndarray:
    """Return a contiguous (H, W) uint32 semantic instance-id map."""
    ids = np.squeeze(semantic)
    if ids.dtype != np.uint32:
        ids = ids.astype(np.uint32)
    return np.ascontiguousarray(ids)


def colorize_semantic(semantic: np.ndarray) -> np.ndarray:
    """Map semantic instance ids to an RGB visualization image."""
    ids = normalize_semantic_ids(semantic)
    rgb = np.zeros((*ids.shape, 3), dtype=np.uint8)
    unique_ids = np.unique(ids)
    for instance_id in unique_ids:
        if instance_id == 0:
            continue
        mask = ids == instance_id
        seed = int(instance_id)
        rgb[mask, 0] = (seed * 2654435761) % 256
        rgb[mask, 1] = (seed * 2246822519) % 256
        rgb[mask, 2] = (seed * 3266489917) % 256
    return rgb
