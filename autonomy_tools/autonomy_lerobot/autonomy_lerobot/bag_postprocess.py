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

"""Offline bag post-processing for semantic masks, pointclouds, and previews."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo

from autonomy_lerobot.conversions import depth_rgb_to_pointcloud_array
from autonomy_lerobot.observation import KEY_POINTCLOUD, KEY_SEMANTIC
from autonomy_lerobot.sam3_segmenter import Sam3GroundSegmenter


@dataclass(frozen=True)
class BagPreviewSample:
    """One sampled visualization snapshot."""

    rgb: np.ndarray
    depth: np.ndarray | None
    ground_mask: np.ndarray | None
    raw_pointcloud: np.ndarray | None
    filtered_pointcloud: np.ndarray | None
    # bag2data-style (N,3) points used only for ground_segmentation.png TopView
    viz_pointcloud: np.ndarray | None = None


@dataclass(frozen=True)
class BagPostprocessConfig:
    """Options for optional offline post-processing."""

    record_pointcloud: bool
    save_segmentation_viz: bool
    pointcloud_stride: int
    max_pointcloud_points: int
    depth_min_m: float
    depth_max_m: float


class BagPostprocessor:
    """Apply optional SAM3 and pointcloud post-processing to decoded frames."""

    def __init__(
        self,
        cfg: BagPostprocessConfig,
        segmenter: Sam3GroundSegmenter | None = None,
    ) -> None:
        self._cfg = cfg
        self._segmenter = segmenter
        self.reset_episode()

    @property
    def has_sam3(self) -> bool:
        return self._segmenter is not None

    def reset_episode(self) -> None:
        """Reset per-episode previews collected for later visualization."""
        self._episode_samples: list[BagPreviewSample] = []

    def build_preview_sample(
        self,
        *,
        rgb: np.ndarray,
        depth: np.ndarray,
        camera_info: CameraInfo,
    ) -> BagPreviewSample:
        """Build one preview sample with mask and pointcloud products."""
        ground_mask: np.ndarray | None = None
        if self._segmenter is not None:
            ground_mask = self._segmenter.segment_ground(rgb)

        raw_pointcloud: np.ndarray | None = None
        filtered_pointcloud: np.ndarray | None = None
        if self._cfg.record_pointcloud:
            raw_pointcloud = depth_rgb_to_pointcloud_array(
                depth,
                camera_info,
                rgb=rgb,
                max_points=self._cfg.max_pointcloud_points,
                depth_min=max(self._cfg.depth_min_m, 0.05),
                depth_max=self._cfg.depth_max_m,
                stride=self._cfg.pointcloud_stride,
            )
            filtered_pointcloud = depth_rgb_to_pointcloud_array(
                depth,
                camera_info,
                rgb=rgb,
                valid_mask=(
                    self._segmenter.nonground_mask(ground_mask)
                    if ground_mask is not None and self._segmenter is not None
                    else None
                ),
                max_points=self._cfg.max_pointcloud_points,
                depth_min=max(self._cfg.depth_min_m, 0.05),
                depth_max=self._cfg.depth_max_m,
                stride=self._cfg.pointcloud_stride,
            )

        viz_pointcloud = _bag2data_depth_to_pointcloud(
            depth,
            camera_info,
            ground_mask,
            max_points=1000,
        )

        return BagPreviewSample(
            rgb=rgb.copy(),
            depth=depth.copy(),
            ground_mask=None if ground_mask is None else ground_mask.copy(),
            raw_pointcloud=raw_pointcloud,
            filtered_pointcloud=filtered_pointcloud,
            viz_pointcloud=viz_pointcloud,
        )

    def process_frame(
        self,
        *,
        frame: dict[str, np.ndarray | str],
        rgb: np.ndarray,
        depth: np.ndarray,
        camera_info: CameraInfo,
    ) -> None:
        """Mutate one frame with optional semantic and pointcloud products."""
        sample = self.build_preview_sample(rgb=rgb, depth=depth, camera_info=camera_info)
        if sample.ground_mask is not None:
            frame[KEY_SEMANTIC] = self._segmenter.semantic_rgb_from_ground_mask(
                sample.ground_mask
            )
            # Collect every processed frame; write_preview_samples picks first/mid/last.
            self._episode_samples.append(sample)

        if sample.filtered_pointcloud is None:
            return
        frame[KEY_POINTCLOUD] = sample.filtered_pointcloud

    def write_episode_artifacts(self, run_root: Path) -> None:
        """Write optional per-run semantic and pointcloud preview artifacts."""
        self.write_preview_samples(run_root, self._episode_samples)

    def write_preview_samples(self, run_root: Path, samples: list[BagPreviewSample]) -> None:
        """Write bag2data-compatible `ground_segmentation.png`.

        Layout matches bag2data `_save_visualization`:
        - Select first/middle/last samples ⇒ 3 stacked rows
        - Each row: RGB+Mask | Depth | PointCloud(TopView)
        """
        if not self._cfg.save_segmentation_viz or not samples:
            return
        meta_dir = run_root / 'meta'
        meta_dir.mkdir(parents=True, exist_ok=True)

        # Same selection rule as bag2data: first / middle / last of all processed frames.
        indices = (
            [0, len(samples) // 2, len(samples) - 1]
            if len(samples) >= 3
            else list(range(len(samples)))
        )
        selected = [samples[index] for index in indices]
        height, width = selected[0].rgb.shape[:2]
        rows: list[np.ndarray] = []
        for sample in selected:
            rows.append(_bag2data_style_row(sample, width=width, height=height))
        big_img = np.concatenate(rows, axis=0)
        font = cv2.FONT_HERSHEY_SIMPLEX
        for index, label in enumerate(
            ['RGB+Mask', 'Depth', 'PointCloud(TopView)']
        ):
            cv2.putText(
                big_img,
                label,
                (index * width + 10, 25),
                font,
                0.7,
                (0, 0, 0),
                2,
            )
        cv2.imwrite(
            str(meta_dir / 'ground_segmentation.png'),
            cv2.cvtColor(big_img, cv2.COLOR_RGB2BGR),
        )

    def write_pointcloud_ply(
        self,
        run_root: Path,
        *,
        frame_index: int,
        sample: BagPreviewSample,
    ) -> None:
        """Write one filtered pointcloud sample as `frame_xxxx.ply`."""
        if sample.filtered_pointcloud is None:
            return
        points = sample.filtered_pointcloud[:, :3]
        valid = np.any(points != 0.0, axis=1)
        xyz = points[valid].astype(np.float32, copy=False)
        pointcloud_dir = run_root / 'pointcloud'
        pointcloud_dir.mkdir(parents=True, exist_ok=True)
        _save_ply_xyz(xyz, pointcloud_dir / f'frame_{frame_index:04d}.ply')


def _bag2data_style_row(
    sample: BagPreviewSample,
    *,
    width: int,
    height: int,
) -> np.ndarray:
    """Build one bag2data `_save_visualization` row."""
    ground_mask = sample.ground_mask
    rgb = sample.rgb

    # Left: RGB + red ground overlay.
    overlay = rgb.copy()
    if ground_mask is not None:
        overlay[ground_mask == 255] = [255, 0, 0]
    left = cv2.addWeighted(rgb, 0.6, overlay, 0.4, 0)

    # Middle: depth colormap with ground set to black.
    # Matches bag2data exactly: ColorMap keeps OpenCV BGR channel order and the
    # full collage is later saved via RGB2BGR (same quirk as bag2data).
    depth_vis = _depth_clean_colormap(
        sample.depth,
        ground_mask,
        height=height,
        width=width,
    )

    # Right: top-view pointcloud (z forward, x lateral).
    pts = sample.viz_pointcloud
    if pts is None and sample.filtered_pointcloud is not None:
        pts = sample.filtered_pointcloud[:, :3]
    pc_img = _render_pointcloud_baselink_image(pts, width=width, height=height)
    return np.concatenate([left, depth_vis, pc_img], axis=1)


def _bag2data_depth_to_pointcloud(
    depth: np.ndarray,
    camera_info: CameraInfo,
    ground_mask: np.ndarray | None,
    *,
    max_points: int = 1000,
    extrinsic: np.ndarray | None = None,
) -> np.ndarray:
    """Mirror bag2data ``GroundRemover.depth_to_pointcloud``.

    Depth is converted to per-frame uint8 gray (same as depth video / viz),
    ground pixels are zeroed, then back-projected with ``depth > 5`` and
    uniform subsample to ``max_points``.
    """
    depth_u8 = _depth_to_uint8_gray(depth)
    if ground_mask is not None:
        depth_clean = depth_u8.copy()
        depth_clean[ground_mask == 255] = 0
    else:
        depth_clean = depth_u8
        ground_mask = np.zeros(depth_clean.shape, dtype=np.uint8)

    h, w = depth_clean.shape
    valid = (ground_mask == 0) & (depth_clean > 5.0)
    if not np.any(valid):
        return np.empty((0, 3), dtype=np.float32)

    v_idx, u_idx = np.where(valid)
    total_valid = int(v_idx.shape[0])
    if total_valid > max_points:
        step = total_valid / float(max_points)
        chosen = (np.arange(max_points) * step).astype(np.int64)
        v_idx = v_idx[chosen]
        u_idx = u_idx[chosen]

    d_s = depth_clean[v_idx, u_idx].astype(np.float32)
    fx = float(camera_info.k[0])
    fy = float(camera_info.k[4])
    cx = float(camera_info.k[2])
    cy = float(camera_info.k[5])
    x = (u_idx.astype(np.float32) - cx) * d_s / fx
    y = (v_idx.astype(np.float32) - cy) * d_s / fy
    z = d_s
    pts = np.stack([x, y, z], axis=1)

    if extrinsic is not None:
        transform = np.asarray(extrinsic, dtype=np.float32).reshape(4, 4)
        rotation = transform[:3, :3]
        translation = transform[:3, 3]
        pts = (rotation @ pts.T).T + translation

    dist = np.linalg.norm(pts, axis=1)
    return pts[dist > 0.5].astype(np.float32, copy=False)


def _depth_to_uint8_gray(depth: np.ndarray) -> np.ndarray:
    """Convert depth to uint8 gray the same way bag2data encodes depth video."""
    if depth.dtype == np.uint8:
        return depth.copy()
    safe = np.nan_to_num(depth.astype(np.float32), nan=0.0, posinf=0.0, neginf=0.0)
    d_min = float(safe.min())
    d_max = float(safe.max())
    if d_max > d_min:
        return ((safe - d_min) / (d_max - d_min) * 255.0).astype(np.uint8)
    return np.zeros_like(safe, dtype=np.uint8)


def _depth_clean_colormap(
    depth: np.ndarray | None,
    ground_mask: np.ndarray | None,
    *,
    height: int,
    width: int,
) -> np.ndarray:
    """Match bag2data depth panel: clean depth + JET + black ground."""
    if depth is None:
        return np.zeros((height, width, 3), dtype=np.uint8)

    depth_clean = _depth_to_uint8_gray(depth)
    if ground_mask is not None:
        depth_clean[ground_mask == 255] = 0
    # Keep applyColorMap BGR layout; bag2data does the same before RGB2BGR save.
    depth_vis = cv2.applyColorMap(depth_clean.astype(np.uint8), cv2.COLORMAP_JET)
    if ground_mask is not None:
        depth_vis[ground_mask == 255] = [0, 0, 0]
    return depth_vis


def _render_pointcloud_baselink_image(
    points: np.ndarray | None,
    *,
    width: int = 640,
    height: int = 480,
) -> np.ndarray:
    """Render top view matching bag2data PointCloud(TopView) panel."""
    canvas = np.full((height, width, 3), 255, dtype=np.uint8)
    if points is None or points.size == 0:
        return canvas
    xyz = np.asarray(points, dtype=np.float32)[:, :3]
    if xyz.ndim != 2 or xyz.shape[0] == 0:
        return canvas
    valid = np.any(xyz != 0.0, axis=1)
    xyz = xyz[valid]
    if xyz.size == 0:
        return canvas

    x3 = xyz[:, 0]
    y3 = xyz[:, 1]
    z3 = xyz[:, 2]
    dist = np.sqrt(x3 * x3 + y3 * y3 + z3 * z3)
    far = dist > 0.5
    x3 = x3[far]
    z3 = z3[far]
    # Top view: z forward (up in image), x lateral (right in image).
    fwd, lat = z3, x3
    front = fwd > 0.0
    fwd = fwd[front]
    lat = lat[front]
    if fwd.size == 0:
        return canvas

    fwd_range = max(float(fwd.max() - fwd.min()), 0.1)
    lat_range = max(float(lat.max() - lat.min()), 0.1)
    scale = min(width * 0.8 / lat_range, height * 0.7 / fwd_range)
    origin_x = width // 2
    origin_y = height - 20
    px = (origin_x + lat * scale).astype(np.int32)
    py = (origin_y - fwd * scale).astype(np.int32)
    valid_px = (px >= 0) & (px < width) & (py >= 0) & (py < height)
    px = px[valid_px]
    py = py[valid_px]

    for x_coord, y_coord in zip(px, py, strict=False):
        cv2.circle(canvas, (int(x_coord), int(y_coord)), 3, (0, 128, 255), -1)
    cv2.circle(canvas, (origin_x, origin_y), 12, (0, 255, 0), 2)
    cv2.arrowedLine(
        canvas,
        (origin_x, origin_y - 14),
        (origin_x, origin_y - 54),
        (0, 0, 255),
        3,
        tipLength=0.15,
    )
    return canvas


def _save_ply_xyz(points: np.ndarray, path: Path) -> None:
    """Save `(N, 3)` pointcloud to binary little-endian PLY."""
    xyz = np.asarray(points, dtype=np.float32)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError('points must have shape (N, 3)')
    header = (
        'ply\n'
        'format binary_little_endian 1.0\n'
        f'element vertex {xyz.shape[0]}\n'
        'property float x\n'
        'property float y\n'
        'property float z\n'
        'end_header\n'
    )
    with path.open('wb') as handle:
        handle.write(header.encode('ascii'))
        xyz.tofile(handle)
