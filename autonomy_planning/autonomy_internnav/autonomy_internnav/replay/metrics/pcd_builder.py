"""深度图 → 世界系点云（含体素降采样）。"""
from __future__ import annotations
import numpy as np
from scipy.spatial import cKDTree


class PointCloudBuilder:
    """backproject + cam->world + 体素降采样。"""

    def __init__(self, voxel_size: float = 0.05):
        self.voxel = float(voxel_size)

    def build(self, depth: np.ndarray, intrinsic: np.ndarray, extrinsic: np.ndarray) -> np.ndarray:
        """返回 (N,3) float32 世界系点云；depth<=0 的像素丢弃。"""
        fx, fy = intrinsic[0, 0], intrinsic[1, 1]
        cx, cy = intrinsic[0, 2], intrinsic[1, 2]
        h, w = depth.shape
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        z = depth
        mask = z > 1e-3
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        pts_cam = np.stack([x[mask], y[mask], z[mask], np.ones_like(z[mask])], axis=0)  # (4,N)
        R, t = extrinsic[:3, :3], extrinsic[:3, 3]
        pts_world = (R @ pts_cam[:3]) + t[:, None]
        return self._voxel_downsample(pts_world.T.astype(np.float32))

    def _voxel_downsample(self, pts: np.ndarray) -> np.ndarray:
        if pts.shape[0] == 0 or self.voxel <= 0:
            return pts
        keys = np.floor(pts / self.voxel).astype(np.int64)
        _, idx = np.unique(keys, axis=0, return_index=True)
        return pts[np.sort(idx)]