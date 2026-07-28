"""安全指标：单帧碰撞计数（轨迹上碰撞点 >= 阈值即判碰撞）。"""
from __future__ import annotations
import numpy as np
from scipy.spatial import cKDTree


class SafetyMetric:
    """pred 与 gt 各跑一份，最后取 collision_rate 做对比。"""

    def __init__(self, safe_radius: float, collision_threshold: int):
        self.r = float(safe_radius)
        self.k = int(collision_threshold)
        self.pred_hit, self.pred_n = 0, 0
        self.gt_hit, self.gt_n = 0, 0

    def update(self, pred_traj: np.ndarray, pcd_world: np.ndarray) -> None:
        self._accum(pred_traj, pcd_world, attr=('pred_hit', 'pred_n'))

    def update_gt(self, gt_traj: np.ndarray, pcd_world: np.ndarray) -> None:
        self._accum(gt_traj, pcd_world, attr=('gt_hit', 'gt_n'))

    def _accum(self, traj: np.ndarray, pcd: np.ndarray, attr) -> None:
        if traj is None or traj.shape[0] == 0 or pcd.shape[0] == 0:
            return
        tree = cKDTree(pcd[:, :3])
        d, _ = tree.query(traj[:, :3], k=1, workers=1)
        hit_count = int(np.sum(d < self.r))
        if hit_count >= self.k:
            setattr(self, attr[0], getattr(self, attr[0]) + 1)
        setattr(self, attr[1], getattr(self, attr[1]) + 1)

    def compute(self) -> dict:
        return {
            'pred_collision_count': self.pred_hit,
            'pred_total_frames': self.pred_n,
            'pred_collision_rate': (self.pred_hit / self.pred_n) if self.pred_n else 0.0,
            'gt_collision_count': self.gt_hit,
            'gt_total_frames': self.gt_n,
            'gt_collision_rate': (self.gt_hit / self.gt_n) if self.gt_n else 0.0,
        }