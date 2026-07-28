"""ADE / FDE / Lateral 偏差指标累计器。"""
from __future__ import annotations
import numpy as np


class DeviationMetric:
    def __init__(self):
        self.ade_buf, self.fde_buf, self.lat_buf = [], [], []

    def update(self, pred: np.ndarray, gt: np.ndarray) -> None:
        if pred.shape != gt.shape or pred.shape[0] == 0:
            return
        self.ade_buf.append(float(np.linalg.norm(pred - gt, axis=-1).mean()))
        self.fde_buf.append(float(np.linalg.norm(pred[-1] - gt[-1])))
        self.lat_buf.append(float(self._lateral(pred, gt)))

    @staticmethod
    def _lateral(pred: np.ndarray, gt: np.ndarray) -> float:
        d = gt[-1] - gt[0]
        n = np.linalg.norm(d[:2]) + 1e-9
        tangent = d[:2] / n
        normal = np.array([-tangent[1], tangent[0]])
        diff = (pred[:, :2] - gt[:, :2]) @ normal
        return float(np.abs(diff).mean())

    def compute(self) -> dict:
        if not self.ade_buf:
            return {'ade': 0.0, 'fde': 0.0, 'lateral': 0.0}
        return {
            'ade': float(np.mean(self.ade_buf)),
            'fde': float(np.mean(self.fde_buf)),
            'lateral': float(np.mean(self.lat_buf)),
        }