"""轨迹回放可视化后端，matplotlib 内存安全。"""
from __future__ import annotations
import io
import base64
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


class TrajectoryVisualizer:
    def render(self, gt: np.ndarray, pred: np.ndarray, title: str) -> str:
        """gt/pred (N,3) → 返回 base64 PNG data URI；必须显式 close。"""
        fig, ax = plt.subplots(figsize=(6, 6))
        try:
            if gt.size:
                ax.plot(gt[:, 1], gt[:, 0], "-g", label="GT", linewidth=2)
            if pred.size:
                ax.plot(pred[:, 1], pred[:, 0], "-r", label="Pred", linewidth=2)
                ax.scatter(pred[0, 1], pred[0, 0], c="b", marker="o", s=40, label="Start")
                ax.scatter(pred[-1, 1], pred[-1, 0], c="m", marker="x", s=80, label="End")
            ax.set_aspect("equal")
            ax.set_xlabel("y"); ax.set_ylabel("x")
            ax.set_title(title); ax.legend(loc="best", fontsize=8)
            buf = io.BytesIO()
            fig.savefig(buf, format="png", dpi=110, bbox_inches="tight")
            return "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode()
        finally:
            plt.close(fig)  # 内存安全：显式关闭