"""局部增量轨迹 → 世界系（pose 锚定）。"""
from __future__ import annotations
import numpy as np


class WorldFrameConverter:
    def to_world(self, local_traj: np.ndarray, base_extrinsic: np.ndarray) -> np.ndarray:
        """local_traj (T,3) 累计位移 (x,y,yaw) 拼到 base_extrinsic 上。"""
        if local_traj.shape[0] == 0:
            return local_traj.copy()
        R = base_extrinsic[:3, :3]
        t = base_extrinsic[:3, 3]
        xy_world = (R[:2, :2] @ local_traj[:, :2].T).T + t[None, :2]
        return np.concatenate([xy_world, local_traj[:, 2:3]], axis=1)