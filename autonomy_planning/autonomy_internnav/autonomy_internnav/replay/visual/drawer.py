"""单帧轨迹叠加：3D 世界点 → 相机系 → 像素坐标，画到 RGB 上。"""
from __future__ import annotations
import numpy as np
import cv2


class TrajectoryDrawer:
    def __init__(self, out_size: tuple = (640, 480)):
        self.out_w, self.out_h = int(out_size[0]), int(out_size[1])
        self.color_gt = (0, 255, 0)     # 绿
        self.color_pred = (0, 0, 255)   # 红
        self.color_pose = (255, 0, 0)   # 蓝
        self.font = cv2.FONT_HERSHEY_SIMPLEX

    def draw(self, rgb: np.ndarray,
             gt_px: np.ndarray, pred_px: np.ndarray,
             pose_px: np.ndarray | None = None) -> np.ndarray:
        """gt_px/pred_px: (N,2) 像素坐标（已投影到图像系）；pose_px: (2,) 像素坐标。"""
        if rgb.shape[1] != self.out_w or rgb.shape[0] != self.out_h:
            bgr = cv2.resize(rgb, (self.out_w, self.out_h), interpolation=cv2.INTER_AREA)
        else:
            bgr = rgb.copy()
        bgr = cv2.cvtColor(bgr, cv2.COLOR_RGB2BGR) if rgb.shape[2] == 3 else bgr
        # 缩放 gt/pred 像素坐标（原图→out_size）
        sx = self.out_w / rgb.shape[1] if rgb.shape[1] != self.out_w else 1.0
        sy = self.out_h / rgb.shape[0] if rgb.shape[0] != self.out_h else 1.0
        def rescale(pts):
            out = pts.astype(np.float64).copy()
            out[:, 0] *= sx
            out[:, 1] *= sy
            return out.astype(np.int32)
        # GT 绿
        if len(gt_px) >= 2:
            self._draw_polyline(bgr, rescale(gt_px), self.color_gt, thickness=3)
        elif len(gt_px) == 1:
            p = rescale(gt_px)[0]
            cv2.circle(bgr, tuple(p), 6, self.color_gt, -1)
        # Pred 红
        if len(pred_px) >= 2:
            self._draw_polyline(bgr, rescale(pred_px), self.color_pred, thickness=3)
        elif len(pred_px) == 1:
            p = rescale(pred_px)[0]
            cv2.circle(bgr, tuple(p), 6, self.color_pred, -1)
        # Pose 蓝点
        if pose_px is not None:
            pp = np.array([pose_px[0] * sx, pose_px[1] * sy], dtype=np.int32)
            cv2.circle(bgr, tuple(pp), 8, self.color_pose, -1)
            cv2.circle(bgr, tuple(pp), 8, (255, 255, 255), 2)
        # HUD
        cv2.rectangle(bgr, (5, 5), (340, 70), (0, 0, 0), -1)
        cv2.putText(bgr, "GT=green  Pred=red  Pose=blue", (10, 25),
                    self.font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(bgr, f"GT={len(gt_px)} pts  Pred={len(pred_px)} pts",
                    (10, 55), self.font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        return bgr

    @staticmethod
    def _draw_polyline(img, pts: np.ndarray, color, thickness: int = 3) -> None:
        if len(pts) < 2:
            return
        H, W = img.shape[:2]
        m = (pts[:, 0] >= 0) & (pts[:, 0] < W) & (pts[:, 1] >= 0) & (pts[:, 1] < H)
        pts = pts[m]
        if len(pts) < 2:
            return
        cv2.polylines(img, [pts], isClosed=False, color=color, thickness=thickness, lineType=cv2.LINE_AA)


def world_to_pixel(pts3d_world: np.ndarray, extrinsic: np.ndarray, intrinsic: np.ndarray) -> np.ndarray:
    """3D 世界点 (N,3) → 像素 (N,2)。extrinsic: cam→world (4,4)。intrinsic: (3,3)。"""
    # cam→world 逆 = world→cam
    T_wc = np.linalg.inv(extrinsic)  # (4,4)
    pts_h = np.concatenate([pts3d_world, np.ones((len(pts3d_world), 1))], axis=1)  # (N,4)
    pts_cam = (T_wc @ pts_h.T).T[:, :3]  # (N,3)
    # 只取 z > 0 的点（相机前方）
    valid = pts_cam[:, 2] > 0.01
    # 投影
    uv = (intrinsic @ pts_cam.T).T  # (N,3)
    uv = uv[:, :2] / np.maximum(uv[:, 2:3], 1e-6)  # (N,2)
    uv[~valid] = -1  # 标记无效
    return uv.astype(np.float32), valid