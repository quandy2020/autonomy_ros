"""回放接口：流式 + 视频拼装。视觉依赖 (cv2/imageio) 惰性。"""
from __future__ import annotations
from pathlib import Path
from typing import Iterator, Tuple
import numpy as np


def _try_visual():
    try:
        from ..visual.drawer import TrajectoryDrawer
        from ..visual.video_writer import VideoWriter
        return TrajectoryDrawer, VideoWriter
    except Exception:
        return None, None


def stream_episodes(ep_loader: Iterator[dict], videos_dir: Path | None = None,
                    fps: float = 10.0, out_size: tuple = (640, 480)) -> Iterator[Tuple[str, np.ndarray, np.ndarray, str]]:
    """逐 episode 流：yield (name, gt, pred, video_path)。
    ep_loader 提供：rgb, gt_px_list, pred_px_list, pose_px_list（每帧像素坐标）。
    """
    drawer_cls, writer_cls = _try_visual()
    for ep in ep_loader:
        name = ep.get("name", "episode")
        rgb_list = ep["rgb"]
        gt_px_list = ep.get("gt_px_list", [])
        pred_px_list = ep.get("pred_px_list", [])
        pose_px_list = ep.get("pose_px_list", [])
        out_uri = ""
        # 汇总全帧 GT/Pred（用于 ADE 计算）
        all_gt = np.vstack(gt_px_list) if gt_px_list else np.zeros((1, 2))
        all_pred = np.vstack(pred_px_list) if pred_px_list else np.zeros((1, 2))
        if videos_dir is not None and drawer_cls is not None:
            videos_dir = Path(videos_dir)
            videos_dir.mkdir(parents=True, exist_ok=True)
            mp4 = videos_dir / f"{name}_traj.mp4"
            drawer = drawer_cls(out_size=out_size)
            vw = writer_cls(str(mp4), fps=fps, size=out_size)
            try:
                for i, rgb in enumerate(rgb_list):
                    gt_px = gt_px_list[i] if i < len(gt_px_list) else np.zeros((1, 2))
                    pred_px = pred_px_list[i] if i < len(pred_px_list) else np.zeros((1, 2))
                    pose_px = pose_px_list[i] if i < len(pose_px_list) else None
                    vw.append(drawer.draw(rgb, gt_px, pred_px, pose_px))
            finally:
                vw.close()
            out_uri = str(mp4)
        yield name, all_gt, all_pred, out_uri