"""视觉子包，惰性 import cv2（避免无 cv2 环境阻断其他模块）。"""
__all__ = ["TrajectoryDrawer", "VideoWriter"]


def __getattr__(name):
    if name == "TrajectoryDrawer":
        from .drawer import TrajectoryDrawer
        return TrajectoryDrawer
    if name == "VideoWriter":
        from .video_writer import VideoWriter
        return VideoWriter
    raise AttributeError(name)