"""标准 mp4（H.264）写入器：imageio + 内置 ffmpeg；保证 ftyp 头与浏览器/ffplay 兼容。"""
from __future__ import annotations


def _try_writer():
    try:
        import imageio
        return imageio
    except Exception:
        return None


class VideoWriter:
    def __init__(self, path: str, fps: float, size: tuple):
        self.path = path
        self.size = tuple(int(x) for x in size)  # (W, H)
        self._W, self._H = self.size
        imageio = _try_writer()
        if imageio is None:
            raise RuntimeError("imageio 未安装，请 pip install imageio imageio-ffmpeg")
        # libx264 标准 mp4，浏览器/ffplay 通用
        self._writer = imageio.get_writer(
            path, fps=float(fps), codec="libx264",
            macro_block_size=1, ffmpeg_params=["-pix_fmt", "yuv420p"],
        )

    def append(self, frame_bgr) -> None:
        # imageio 接受 RGB；BGR→RGB
        if frame_bgr.shape[1] != self._W or frame_bgr.shape[0] != self._H:
            import cv2
            frame_bgr = cv2.resize(frame_bgr, (self._W, self._H))
        import cv2
        self._writer.append_data(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))

    def close(self) -> None:
        try:
            self._writer.close()
        except Exception:
            pass