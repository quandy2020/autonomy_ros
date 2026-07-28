"""模型推理封装：直接复用 NavDPNet.predict_nogoal_batch_action_vel。"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
import torch
import cv2

from autonomy_internnav.train.navdp_model import NavDPNet


@dataclass
class InferenceResult:
    samples: np.ndarray       # (N, T, 3) 候选轨迹
    scores: np.ndarray        # (N,) critic 分数
    best: np.ndarray          # (T, 3) critic 最低那条


class NavDPModel:
    """加载 + 单帧推理。"""

    def __init__(self, ckpt_path: str, device: str, sample_num: int, image_size: int = 224):
        self.ckpt = ckpt_path
        self.device = device
        self.sample_num = int(sample_num)
        self.image_size = int(image_size)
        self.net = NavDPNet.from_pretrained(ckpt_path).to(device)
        self.net.eval()

    def _prep(self, rgb: np.ndarray, depth: np.ndarray) -> tuple[torch.Tensor, torch.Tensor]:
        img = cv2.resize(rgb, (self.image_size, self.image_size))
        d = cv2.resize(depth, (self.image_size, self.image_size))
        img = img.astype(np.float32).transpose(2, 0, 1)[None] / 255.0
        d = d.astype(np.float32)[None, None]
        return (torch.as_tensor(img, device=self.device),
                torch.as_tensor(d, device=self.device))

    def infer(self, rgb: np.ndarray, depth: np.ndarray) -> InferenceResult:
        imgs, deps = self._prep(rgb, depth)
        neg, pos = self.net.predict_nogoal_batch_action_vel(
            imgs, deps, sample_num=self.sample_num
        )
        # neg / pos 形状: (8, predict_size, 3) 已排序
        # 扩成完整 sample_num 以便 selector 复用
        samples = neg.cpu().numpy().astype(np.float32)
        scores = np.arange(samples.shape[0], dtype=np.float32)  # 越小越好
        return InferenceResult(samples=samples, scores=scores, best=samples[0])