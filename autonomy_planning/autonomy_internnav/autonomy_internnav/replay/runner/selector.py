"""Critic 选最优：argmin scores。"""
from __future__ import annotations
import numpy as np


class CriticSelector:
    def select(self, samples: np.ndarray, scores: np.ndarray) -> np.ndarray:
        return samples[int(np.argmin(scores))]