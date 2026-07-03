"""Shared policy inference interface."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass
class InferenceResult:
    """Point- or image-goal inference output."""

    trajectory: np.ndarray
    all_trajectory: np.ndarray
    values: np.ndarray
    trajectory_mask: np.ndarray | None = None
    stopped: bool = False


class PolicyInference(ABC):
    """Lazy-loaded navigation policy for the ROS bridge."""

    def __init__(self, intrinsic: np.ndarray | None = None) -> None:
        self._intrinsic = intrinsic

    @property
    def intrinsic(self) -> np.ndarray | None:
        return self._intrinsic

    @intrinsic.setter
    def intrinsic(self, value: np.ndarray | None) -> None:
        self._intrinsic = value

    @abstractmethod
    def reset(self) -> None:
        """Reset episode memory."""

    @abstractmethod
    def step_pointgoal(
        self,
        goal_xy: np.ndarray,
        rgb_bgr: np.ndarray,
        depth_m: np.ndarray,
    ) -> InferenceResult:
        """Run one point-goal step."""

    def step_imagegoal(
        self,
        goal_rgb: np.ndarray,
        rgb_bgr: np.ndarray,
        depth_m: np.ndarray,
    ) -> InferenceResult:
        """Run one image-goal step (optional per policy)."""
        raise NotImplementedError(f'{type(self).__name__} does not support image goals')

    def step_nogoal(
        self,
        rgb_bgr: np.ndarray,
        depth_m: np.ndarray,
    ) -> InferenceResult:
        """Run one no-goal exploration step (optional per policy)."""
        raise NotImplementedError(f'{type(self).__name__} does not support no-goal mode')

    def step_pixelgoal(
        self,
        pixel_uv: np.ndarray,
        rgb_bgr: np.ndarray,
        depth_m: np.ndarray,
    ) -> InferenceResult:
        """Run one pixel-goal step (optional per policy)."""
        raise NotImplementedError(f'{type(self).__name__} does not support pixel goals')

    def step_point_image_goal(
        self,
        goal_xy: np.ndarray,
        goal_rgb: np.ndarray,
        rgb_bgr: np.ndarray,
        depth_m: np.ndarray,
    ) -> InferenceResult:
        """Run one point+image mixed goal step (optional per policy)."""
        raise NotImplementedError(f'{type(self).__name__} does not support mixed goals')


def to_numpy(value) -> np.ndarray:
    if hasattr(value, 'detach'):
        return value.detach().cpu().numpy()
    return np.asarray(value)
