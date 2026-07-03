# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""RGB-D and goal preprocessing for NavDP inference."""

from __future__ import annotations

import cv2
import numpy as np

_MAX_DEPTH_M = 5.0
_MIN_DEPTH_M = 0.1
_PIXEL_GOAL_RADIUS = 10


class ObservationPreprocessor:
    """Resize, pad, and normalize observations to the model input size."""

    def __init__(self, image_size: int = 224) -> None:
        self._image_size = image_size

    @property
    def image_size(self) -> int:
        return self._image_size

    def process_images(self, images: np.ndarray) -> np.ndarray:
        """Convert uint8 BGR batch ``(B, H, W, C)`` to float32 ``(B, S, S, C)``."""
        if images.ndim != 4:
            raise ValueError(f'images must be 4-D, got shape {images.shape}')
        height, width = images.shape[1], images.shape[2]
        scale = self._image_size / max(height, width)
        outputs = []
        for image in images:
            resized = cv2.resize(image, (-1, -1), fx=scale, fy=scale)
            pad_w = max((self._image_size - resized.shape[1]) // 2, 0)
            pad_h = max((self._image_size - resized.shape[0]) // 2, 0)
            padded = np.pad(
                resized,
                ((pad_h, pad_h), (pad_w, pad_w), (0, 0)),
                mode='constant',
                constant_values=0,
            )
            square = cv2.resize(padded, (self._image_size, self._image_size))
            outputs.append(square.astype(np.float32) / 255.0)
        return np.asarray(outputs)

    def process_depths(self, depths: np.ndarray) -> np.ndarray:
        """Convert depth batch ``(B, H, W, C)`` to float32 ``(B, S, S, 1)``."""
        if depths.ndim != 4:
            raise ValueError(f'depths must be 4-D, got shape {depths.shape}')
        depths = depths.copy()
        depths[depths == np.inf] = 0.0
        height, width = depths.shape[1], depths.shape[2]
        scale = self._image_size / max(height, width)
        outputs = []
        for depth in depths:
            resized = cv2.resize(depth, (-1, -1), fx=scale, fy=scale)
            pad_w = max((self._image_size - resized.shape[1]) // 2, 0)
            pad_h = max((self._image_size - resized.shape[0]) // 2, 0)
            padded = np.pad(
                resized,
                ((pad_h, pad_h), (pad_w, pad_w)),
                mode='constant',
                constant_values=0,
            )
            square = cv2.resize(padded, (self._image_size, self._image_size))
            square[square > _MAX_DEPTH_M] = 0.0
            square[square < _MIN_DEPTH_M] = 0.0
            outputs.append(square[:, :, np.newaxis])
        return np.asarray(outputs)

    def process_pixel_goals(
        self,
        pixel_coords: np.ndarray,
        input_images: np.ndarray,
    ) -> np.ndarray:
        """Build pixel-goal masks and return grayscale goal maps."""
        height, width = input_images.shape[1], input_images.shape[2]
        scale = self._image_size / max(height, width)
        outputs = []
        radius = _PIXEL_GOAL_RADIUS
        for pixel_coord, input_image in zip(pixel_coords, input_images):
            panel = np.zeros_like(input_image, dtype=np.uint8)
            min_x = int(pixel_coord[0]) - radius
            min_y = int(pixel_coord[1]) - radius
            max_x = int(pixel_coord[0]) + radius
            max_y = int(pixel_coord[1]) + radius

            if min_x <= 0:
                panel[:, :radius] = 255
            elif min_y <= 0:
                panel[:radius, :] = 255
            elif max_x >= panel.shape[1]:
                panel[:, panel.shape[1] - radius:] = 255
            elif max_y >= panel.shape[0]:
                panel[panel.shape[0] - radius:, :] = 255
            elif (
                min_x > 0
                and min_y > 0
                and max_x < panel.shape[1]
                and max_y < panel.shape[0]
            ):
                panel[min_y:max_y, min_x:max_x] = 255

            resized = cv2.resize(
                panel, (-1, -1), fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST,
            )
            pad_w = max((self._image_size - resized.shape[1]) // 2, 0)
            pad_h = max((self._image_size - resized.shape[0]) // 2, 0)
            padded = np.pad(
                resized,
                ((pad_h, pad_h), (pad_w, pad_w), (0, 0)),
                mode='constant',
                constant_values=0,
            )
            square = cv2.resize(padded, (self._image_size, self._image_size))
            outputs.append(square.astype(np.float32) / 255.0)
        return np.asarray(outputs).mean(axis=-1)

    @staticmethod
    def clip_point_goals(goals: np.ndarray) -> np.ndarray:
        """Clip point goals to the NavDP training range."""
        clipped = goals.clip(-10.0, 10.0)
        clipped[:, 0] = np.clip(clipped[:, 0], 0.0, 10.0)
        return clipped
