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

"""NavDP runtime agent: preprocessing, memory, inference, and overlay."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

import cv2
import numpy as np
import torch

from autonomy_internnav.navdp.colormap import critic_value_to_bgr_u8
from autonomy_internnav.navdp.preprocessing import ObservationPreprocessor


class NavDP_Agent:
    """NavDP diffusion policy agent with temporal RGB-D memory."""

    def __init__(
        self,
        image_intrinsic: np.ndarray,
        image_size: int = 224,
        memory_size: int = 8,
        predict_size: int = 24,
        temporal_depth: int = 16,
        heads: int = 8,
        token_dim: int = 384,
        navi_model: str = './100.ckpt',
        device: str = 'cuda:0',
    ) -> None:
        self.image_intrinsic = image_intrinsic
        self.device = device
        self.predict_size = predict_size
        self.image_size = image_size
        self.memory_size = memory_size
        self.project_overlay = True
        self._prep = ObservationPreprocessor(image_size)
        from autonomy_internnav.navdp.policy import NavDP_Policy

        self.navi_former = NavDP_Policy(
            image_size=image_size,
            memory_size=memory_size,
            predict_size=predict_size,
            temporal_depth=temporal_depth,
            heads=heads,
            token_dim=token_dim,
            device=device,
        )
        self.navi_former.load_state_dict(
            torch.load(navi_model, map_location=device), strict=False,
        )
        self.navi_former.to(device)
        self.navi_former.eval()
        self.batch_size = 1
        self.stop_threshold = -3.0
        self.sample_num = 16
        self.memory_queue: list[list[np.ndarray]] = []

    def reset(self, batch_size: int, threshold: float, sample_num: int = 16) -> None:
        self.batch_size = batch_size
        self.stop_threshold = threshold
        self.sample_num = sample_num
        self.memory_queue = [[] for _ in range(batch_size)]
        self.project_overlay = True

    def reset_env(self, env_index: int) -> None:
        self.memory_queue[env_index] = []

    def project_trajectory(
        self,
        images: np.ndarray,
        trajectories: np.ndarray,
        values: np.ndarray,
        max_samples: int | None = None,
    ) -> np.ndarray:
        intrinsic = self.image_intrinsic
        masks: list[np.ndarray] = []
        for batch_i in range(images.shape[0]):
            canvas = np.array(images[batch_i])
            trajs = trajectories[batch_i, :, :, 0:2]
            vals = values[batch_i]
            pairs = list(zip(trajs, vals))
            if max_samples is not None and len(pairs) > max_samples:
                pairs = sorted(pairs, key=lambda p: float(p[1]), reverse=True)[:max_samples]
            h = canvas.shape[0]
            fx, fy = intrinsic[0, 0], intrinsic[1, 1]
            cx, cy = intrinsic[0, 2], intrinsic[1, 2]
            for waypoints, value in pairs:
                color = critic_value_to_bgr_u8(float(value))
                pts = np.zeros((waypoints.shape[0], 3), dtype=np.float64) - 0.2
                pts[:, 0:2] = waypoints
                pts[:, 1] = -pts[:, 1]
                depth = pts[:, 0] + 1e-8
                cam_x = fx * pts[:, 1] / depth + cx
                cam_z = h - 1 - fy * pts[:, 2] / depth - cy
                for seg in range(len(cam_x) - 1):
                    x0, z0 = int(cam_x[seg]), int(cam_z[seg])
                    x1, z1 = int(cam_x[seg + 1]), int(cam_z[seg + 1])
                    if x0 <= 0 or z0 <= 0 or x1 <= 0 or z1 <= 0:
                        continue
                    try:
                        cv2.line(canvas, (x0, z0), (x1, z1), color, 5)
                    except cv2.error:
                        pass
            masks.append(canvas)
        return np.concatenate(masks, axis=1)

    def _run_step(
        self,
        images: np.ndarray,
        depths: np.ndarray,
        predict: Callable[..., tuple[np.ndarray, ...]],
        *predict_args: Any,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray | None]:
        proc_img = self._prep.process_images(images)
        proc_dep = self._prep.process_depths(depths)
        batches = []
        for env_idx, queue in enumerate(self.memory_queue):
            if len(queue) < self.memory_size:
                queue.append(proc_img[env_idx])
                batch = np.array(queue)
                pad = self.memory_size - batch.shape[0]
                batch = np.pad(batch, ((pad, 0), (0, 0), (0, 0), (0, 0)))
            else:
                queue.pop(0)
                queue.append(proc_img[env_idx])
                batch = np.array(queue)
            batches.append(batch)
        temporal = np.array(batches)
        all_traj, all_vals, good_traj, _ = predict(
            *predict_args, temporal, proc_dep, sample_num=self.sample_num,
        )
        if all_vals.max() < self.stop_threshold:
            good_traj[:, :, :, 0] = 0.0
            good_traj[:, :, :, 1] = np.sign(good_traj[:, :, :, 1].mean())
        mask = None
        if self.project_overlay:
            mask = self.project_trajectory(images, all_traj, all_vals)
        return good_traj[:, 0], all_traj, all_vals, mask

    def step_nogoal(self, images, depths):
        return self._run_step(images, depths, self.navi_former.predict_nogoal_action)

    def step_pointgoal(self, goals, images, depths):
        return self._run_step(
            images, depths, self.navi_former.predict_pointgoal_action,
            self._prep.clip_point_goals(goals),
        )

    def step_imagegoal(self, goals, images, depths):
        return self._run_step(
            images, depths, self.navi_former.predict_imagegoal_action,
            self._prep.process_images(goals),
        )

    def step_pixelgoal(self, goals, images, depths):
        return self._run_step(
            images, depths, self.navi_former.predict_pixelgoal_action,
            self._prep.process_pixel_goals(goals, images),
        )

    def step_point_image_goal(self, pointgoal, imagegoal, images, depths):
        return self._run_step(
            images, depths, self.navi_former.predict_ip_action,
            self._prep.clip_point_goals(pointgoal),
            self._prep.process_images(imagegoal),
        )
