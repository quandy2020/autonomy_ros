# Copyright (c) Meta Platforms, Inc. and its affiliates.
# MIT License — vendored from TrackVLA/habitat-lab (humanoid_base_controller.py).

from __future__ import annotations

import magnum as mn
import numpy as np


class Pose:
    def __init__(self, joints_quat, root_transform):
        self.joints = list(joints_quat)
        self.root_transform = root_transform


class Motion:
    def __init__(self, joints_quat_array, transform_array, displacement, fps):
        num_poses = joints_quat_array.shape[0]
        self.num_poses = num_poses
        poses = []
        for index in range(num_poses):
            pose = Pose(
                joints_quat_array[index].reshape(-1),
                mn.Matrix4(transform_array[index]),
            )
            poses.append(pose)
        self.poses = poses
        self.fps = fps
        self.displacement = displacement


class HumanoidBaseController:
    def __init__(self, motion_fps=30, base_offset=(0, 0.9, 0)):
        self.base_offset = mn.Vector3(base_offset)
        self.motion_fps = motion_fps
        self.obj_transform_offset = mn.Matrix4()
        self.obj_transform_base = mn.Matrix4()
        self.joint_pose: list[float] = []
        self.prev_orientation = None

    def reset(self, base_transformation: mn.Matrix4) -> None:
        self.obj_transform_offset = mn.Matrix4()
        self.obj_transform_base = base_transformation
        self.prev_orientation = base_transformation.transform_vector(
            mn.Vector3(1.0, 0.0, 0.0),
        )

    def get_pose(self) -> list[float]:
        obj_trans_offset = np.asarray(self.obj_transform_offset.transposed()).flatten()
        obj_trans_base = np.asarray(self.obj_transform_base.transposed()).flatten()
        return self.joint_pose + list(obj_trans_offset) + list(obj_trans_base)
