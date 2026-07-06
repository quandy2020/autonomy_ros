# Copyright (c) Meta Platforms, Inc. and its affiliates.
# MIT License — vendored from TrackVLA/habitat-lab (humanoid_rearrange_controller.py).

from __future__ import annotations

import os
import pickle as pkl

import magnum as mn
import numpy as np

from habitat.humanoid.mocap import HumanoidBaseController, Motion, Pose

MIN_ANGLE_TURN = 30
TURNING_STEP_AMOUNT = 20
THRESHOLD_ROTATE_NOT_MOVE = 100
DIST_TO_STOP = 1e-9


class HumanoidWalkController(HumanoidBaseController):
    """SMPL-X walk replay for kinematic humanoid meshes (TrackVLA / evt_bench)."""

    def __init__(self, walk_pose_path: str, motion_fps: int = 30, base_offset=(0, 0.9, 0)):
        self.obj_transform_base = mn.Matrix4()
        super().__init__(motion_fps, base_offset)
        self.min_angle_turn = MIN_ANGLE_TURN
        self.turning_step_amount = TURNING_STEP_AMOUNT
        self.threshold_rotate_not_move = THRESHOLD_ROTATE_NOT_MOVE

        if not os.path.isfile(walk_pose_path):
            raise FileNotFoundError(f'Humanoid motion data not found: {walk_pose_path}')

        with open(walk_pose_path, 'rb') as f:
            walk_data = pkl.load(f)
        walk_info = walk_data['walk_motion']
        self.walk_motion = Motion(
            walk_info['joints_array'],
            walk_info['transform_array'],
            walk_info['displacement'],
            walk_info['fps'],
        )
        self.stop_pose = Pose(
            walk_data['stop_pose']['joints'].reshape(-1),
            mn.Matrix4(walk_data['stop_pose']['transform']),
        )
        self.prev_orientation = None
        self.walk_mocap_frame = 0
        self.dist_per_step_size = (
            self.walk_motion.displacement[-1] / self.walk_motion.num_poses
        )

    def calculate_stop_pose(self) -> None:
        self.joint_pose = self.stop_pose.joints

    def calculate_turn_pose(self, target_position: mn.Vector3) -> None:
        self.calculate_walk_pose(target_position, distance_multiplier=0.0)

    def calculate_walk_pose(
        self,
        target_position: mn.Vector3,
        distance_multiplier: float = 1.0,
    ) -> None:
        deg_per_rads = 180.0 / np.pi
        forward_V = target_position
        if forward_V.length() < DIST_TO_STOP or np.isnan(target_position).any():
            self.calculate_stop_pose()
            return

        distance_to_walk = float(np.linalg.norm(forward_V))
        did_rotate = False
        forward_angle = 0.0

        new_angle = np.arctan2(forward_V[0], forward_V[2]) * deg_per_rads
        if self.prev_orientation is not None:
            prev_orientation = self.prev_orientation
            prev_angle = (
                np.arctan2(prev_orientation[0], prev_orientation[2]) * deg_per_rads
            )
            forward_angle = new_angle - prev_angle
            forward_angle = (forward_angle + 180) % 360 - 180

            if abs(forward_angle) > self.min_angle_turn:
                actual_angle_move = self.turning_step_amount
                if abs(forward_angle) < actual_angle_move:
                    actual_angle_move = abs(forward_angle)
                new_angle = prev_angle + actual_angle_move * np.sign(forward_angle)
                new_angle /= deg_per_rads
                did_rotate = True
            else:
                new_angle = new_angle / deg_per_rads

            forward_V = mn.Vector3(np.sin(new_angle), 0.0, np.cos(new_angle))

        forward_V = mn.Vector3(forward_V).normalized()
        self.prev_orientation = forward_V

        step_size = int(self.walk_motion.fps / self.motion_fps)
        if did_rotate:
            distance_to_walk = self.dist_per_step_size * 2
            if abs(forward_angle) >= self.threshold_rotate_not_move:
                distance_to_walk = 0.0

        step_size = max(
            1, min(step_size, int(distance_to_walk / self.dist_per_step_size)),
        )
        cosmetic_only = distance_multiplier == 0.0
        if cosmetic_only:
            step_size = max(step_size, 1)

        prev_mocap_frame = self.walk_mocap_frame
        self.walk_mocap_frame = (self.walk_mocap_frame + step_size) % self.walk_motion.num_poses

        prev_cum = self.walk_motion.displacement[prev_mocap_frame]
        new_cum = self.walk_motion.displacement[self.walk_mocap_frame]
        offset = self.walk_motion.displacement[-1] if self.walk_mocap_frame < prev_mocap_frame else 0
        if cosmetic_only:
            dist_diff = 0.0
        else:
            dist_diff = min(
                distance_to_walk,
                max(0.0, new_cum + offset - prev_cum),
            )

        new_pose = self.walk_motion.poses[self.walk_mocap_frame]
        joint_pose, obj_transform = new_pose.joints, new_pose.root_transform

        forward_V_norm = mn.Vector3(forward_V[2], forward_V[1], -forward_V[0])
        look_at_path_T = mn.Matrix4.look_at(
            self.obj_transform_base.translation,
            self.obj_transform_base.translation + forward_V_norm.normalized(),
            mn.Vector3.y_axis(),
        )
        add_rot = mn.Matrix4.rotation(mn.Rad(np.pi), mn.Vector3(0.0, 1.0, 0.0))
        obj_transform = add_rot @ obj_transform
        obj_transform.translation *= mn.Vector3.x_axis() + mn.Vector3.y_axis()
        self.obj_transform_offset = obj_transform

        obj_transform_base = look_at_path_T
        obj_transform_base.translation += forward_V * dist_diff * distance_multiplier
        rot_offset = mn.Matrix4.rotation(mn.Rad(-np.pi / 2), mn.Vector3(1, 0, 0))
        self.obj_transform_base = obj_transform_base @ rot_offset
        self.joint_pose = joint_pose

    def calculate_walk_pose_directional(
        self,
        target_position: mn.Vector3,
        distance_multiplier: float = 1.0,
        target_dir=None,
    ) -> None:
        deg_per_rads = 180.0 / np.pi
        epsilon = 1e-5
        forward_V = target_position
        if forward_V.length() < epsilon or np.isnan(target_position).any():
            self.calculate_stop_pose()
            return

        distance_to_walk = float(np.linalg.norm(forward_V))
        did_rotate = False

        forward_V_orientation = forward_V
        if target_dir is not None:
            new_angle = np.arctan2(target_dir[2], target_dir[0]) * deg_per_rads
            new_angle = (new_angle + 180) % 360 - 180
            new_angle_walk = np.arctan2(forward_V[2], forward_V[0]) * deg_per_rads
        else:
            new_angle = np.arctan2(forward_V[2], forward_V[0]) * deg_per_rads
            new_angle_walk = new_angle

        if self.prev_orientation is not None:
            prev_orientation = self.prev_orientation
            prev_angle = (
                np.arctan2(prev_orientation[2], prev_orientation[0]) * deg_per_rads
            )
            forward_angle = new_angle - prev_angle
            if forward_angle >= 180:
                forward_angle -= 360
            if forward_angle <= -180:
                forward_angle += 360

            if abs(forward_angle) > self.min_angle_turn:
                if target_dir is None:
                    actual_angle_move = self.turning_step_amount
                else:
                    actual_angle_move = self.turning_step_amount * 20
                if abs(forward_angle) < actual_angle_move:
                    actual_angle_move = abs(forward_angle)
                new_angle = prev_angle + actual_angle_move * np.sign(forward_angle)
                new_angle /= deg_per_rads
                did_rotate = True
                new_angle_walk = new_angle
            else:
                new_angle /= deg_per_rads
                new_angle_walk /= deg_per_rads

            forward_V = mn.Vector3(
                np.cos(new_angle_walk), 0.0, np.sin(new_angle_walk),
            )
            forward_V_orientation = mn.Vector3(
                np.cos(new_angle), 0.0, np.sin(new_angle),
            )

        forward_V = mn.Vector3(forward_V).normalized()
        self.prev_orientation = forward_V_orientation

        step_size = int(self.walk_motion.fps / 30.0)
        if did_rotate:
            distance_to_walk = 0.05

        step_size = max(
            1, min(step_size, int(distance_to_walk / self.dist_per_step_size)),
        )
        cosmetic_only = distance_multiplier == 0.0
        if cosmetic_only:
            step_size = max(step_size, 1)

        prev_mocap_frame = self.walk_mocap_frame
        self.walk_mocap_frame = (self.walk_mocap_frame + step_size) % self.walk_motion.num_poses

        prev_cum = self.walk_motion.displacement[prev_mocap_frame]
        new_cum = self.walk_motion.displacement[self.walk_mocap_frame]
        offset = self.walk_motion.displacement[-1] if self.walk_mocap_frame < prev_mocap_frame else 0
        if cosmetic_only:
            dist_diff = 0.0
        else:
            dist_diff = min(
                distance_to_walk,
                max(0.0, new_cum + offset - prev_cum),
            )

        new_pose = self.walk_motion.poses[self.walk_mocap_frame]
        joint_pose, obj_transform = new_pose.joints, new_pose.root_transform

        forward_V_norm = mn.Vector3(
            forward_V_orientation[2],
            forward_V_orientation[1],
            -forward_V_orientation[0],
        )
        look_at_path_T = mn.Matrix4.look_at(
            self.obj_transform_base.translation,
            self.obj_transform_base.translation + forward_V_norm.normalized(),
            mn.Vector3.y_axis(),
        )
        add_rot = mn.Matrix4.rotation(mn.Rad(np.pi), mn.Vector3(0.0, 1.0, 0.0))
        obj_transform = add_rot @ obj_transform
        obj_transform.translation *= mn.Vector3.x_axis() + mn.Vector3.y_axis()
        self.obj_transform_offset = obj_transform

        obj_transform_base = look_at_path_T
        obj_transform_base.translation += forward_V * dist_diff * distance_multiplier
        rot_offset = mn.Matrix4.rotation(mn.Rad(-np.pi / 2), mn.Vector3(1, 0, 0))
        self.obj_transform_base = obj_transform_base @ rot_offset
        self.joint_pose = joint_pose
