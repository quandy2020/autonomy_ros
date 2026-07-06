# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Single kinematic humanoid articulated object (TrackVLA KinematicHumanoid subset)."""

from __future__ import annotations

import math
import pickle as pkl
from typing import Any

import magnum as mn
import numpy as np

import habitat_sim
from habitat.humanoid.walk_controller import HumanoidWalkController


class HumanoidAvatar:
    """Loads a URDF humanoid AO and drives it with SMPL-X walk replay."""

    _BASE_OFFSET = mn.Vector3(0.0, -0.9, 0.0)
    _OFFSET_ROT = -math.pi / 2

    def __init__(
        self,
        sim: habitat_sim.Simulator,
        urdf_path: str,
        motion_path: str,
        semantic_id: int,
        logger: Any,
    ) -> None:
        self._sim = sim
        self._urdf_path = urdf_path
        self._motion_path = motion_path
        self._semantic_id = semantic_id
        self._logger = logger
        self._sim_obj = None
        self._offset_transform = mn.Matrix4()
        add_rot = mn.Matrix4.rotation(
            mn.Rad(self._OFFSET_ROT), mn.Vector3(0.0, 1.0, 0.0),
        )
        perm = mn.Matrix4.rotation(
            mn.Rad(self._OFFSET_ROT), mn.Vector3(0.0, 0.0, 1.0),
        )
        self._offset_transform_base = perm @ add_rot
        self._rest_joints: list[float] | None = None
        self._controller = HumanoidWalkController(motion_path)
        self._load_rest_pose()

    def _load_rest_pose(self) -> None:
        with open(self._motion_path, 'rb') as f:
            rest_pose = pkl.load(f)['stop_pose']
        self._rest_joints = list(rest_pose['joints'].reshape(-1))

    @property
    def sim_obj(self):
        return self._sim_obj

    def spawn(self) -> None:
        if self._sim_obj is not None and self._sim_obj.is_alive:
            return
        ao_mgr = self._sim.get_articulated_object_manager()
        self._sim_obj = ao_mgr.add_articulated_object_from_urdf(
            self._urdf_path,
            fixed_base=False,
            maintain_link_order=True,
        )
        self._sim_obj.motion_type = habitat_sim.physics.MotionType.KINEMATIC
        for motor_id in list(self._sim_obj.existing_joint_motor_ids):
            self._sim_obj.remove_joint_motor(motor_id)
        for node in self._sim_obj.visual_scene_nodes:
            node.semantic_id = self._semantic_id
        self._offset_transform = mn.Matrix4()
        self.set_rest_position()
        self._sim_obj.awake = True

    def remove(self) -> None:
        if self._sim_obj is None:
            return
        ao_mgr = self._sim.get_articulated_object_manager()
        if self._sim_obj.is_alive:
            ao_mgr.remove_object_by_handle(self._sim_obj.handle)
        self._sim_obj = None

    @property
    def _inverse_offset_transform(self) -> mn.Matrix4:
        rot = self._offset_transform.rotation().transposed()
        translation = -rot * self._offset_transform.translation
        return mn.Matrix4.from_(rot, translation)

    @property
    def base_transformation(self) -> mn.Matrix4:
        return (
            self._sim_obj.transformation
            @ self._inverse_offset_transform
            @ self._offset_transform_base
        )

    def set_joint_transform(
        self,
        joint_list: list[float],
        offset_transform: mn.Matrix4,
        base_transform: mn.Matrix4,
    ) -> None:
        self._sim_obj.joint_positions = joint_list
        self._offset_transform = offset_transform
        add_rot = self._offset_transform_base.inverted()
        self._sim_obj.transformation = (base_transform @ add_rot) @ offset_transform
        self._sim_obj.awake = True

    def set_rest_position(self) -> None:
        joints = self._rest_joints or list(self._sim_obj.joint_positions)
        self.set_joint_transform(joints, mn.Matrix4(), self.base_transformation)

    def set_base_from_habitat(self, position: mn.Vector3, yaw_rad: float) -> None:
        """Place humanoid ground base at a Habitat navmesh point."""
        if len(position) != 3:
            raise ValueError('position must be 3D')
        base_transform = self.base_transformation
        base_pos = position - self._BASE_OFFSET
        base_transform.translation = base_pos
        add_rot = self._offset_transform_base.inverted()
        self._sim_obj.transformation = base_transform @ add_rot @ self._offset_transform
        angle_rot = -self._OFFSET_ROT
        self._sim_obj.rotation = mn.Quaternion.rotation(
            mn.Rad(yaw_rad + angle_rot), mn.Vector3(0.0, 1.0, 0.0),
        )

    def reset_controller(self) -> None:
        self._controller.reset(self.base_transformation)

    def sync_walk(
        self,
        habitat_pos: np.ndarray,
        rel_hab_xz: np.ndarray,
        yaw_rad: float,
        moving: bool,
    ) -> None:
        """Replay walk mocap; pedestrian logic owns world translation."""
        self.set_base_from_habitat(mn.Vector3(habitat_pos), yaw_rad)
        self._controller.obj_transform_base = self.base_transformation
        if moving and float(np.linalg.norm(rel_hab_xz)) > 1e-3:
            # distance_multiplier=0: animate joints without controller base drift.
            self._controller.calculate_walk_pose_directional(
                mn.Vector3(float(rel_hab_xz[0]), 0.0, float(rel_hab_xz[1])),
                distance_multiplier=0.0,
            )
        else:
            self._controller.calculate_stop_pose()
        self.set_joint_transform(
            self._controller.joint_pose,
            self._controller.obj_transform_offset,
            self._controller.obj_transform_base,
        )
