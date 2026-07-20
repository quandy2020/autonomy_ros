"""Simple kinematic robot actor loaded from URDF."""

from __future__ import annotations

from typing import Any
import xml.etree.ElementTree as ET

import magnum as mn
import numpy as np

import habitat_sim
from habitat.robot.quadruped_controller import QuadrupedController, supports_quadruped_gait


class RobotAvatar:
    """Loads a URDF robot AO and drives it kinematically along navmesh paths."""

    def __init__(
        self,
        sim: habitat_sim.Simulator,
        urdf_path: str,
        semantic_id: int,
        asset_name: str,
        logger: Any,
    ) -> None:
        self._sim = sim
        self._urdf_path = urdf_path
        self._semantic_id = semantic_id
        self._asset_name = str(asset_name).strip().lower()
        self._logger = logger
        self._sim_obj = None
        self._joint_name_to_index: dict[str, int] = {}
        self._controller = (
            QuadrupedController(self._asset_name)
            if supports_quadruped_gait(self._asset_name)
            else None
        )

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
        if self._controller is not None:
            self._cache_joint_indices()
            self._apply_joint_positions(self._controller.initial_joint_positions())
        self._sim_obj.awake = True

    def remove(self) -> None:
        if self._sim_obj is None:
            return
        ao_mgr = self._sim.get_articulated_object_manager()
        if self._sim_obj.is_alive:
            ao_mgr.remove_object_by_handle(self._sim_obj.handle)
        self._sim_obj = None

    def set_base_from_habitat(self, position: np.ndarray, yaw_rad: float) -> None:
        if self._sim_obj is None:
            return
        place = np.array(position, dtype=np.float32)
        if self._controller is not None:
            place[1] += float(self._controller.base_height_offset)
        self._sim_obj.translation = mn.Vector3(place)
        self._sim_obj.rotation = mn.Quaternion.rotation(
            mn.Rad(float(yaw_rad)),
            mn.Vector3(0.0, 1.0, 0.0),
        )
        self._sim_obj.awake = True

    def joint_state_map(self) -> dict[str, float]:
        if self._controller is None or self._sim_obj is None:
            return {}
        joint_values = list(self._sim_obj.joint_positions)
        names = self._controller.joint_names
        return {
            name: float(joint_values[idx])
            for name, idx in self._joint_name_to_index.items()
            if name in names and idx < len(joint_values)
        }

    def _cache_joint_indices(self) -> None:
        if self._sim_obj is None or self._controller is None:
            return
        root = ET.parse(self._urdf_path).getroot()
        child_link_to_joint: dict[str, str] = {}
        for joint in root.findall('joint'):
            name = str(joint.attrib.get('name', '')).strip()
            child = joint.find('child')
            child_link = str(child.attrib.get('link', '')).strip() if child is not None else ''
            if name and child_link:
                child_link_to_joint[child_link] = name
        mapping: dict[str, int] = {}
        for link_id in self._sim_obj.get_link_ids():
            link_name = str(self._sim_obj.get_link_name(link_id)).strip()
            joint_name = child_link_to_joint.get(link_name)
            if not joint_name:
                continue
            mapping[joint_name] = int(self._sim_obj.get_link_joint_pos_offset(link_id))
        self._joint_name_to_index = mapping

    def _apply_joint_positions(self, positions: list[float]) -> None:
        if self._sim_obj is None or self._controller is None:
            return
        joint_values = list(self._sim_obj.joint_positions)
        for idx, joint_name in enumerate(self._controller.joint_names):
            if idx >= len(positions):
                break
            pos_idx = self._joint_name_to_index.get(joint_name)
            if pos_idx is None or pos_idx >= len(joint_values):
                continue
            joint_values[pos_idx] = float(positions[idx])
        self._sim_obj.joint_positions = joint_values

    def sync_drive(
        self,
        habitat_pos: np.ndarray,
        rel_hab_xz: np.ndarray,
        yaw_rad: float,
        moving: bool,
    ) -> None:
        self.set_base_from_habitat(habitat_pos, yaw_rad)
        if self._controller is not None and self._sim_obj is not None:
            self._apply_joint_positions(self._controller.step(rel_hab_xz, moving))
            self._sim_obj.awake = True
