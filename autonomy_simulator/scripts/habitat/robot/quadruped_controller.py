"""Procedural quadruped gait controller for official Habitat robots."""

from __future__ import annotations

import math


SPOT_JOINT_ORDER = [
    'fl.hx', 'fl.hy', 'fl.kn',
    'fr.hx', 'fr.hy', 'fr.kn',
    'hl.hx', 'hl.hy', 'hl.kn',
    'hr.hx', 'hr.hy', 'hr.kn',
    'arm0.sh0', 'arm0.sh1', 'arm0.hr0', 'arm0.el0', 'arm0.el1', 'arm0.wr0', 'arm0.wr1', 'arm0.f1x',
]

SPOT_STAND_POSE = {
    'fl.hx': 0.0,
    'fl.hy': 0.7,
    'fl.kn': -1.5,
    'fr.hx': 0.0,
    'fr.hy': 0.7,
    'fr.kn': -1.5,
    'hl.hx': 0.0,
    'hl.hy': 0.7,
    'hl.kn': -1.5,
    'hr.hx': 0.0,
    'hr.hy': 0.7,
    'hr.kn': -1.5,
    'arm0.sh0': 0.0,
    'arm0.sh1': -3.14,
    'arm0.hr0': 0.0,
    'arm0.el0': 3.0,
    'arm0.el1': 0.0,
    'arm0.wr0': 0.0,
    'arm0.wr1': 0.0,
    'arm0.f1x': -1.56,
}

SPOT_BASE_HEIGHT_OFFSET = 0.40


def supports_quadruped_gait(asset_name: str) -> bool:
    return str(asset_name).strip().lower() == 'spot'


class QuadrupedController:
    """Simple articulated gait generator for quadruped robot meshes."""

    def __init__(self, asset_name: str) -> None:
        self._asset_name = str(asset_name).strip().lower()
        self._phase = 0.0
        self._joint_names = list(SPOT_JOINT_ORDER) if self._asset_name == 'spot' else []
        self._rest_pose = dict(SPOT_STAND_POSE) if self._asset_name == 'spot' else {}

    @property
    def base_height_offset(self) -> float:
        if self._asset_name == 'spot':
            return SPOT_BASE_HEIGHT_OFFSET
        return 0.0

    def current_joint_map(self) -> dict[str, float]:
        return dict(self._rest_pose)

    @property
    def joint_names(self) -> list[str]:
        return list(self._joint_names)

    def initial_joint_positions(self) -> list[float]:
        return [float(self._rest_pose.get(name, 0.0)) for name in self._joint_names]

    def step(self, rel_hab_xz, moving: bool) -> list[float]:
        if self._asset_name != 'spot':
            return self.initial_joint_positions()
        vx = float(rel_hab_xz[0]) if len(rel_hab_xz) > 0 else 0.0
        vz = float(rel_hab_xz[1]) if len(rel_hab_xz) > 1 else 0.0
        speed = math.hypot(vx, vz)
        if moving and speed > 1e-4:
            self._phase = (self._phase + min(0.42, max(0.12, speed * 0.45))) % (2.0 * math.pi)
            return self._spot_trot_pose(speed)
        return self.initial_joint_positions()

    def _spot_trot_pose(self, speed: float) -> list[float]:
        stride = min(0.42, 0.20 + 0.28 * speed)
        knee_amp = min(0.65, 0.30 + 0.30 * speed)
        roll_amp = min(0.18, 0.05 + 0.08 * speed)
        phases = {
            'fl': self._phase,
            'hr': self._phase,
            'fr': self._phase + math.pi,
            'hl': self._phase + math.pi,
        }
        pose = dict(self._rest_pose)
        for leg, leg_phase in phases.items():
            s = math.sin(leg_phase)
            c = math.cos(leg_phase)
            pose[f'{leg}.hx'] = (roll_amp if leg.endswith('l') else -roll_amp) * s * 0.35
            pose[f'{leg}.hy'] = float(self._rest_pose[f'{leg}.hy']) + stride * s
            lift = max(0.0, c)
            pose[f'{leg}.kn'] = float(self._rest_pose[f'{leg}.kn']) - knee_amp * lift
        return [float(pose.get(name, 0.0)) for name in self._joint_names]
