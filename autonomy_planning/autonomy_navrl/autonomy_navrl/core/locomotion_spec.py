"""Locomotion layer configuration (high-level cmd -> joint / root actuation)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class LocomotionSpec:
    """How high-level velocity commands reach the robot articulation."""

    backend: str = 'root_velocity'
    decimation: int = 4
    policy_path: str = ''
    scale: tuple[float, float, float] = (1.0, 1.0, 1.0)
    clip_vx: float = 0.65
    clip_vy: float = 0.65
    clip_w: float = 1.2
    enable_clip: bool = True
    joint_pos_action_scale: float = 0.25
    joint_vel_action_scale: float = 5.0
    hip_joint_pos_scale: float = 0.125
    hold_stance: bool = False
    # S10 kinematic gait (``s10_gait``)
    gait_mode: str = 'auto'  # auto | stand | march | walk | strafe
    gait_step_freq_hz: float = 1.5
    gait_hipy_amp: float = 0.14
    gait_knee_amp: float = 0.22
    gait_hipx_amp: float = 0.10
    gait_march_when_idle: bool = False
    gait_precision_speed_mps: float = 0.07

    @classmethod
    def from_dict(cls, raw: dict[str, Any] | None) -> LocomotionSpec:
        if not raw:
            return cls()
        scale_raw = raw.get('scale', [1.0, 1.0, 1.0])
        if isinstance(scale_raw, (int, float)):
            scale = (float(scale_raw), float(scale_raw), float(scale_raw))
        else:
            scale = tuple(float(v) for v in list(scale_raw)[:3])
            while len(scale) < 3:
                scale = scale + (1.0,)
        clip = raw.get('clip') or {}
        gait = raw.get('gait') or {}
        return cls(
            backend=str(raw.get('backend', 'root_velocity')),
            decimation=int(raw.get('decimation', 4)),
            policy_path=str(raw.get('policy_path', '')),
            scale=(scale[0], scale[1], scale[2]),
            clip_vx=float(clip.get('max_vx', raw.get('clip_vx', 0.65))),
            clip_vy=float(clip.get('max_vy', raw.get('clip_vy', 0.65))),
            clip_w=float(clip.get('max_w', raw.get('clip_w', 1.2))),
            enable_clip=bool(raw.get('enable_clip', True)),
            joint_pos_action_scale=float(raw.get('joint_pos_action_scale', 0.25)),
            joint_vel_action_scale=float(raw.get('joint_vel_action_scale', 5.0)),
            hip_joint_pos_scale=float(raw.get('hip_joint_pos_scale', 0.125)),
            hold_stance=bool(raw.get('hold_stance', False)),
            gait_mode=str(gait.get('mode', raw.get('gait_mode', 'auto'))),
            gait_step_freq_hz=float(gait.get('step_freq_hz', raw.get('gait_step_freq_hz', 1.5))),
            gait_hipy_amp=float(gait.get('hipy_amp', raw.get('gait_hipy_amp', 0.14))),
            gait_knee_amp=float(gait.get('knee_amp', raw.get('gait_knee_amp', 0.22))),
            gait_hipx_amp=float(gait.get('hipx_amp', raw.get('gait_hipx_amp', 0.10))),
            gait_march_when_idle=bool(
                gait.get('march_when_idle', raw.get('gait_march_when_idle', False))
            ),
            gait_precision_speed_mps=float(
                gait.get('precision_speed_mps', raw.get('gait_precision_speed_mps', 0.07))
            ),
        )
