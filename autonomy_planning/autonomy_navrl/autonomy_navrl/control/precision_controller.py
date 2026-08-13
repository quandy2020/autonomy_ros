"""Proportional pose-tracking controller for precision alignment demos."""

from __future__ import annotations

import math

import numpy as np


def wrap_angle(angle: float) -> float:
    return float(np.arctan2(np.sin(angle), np.cos(angle)))


def is_fully_aligned(
    distance: float,
    yaw_error: float,
    speed: float,
    *,
    pos_tol_m: float,
    yaw_tol_rad: float,
    speed_tol_mps: float,
) -> bool:
    """Return True when pose and speed are within parking tolerances."""
    return (
        distance <= pos_tol_m
        and abs(yaw_error) <= yaw_tol_rad
        and speed <= speed_tol_mps
    )


def _apply_min_speed(cmd: float, error: float, deadband: float, min_speed: float) -> float:
    """Keep a minimum signed speed when error exceeds deadband."""
    if abs(error) <= deadband:
        return 0.0 if abs(cmd) < deadband else cmd
    if abs(cmd) < min_speed:
        return math.copysign(min_speed, error if abs(error) > 1e-6 else cmd)
    return cmd


def compute_pose_tracking_action(
    dx_body: float,
    dy_body: float,
    distance: float,
    bearing: float,
    yaw_error: float,
    *,
    max_vx: float,
    max_vy: float,
    max_w: float,
    kp_lin: float = 1.8,
    kp_lat: float = 1.2,
    kp_yaw: float = 2.4,
    slowdown_dist_m: float = 1.2,
    parking_zone_m: float = 0.35,
    parking_yaw_gain: float = 6.0,
    parking_min_lin_mps: float = 0.06,
    parking_speed_cap_mps: float = 0.18,
    yaw_priority_rad: float = 0.12,
    yaw_lock_dist_m: float = 0.12,
    deadband_pos_m: float = 0.015,
    deadband_yaw_rad: float = 0.025,
    align_bearing_rad: float = 0.40,
    min_turn_frac: float = 0.65,
    min_approach_frac: float = 0.50,
    turn_assist_vx_frac: float = 0.28,
) -> np.ndarray:
    """Map body-frame pose error to normalized (vx, vy, w) in [-1, 1].

    S10 with locked-leg wheel drive is effectively non-holonomic: long-range
    strafing and pure spin are weak. Strategy:

    1. Far + off-heading — arc-turn (forward roll + yaw) until facing the goal.
    2. Far + facing — drive forward with light lateral trim.
    3. Near — holonomic parking creep + yaw trim.
    """
    distance = max(float(distance), 0.0)
    bearing = wrap_angle(float(bearing))
    yaw_error = wrap_angle(float(yaw_error))
    dx_body = float(dx_body)
    dy_body = float(dy_body)
    abs_bearing = abs(bearing)

    if distance <= parking_zone_m:
        creep = min(
            1.0,
            max(0.0, (distance - yaw_lock_dist_m) / max(parking_zone_m - yaw_lock_dist_m, 1e-3)),
        )
        yaw_lock = abs(yaw_error) > yaw_priority_rad and distance <= yaw_lock_dist_m

        if yaw_lock:
            vx_cmd = turn_assist_vx_frac * max_vx * 0.5
            vy_cmd = 0.0
            w_cmd = parking_yaw_gain * yaw_error * 1.4
        elif abs_bearing > align_bearing_rad:
            vx_cmd = turn_assist_vx_frac * max_vx
            vy_cmd = 0.0
            w_cmd = parking_yaw_gain * bearing
            if abs(w_cmd) < min_turn_frac * max_w:
                w_cmd = math.copysign(min_turn_frac * max_w, bearing)
        else:
            vx_cmd = kp_lin * dx_body
            vy_cmd = kp_lat * dy_body
            vx_cmd = _apply_min_speed(vx_cmd, dx_body, deadband_pos_m, parking_min_lin_mps)
            vy_cmd = _apply_min_speed(vy_cmd, dy_body, deadband_pos_m, parking_min_lin_mps)
            w_cmd = parking_yaw_gain * yaw_error + kp_yaw * bearing * 0.35
            if abs(yaw_error) <= deadband_yaw_rad and distance <= deadband_pos_m * 2:
                w_cmd *= 0.25

        cap = parking_speed_cap_mps * max(creep, 0.35)
        vx_cmd = float(np.clip(vx_cmd, -cap, cap))
        vy_cmd = float(np.clip(vy_cmd, -cap, cap))
    elif abs_bearing > align_bearing_rad:
        # Arc-turn toward the goal — never long-range crab-walk.
        vx_cmd = turn_assist_vx_frac * max_vx
        # Back up slightly if goal is behind (>90°).
        if abs_bearing > (0.5 * math.pi):
            vx_cmd = -0.6 * turn_assist_vx_frac * max_vx
        vy_cmd = 0.0
        w_cmd = kp_yaw * bearing * 2.0
        if abs(w_cmd) < min_turn_frac * max_w:
            w_cmd = math.copysign(min_turn_frac * max_w, bearing)
    else:
        speed_scale = min(1.0, max(0.40, distance / max(slowdown_dist_m, 1e-3)))
        vx_cmd = kp_lin * dx_body * speed_scale
        vy_cmd = kp_lat * dy_body * speed_scale * 0.25
        if dx_body > deadband_pos_m:
            vx_cmd = max(vx_cmd, min_approach_frac * max_vx * speed_scale)
        elif dx_body < -deadband_pos_m:
            vx_cmd = min(vx_cmd, -min_approach_frac * max_vx * speed_scale)
        w_cmd = 1.0 * kp_yaw * bearing + 0.4 * kp_yaw * yaw_error

    vx = np.clip(vx_cmd, -max_vx, max_vx) / max(max_vx, 1e-6)
    vy = np.clip(vy_cmd, -max_vy, max_vy) / max(max_vy, 1e-6)
    w = np.clip(w_cmd, -max_w, max_w) / max(max_w, 1e-6)
    return np.array([vx, vy, w], dtype=np.float32)
