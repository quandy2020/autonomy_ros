"""Online dock pose refinement from LEFT / CENTER / RIGHT IR regions."""

from __future__ import annotations

import math

from autocharge.common.geometry import wrap_pi
from autocharge.docking.type import DockingConfig
from autocharge.docking.utils import norm_token


def _token_side_score(token: int, cfg: DockingConfig) -> float:
    """Signed lateral score from one receiver token (+ = left region)."""
    bits = int(norm_token(token, cfg.token_mask))
    if bits == 0:
        return 0.0
    score = 0.0
    if bits & int(cfg.bit_l1):
        score += 2.0
    if bits & int(cfg.bit_l2):
        score += 1.0
    if bits & int(cfg.bit_r2):
        score -= 1.0
    if bits & int(cfg.bit_r1):
        score -= 2.0
    if bits & int(cfg.bit_c):
        score *= 0.15
    return score


def ir_region_balance(
    left_token: int,
    right_token: int,
    cfg: DockingConfig,
) -> tuple[float, float, bool]:
    """IR lateral / yaw balance in [-1, 1] from LEFT, CENTER, RIGHT zones.

    Returns (lateral_balance, yaw_balance, has_ir).
    +lateral: seeing left-side beams → robot is left of true center axis.
    """
    l = int(norm_token(left_token, cfg.token_mask))
    r = int(norm_token(right_token, cfg.token_mask))
    bits = l | r
    if bits == 0:
        return 0.0, 0.0, False

    l_score = _token_side_score(l, cfg)
    r_score = _token_side_score(r, cfg)
    combined = l_score + r_score

    # Strong center evidence: anchor estimate on the axis.
    if (bits & int(cfg.bit_c)) != 0:
        combined *= 0.12

    lateral = max(-1.0, min(1.0, combined / 3.0))
    asym = max(-1.0, min(1.0, (l_score - r_score) / 3.0))
    yaw = max(-1.0, min(1.0, 0.65 * lateral + 0.35 * asym))
    return lateral, yaw, True


class DockPoseRefiner:
    """Low-pass IR correction on nominal dock pose (y lateral + yaw)."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._dy_dock = 0.0
        self._dyaw = 0.0

    def refine(
        self,
        nominal_x: float,
        nominal_y: float,
        nominal_yaw: float,
        left_token: int,
        right_token: int,
        cfg: DockingConfig,
        dist_m: float | None,
        dt: float,
    ) -> tuple[float, float, float]:
        """Return (x, y, yaw) with IR-based lateral/yaw correction applied."""
        lat, yaw_bal, has_ir = ir_region_balance(left_token, right_token, cfg)
        alpha = float(cfg.dock_ir_refine_alpha)
        alpha = max(0.0, min(0.99, alpha))

        if has_ir:
            dist = 0.45 if dist_m is None else max(0.05, float(dist_m))
            # Farther out: allow more lateral shift; near dock: smaller shift.
            dist_scale = max(0.25, min(1.0, dist / 0.45))
            dy_target = float(cfg.dock_ir_refine_y_m) * lat * dist_scale
            bits = int(norm_token(left_token, cfg.token_mask)) | int(
                norm_token(right_token, cfg.token_mask)
            )
            inner = bits & (int(cfg.bit_l2) | int(cfg.bit_r2) | int(cfg.bit_c))
            dyaw_target = 0.0
            if inner != 0:
                dyaw_target = float(cfg.dock_ir_refine_yaw_rad) * yaw_bal
            blend = max(0.15, min(1.0, float(dt) / 0.05))
            self._dy_dock = alpha * self._dy_dock + (1.0 - alpha) * dy_target * blend
            if inner != 0:
                self._dyaw = alpha * self._dyaw + (1.0 - alpha) * dyaw_target * blend
        else:
            # Dark: decay correction slowly (do not snap back).
            decay = max(0.0, 1.0 - 0.20 * max(0.0, float(dt)))
            self._dy_dock *= decay
            self._dyaw *= decay

        yaw0 = float(nominal_yaw)
        c = math.cos(yaw0)
        s = math.sin(yaw0)
        dy = float(self._dy_dock)
        dx = -s * dy
        dy_w = c * dy
        return (
            float(nominal_x) + dx,
            float(nominal_y) + dy_w,
            wrap_pi(yaw0 + float(self._dyaw)),
        )
