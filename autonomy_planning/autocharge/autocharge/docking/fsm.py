"""Strategy-1 docking FSM: SEARCH → BIAS_* → CENTER → SUCCESS.

See doc/docking_strategy.md. Control uses odom + IR only (no δ / red-arc).
"""

from __future__ import annotations

import math
from typing import Optional

from autocharge.common.geometry import wrap_pi
from autocharge.docking.type import (
    CENTER_ZZ_HALF_PERIOD_S,
    ZZ_DARK_FRAMES,
    ZZ_DARK_FRAMES_FAR_FIRST,
    ZZ_FORWARD_M,
    ZZ_FORWARD_M_FAR,
    ZZ_MAX_SCAN_RAD,
    BiasZigzagPhase,
    DockingConfig,
    DockObservation,
    DockState,
    RelativeState,
    TwistCmd,
)
from autocharge.docking.utils import (
    classify_ir_region,
    norm_token,
    region_side_priority,
    region_to_bias_side,
)


class DockingFSM:
    """IR region state machine for strategy-1 autocharge docking."""

    def __init__(self, cfg: DockingConfig | None = None) -> None:
        self.cfg = cfg or DockingConfig()
        self.reset()

    def reset(self) -> None:
        self.state = DockState.SEARCH
        self.state_time_s = 0.0
        self.last_dist_m: Optional[float] = None
        self.last_yaw_rad: Optional[float] = None
        self.ir_elapsed_s = 0.0
        self.l_last = 0
        self.r_last = 0
        self.smoothed_rel = RelativeState.UNKNOWN

        self.cnt_center = 0
        self.cnt_left = 0
        self.cnt_right = 0
        self.cnt_weak_center = 0
        self.no_signal_elapsed_s = 0.0
        self.center_hold_elapsed_s = 0.0
        self.last_fine_region = RelativeState.UNKNOWN

        # CENTER runtime.
        self.center_side_hint = 0
        self.center_near_dist_m = float(self.cfg.center_near_dist_m)
        self.center_near_vx = float(self.cfg.center_near_vx)
        self.center_near_wz = float(self.cfg.center_near_wz)
        self.center_very_near_dist_m = float(self.cfg.center_very_near_dist_m)
        self.center_very_near_vx = float(self.cfg.center_very_near_vx)
        self.center_very_near_wz = float(self.cfg.center_very_near_wz)
        self.center_side_reacquire_elapsed_s = 0.0
        self.center_lost_bit_elapsed_s = 0.0
        self.center_lock_yaw: Optional[float] = None
        self.center_reacq_dir = -1
        self.center_reacq_progress_rad = 0.0
        self.center_reacq_last_yaw: Optional[float] = None
        self.center_seen_strong = False
        self.center_zz_elapsed_s = 0.0
        self.center_zz_sign = 1

        self.debug_events: list[str] = []
        self.odom_x: Optional[float] = None
        self.odom_y: Optional[float] = None

        # Odom forward task (BIAS zigzag step).
        self.forward_task_target_m = 0.0
        self.forward_task_progress_m = 0.0
        self.forward_task_active = False
        self.forward_start_x: Optional[float] = None
        self.forward_start_y: Optional[float] = None

        # BIAS zigzag runtime.
        self.bias_zz_phase = BiasZigzagPhase.SCAN_PRIMARY
        self.bias_zz_dark_streak = 0
        self.bias_zz_first_scan = True
        self.bias_zz_scan_yaw0: Optional[float] = None
        self.bias_zz_scan_progress = 0.0
        self.bias_zz_last_scan_yaw: Optional[float] = None
        self.bias_zz_seen_watch = False
        self.bias_zz_seen_center = False
        self.bias_switch_confirm_samples = int(self.cfg.bias_switch_confirm_samples)
        self.bias_l2r_count = 0
        self.bias_r2l_count = 0
        self.bias_switch_lock_elapsed_s = 0.0
        self.bias_lost_signal_elapsed_s = 0.0
        self.bias_best_dist_m: Optional[float] = None

        # SEARCH full-turn scan.
        self.search_start_yaw: Optional[float] = None
        self.search_last_yaw: Optional[float] = None
        self.search_yaw_progress = 0.0
        self.search_spin_sign = -1.0
        self.search_worst_dist_m: Optional[float] = None
        self.search_dist_worsen_s = 0.0
        self.search_hits: dict[RelativeState, int] = {}
    def drain_debug_events(self) -> list[str]:
        events = list(self.debug_events)
        self.debug_events.clear()
        return events

    def clear_forward_task(self) -> None:
        self.forward_task_active = False
        self.forward_task_target_m = 0.0
        self.forward_task_progress_m = 0.0
        self.forward_start_x = None
        self.forward_start_y = None

    def enter_center(self) -> TwistCmd:
        self.reset_search_scan()
        self.reset_bias_switch_tracking()
        self.bias_lost_signal_elapsed_s = 0.0
        self.center_lost_bit_elapsed_s = 0.0
        self.center_lock_yaw = self.last_yaw_rad
        self.center_reacq_dir = -1
        self.center_reacq_progress_rad = 0.0
        self.center_reacq_last_yaw = self.last_yaw_rad
        self.center_seen_strong = False
        self.center_zz_elapsed_s = 0.0
        self.center_zz_sign = 1
        self.state = DockState.CENTER
        return TwistCmd(0.0, 0.0)

    def geometry_allows_center(self, obs: Optional[DockObservation]) -> bool:
        """Commit to CENTER only when pose is already near the dock axis."""
        if obs is None:
            return False
        lat_lim = abs(float(self.cfg.center_enter_lateral_m))
        yaw_lim = abs(float(self.cfg.center_enter_yaw_rad))
        if obs.lateral_m is not None and abs(float(obs.lateral_m)) > lat_lim:
            return False
        if obs.yaw_err_rad is not None and abs(float(obs.yaw_err_rad)) > yaw_lim:
            return False
        # If geometry is unavailable, only allow strong center pairs.
        if obs.lateral_m is None and obs.yaw_err_rad is None:
            return False
        return True

    def try_enter_center(
        self,
        rel: RelativeState,
        obs: Optional[DockObservation],
        *,
        source: str,
    ) -> Optional[TwistCmd]:
        """Enter CENTER for strong pairs, or weak center when safe.

        SEARCH: weak/single-bit center needs a geometry gate (83.log dark-spin).
        BIAS: we are already zigzagging into the beam — accept confirmed WEAK_CENTER
        immediately so we do not sweep past 0x04 (86.log never committed).
        """
        if rel == RelativeState.CENTER and self.has_center_evidence():
            self.debug_events.append(f'{source}_hit_center')
            return self.enter_center()
        if rel != RelativeState.WEAK_CENTER:
            return None
        in_bias = self.state in (
            DockState.BIAS_LEFT_FAR,
            DockState.BIAS_RIGHT_FAR,
            DockState.BIAS_LEFT,
            DockState.BIAS_RIGHT,
        )
        if in_bias or self.geometry_allows_center(obs):
            self.debug_events.append(
                f'{source}_hit_weak_center' + ('_bias' if in_bias else '_gated')
            )
            return self.enter_center()
        # Weak center without geometry outside BIAS: route via side bias.
        side = None
        if obs is not None:
            side = self.receiver_side_observation(obs.left_token, obs.right_token)
        if side is None:
            side = self.receiver_side_observation(self.l_last, self.r_last)
        if side == 'left':
            self.debug_events.append(f'{source}_weak_to_bias_left')
            return self.enter_bias_left()
        if side == 'right':
            self.debug_events.append(f'{source}_weak_to_bias_right')
            return self.enter_bias_right()
        if self.center_side_hint > 0:
            self.debug_events.append(f'{source}_weak_to_bias_left_hint')
            return self.enter_bias_left()
        if self.center_side_hint < 0:
            self.debug_events.append(f'{source}_weak_to_bias_right_hint')
            return self.enter_bias_right()
        return None

    def lateral_trim_wz(self, obs: DockObservation, wz_mag: float) -> float:
        """Yaw command that reduces front lateral error when geometry is known."""
        if obs.lateral_m is None:
            return 0.0
        lat = float(obs.lateral_m)
        if abs(lat) < 0.005:
            return 0.0
        # Positive lateral (front contact left of dock) → turn CW (negative wz)
        # to swing the nose right toward the dock axis.
        return -math.copysign(abs(wz_mag), lat)

    def tick(self, dt: float, obs: DockObservation) -> TwistCmd:
        dt = max(float(self.cfg.min_dt_s), min(float(self.cfg.max_dt_s), float(dt)))
        self.state_time_s += dt
        self.last_dist_m = obs.dist_m
        self.last_yaw_rad = obs.yaw_rad
        self.odom_x = obs.odom_x
        self.odom_y = obs.odom_y
        if obs.abort:
            self.state = DockState.ABORT
            return TwistCmd(0.0, 0.0)
        if obs.charge_connected:
            self.state = DockState.SUCCESS
            return TwistCmd(0.0, 0.0)
        if self.state in (DockState.SUCCESS, DockState.ABORT):
            return TwistCmd(0.0, 0.0)

        rel = self.sample_and_smooth(dt, obs.left_token, obs.right_token)
        # Minimal but functional state dispatch.
        if self.state == DockState.SEARCH:
            return self.handle_search(dt, rel, obs)
        if self.state == DockState.CENTER:
            return self.handle_center(dt, rel, obs)
        if self.state in (
            DockState.BIAS_LEFT_FAR,
            DockState.BIAS_RIGHT_FAR,
            DockState.BIAS_LEFT,
            DockState.BIAS_RIGHT,
        ):
            is_left = self.state in (DockState.BIAS_LEFT_FAR, DockState.BIAS_LEFT)
            is_far = self.state in (
                DockState.BIAS_LEFT_FAR,
                DockState.BIAS_RIGHT_FAR,
            )
            return self.handle_bias(dt, rel, obs, is_left, is_far)
        self.state = DockState.SEARCH
        self.reset_search_scan()
        return self.handle_search(dt, rel, obs)

    def pair_semantic(self, left_token: int, right_token: int) -> RelativeState:
        """Fine-grained IR region classification for search/bias routing."""
        return classify_ir_region(
            left_token,
            right_token,
            bit_l1=int(self.cfg.bit_l1),
            bit_l2=int(self.cfg.bit_l2),
            bit_c=int(self.cfg.bit_c),
            bit_r2=int(self.cfg.bit_r2),
            bit_r1=int(self.cfg.bit_r1),
            token_mask=int(self.cfg.token_mask),
        )

    def coarse_relative(self, region: RelativeState) -> RelativeState:
        """Collapse fine regions into LEFT/RIGHT/CENTER/UNKNOWN for legacy handlers."""
        if region in (RelativeState.CENTER, RelativeState.WEAK_CENTER):
            return RelativeState.CENTER
        side = region_to_bias_side(region)
        if side == 'left':
            return RelativeState.LEFT
        if side == 'right':
            return RelativeState.RIGHT
        return RelativeState.UNKNOWN

    def sample_and_smooth(self, dt: float, left_token: int, right_token: int) -> RelativeState:
        self.ir_elapsed_s += dt
        if self.ir_elapsed_s < max(0.05, float(self.cfg.ir_decision_period_s)):
            return self.smoothed_rel
        self.ir_elapsed_s = 0.0
        self.l_last = norm_token(left_token, self.cfg.token_mask)
        self.r_last = norm_token(right_token, self.cfg.token_mask)
        cand = self.pair_semantic(self.l_last, self.r_last)
        self.last_fine_region = cand
        coarse = self.coarse_relative(cand)

        if cand == RelativeState.CENTER:
            self.cnt_center += 1
            self.cnt_weak_center = 0
            self.cnt_left = self.cnt_right = 0
            self.no_signal_elapsed_s = 0.0
            if self.cnt_center >= max(1, int(self.cfg.state_confirm_samples)):
                self.smoothed_rel = RelativeState.CENTER
                self.center_hold_elapsed_s = float(self.cfg.center_hold_s)
                self.center_side_reacquire_elapsed_s = 0.0
            return self.smoothed_rel
        if cand == RelativeState.WEAK_CENTER:
            self.cnt_weak_center += 1
            self.cnt_center = 0
            self.cnt_left = self.cnt_right = 0
            self.no_signal_elapsed_s = 0.0
            need = max(1, int(self.cfg.weak_center_confirm_samples))
            if self.cnt_weak_center >= need:
                self.smoothed_rel = RelativeState.WEAK_CENTER
                self.center_hold_elapsed_s = float(self.cfg.center_hold_s)
                self.center_side_reacquire_elapsed_s = 0.0
            return self.smoothed_rel
        if coarse == RelativeState.LEFT:
            self.cnt_left += 1
            self.cnt_center = self.cnt_right = self.cnt_weak_center = 0
            self.no_signal_elapsed_s = 0.0
            if self.cnt_left >= max(1, int(self.cfg.state_confirm_samples)):
                self.smoothed_rel = cand
                self.center_side_hint = 1
                self.center_side_reacquire_elapsed_s = 0.0
            return self.smoothed_rel
        if coarse == RelativeState.RIGHT:
            self.cnt_right += 1
            self.cnt_center = self.cnt_left = self.cnt_weak_center = 0
            self.no_signal_elapsed_s = 0.0
            if self.cnt_right >= max(1, int(self.cfg.state_confirm_samples)):
                self.smoothed_rel = cand
                self.center_side_hint = -1
                self.center_side_reacquire_elapsed_s = 0.0
            return self.smoothed_rel

        self.no_signal_elapsed_s += float(self.cfg.ir_decision_period_s)
        if self.center_hold_elapsed_s > 0.0:
            self.center_hold_elapsed_s = max(
                0.0, self.center_hold_elapsed_s - float(self.cfg.ir_decision_period_s)
            )
            self.smoothed_rel = RelativeState.CENTER
            return self.smoothed_rel
        if self.no_signal_elapsed_s >= float(self.cfg.no_signal_search_timeout_s):
            self.smoothed_rel = RelativeState.UNKNOWN
        return self.smoothed_rel

    def has_center_evidence(self) -> bool:
        """True when either receiver currently sees the center beam bit."""
        return self.sees_center_bit(self.l_last, self.r_last)

    def sees_center_bit(self, left_token: int, right_token: int) -> bool:
        """Any receiver showing bit_c (0x04) means we are in the center beam."""
        bit_c = int(self.cfg.bit_c)
        if bit_c == 0:
            return False
        l = norm_token(left_token, self.cfg.token_mask)
        r = norm_token(right_token, self.cfg.token_mask)
        return ((l & bit_c) != 0) or ((r & bit_c) != 0)

    def receiver_side_observation(self, left_token: int, right_token: int) -> Optional[str]:
        """Returns which receiver dominates a single-sided hit."""
        l = norm_token(left_token, self.cfg.token_mask)
        r = norm_token(right_token, self.cfg.token_mask)
        if l != 0 and r == 0:
            return 'left'
        if r != 0 and l == 0:
            return 'right'
        return None

    def reset_bias_switch_tracking(self) -> None:
        """Clears cross-side switch counters for the current bias cycle."""
        self.bias_l2r_count = 0
        self.bias_r2l_count = 0
        self.bias_switch_lock_elapsed_s = 0.0

    def current_bias_phase_allows_switch(self) -> bool:
        """Side switch only after a short lock while still tracking."""
        return self.bias_switch_lock_elapsed_s >= float(self.cfg.bias_switch_lock_s)

    def reset_bias_zigzag(self) -> None:
        self.bias_zz_phase = BiasZigzagPhase.SCAN_PRIMARY
        self.bias_zz_dark_streak = 0
        self.bias_zz_first_scan = True
        self.bias_zz_scan_yaw0 = self.last_yaw_rad
        self.bias_zz_scan_progress = 0.0
        self.bias_zz_last_scan_yaw = self.last_yaw_rad
        self.bias_zz_seen_watch = False
        self.bias_zz_seen_center = False
        self.clear_forward_task()

    def _reset_bias_scan_edge(self) -> None:
        """Reset edge detectors when entering a new scan phase."""
        self.bias_zz_dark_streak = 0
        self.bias_zz_seen_watch = False
        # Keep session yaw progress / seen_center across phases.
        self.bias_zz_last_scan_yaw = self.last_yaw_rad

    def _bias_reclassify_after_full_turn(
        self, obs: Optional[DockObservation]
    ) -> TwistCmd:
        """No 0x04 after ≥1 turn: re-enter the region indicated by current IR."""
        self.debug_events.append('bias_zz_full_turn_reclassify')
        if obs is None:
            self.state = DockState.SEARCH
            self.reset_search_scan()
            return TwistCmd(0.0, 0.0)
        fine = self.pair_semantic(obs.left_token, obs.right_token)
        if fine == RelativeState.FAR_LEFT:
            return self.enter_bias_left_far()
        if fine in (RelativeState.LEFT, RelativeState.OVERLAP_LEFT):
            return self.enter_bias_left()
        if fine == RelativeState.FAR_RIGHT:
            return self.enter_bias_right_far()
        if fine in (RelativeState.RIGHT, RelativeState.OVERLAP_RIGHT):
            return self.enter_bias_right()
        if fine in (RelativeState.CENTER, RelativeState.WEAK_CENTER):
            entered = self.try_enter_center(fine, obs, source='bias_full_turn')
            if entered is not None:
                return entered
            return self.enter_center()
        self.state = DockState.SEARCH
        self.reset_search_scan()
        return TwistCmd(0.0, 0.0)

    def _enter_bias(self, state: DockState) -> TwistCmd:
        """Start BIAS zigzag for the given side/far state."""
        self.reset_search_scan()
        self.reset_bias_switch_tracking()
        self.reset_bias_zigzag()
        self.bias_lost_signal_elapsed_s = 0.0
        self.bias_best_dist_m = None
        self.state = state
        return TwistCmd(0.0, 0.0)

    def enter_bias_left(self) -> TwistCmd:
        return self._enter_bias(DockState.BIAS_LEFT)

    def enter_bias_left_far(self) -> TwistCmd:
        return self._enter_bias(DockState.BIAS_LEFT_FAR)

    def enter_bias_right(self) -> TwistCmd:
        return self._enter_bias(DockState.BIAS_RIGHT)

    def enter_bias_right_far(self) -> TwistCmd:
        return self._enter_bias(DockState.BIAS_RIGHT_FAR)

    def is_near_dock(self) -> bool:
        if self.last_dist_m is None:
            return False
        return float(self.last_dist_m) <= float(self.cfg.center_near_no_backup_dist_m)

    def center_cruise_vx(self) -> float:
        """Distance-scaled forward speed in CENTER (never negative)."""
        vx_max = float(self.cfg.center_vx)
        if self.last_dist_m is None:
            return vx_max
        dist = float(self.last_dist_m)
        if dist >= float(self.cfg.center_near_no_backup_dist_m):
            return vx_max
        d_far = float(self.cfg.center_near_no_backup_dist_m)
        d_near = self.center_very_near_dist_m
        v_near = self.center_very_near_vx
        if dist <= d_near:
            return v_near
        t = (dist - d_near) / max(1e-6, d_far - d_near)
        return v_near + t * (vx_max - v_near)

    def center_trim_wz(self) -> float:
        wz = min(abs(float(self.cfg.center_trim_wz)), 0.03)
        if self.last_dist_m is None:
            return wz
        dist = float(self.last_dist_m)
        if dist <= self.center_very_near_dist_m:
            return min(wz, self.center_very_near_wz)
        if dist <= self.center_near_dist_m:
            return min(wz, self.center_near_wz)
        return wz

    def handle_center(
        self,
        dt: float,
        rel: RelativeState,
        obs: DockObservation,
    ) -> TwistCmd:
        """Advance only while 0x04 is visible; otherwise reacquire or demote.

        Failure mode from 83.log: weak-center latch → lose beam → spin slowly
        in the dark forever. Fix: faster sweep, geometry assist, early demote
        to BIAS when side IR returns, SEARCH only as last resort.
        """
        center_wz = self.center_trim_wz()
        center_vx = self.center_cruise_vx()
        coarse = self.coarse_relative(rel)
        seeing_c = self.sees_center_bit(obs.left_token, obs.right_token) or (
            self.sees_center_bit(self.l_last, self.r_last)
        )
        hunt_wz = abs(float(self.cfg.center_reacquire_wz))
        sweep_lim = abs(float(self.cfg.center_reacquire_sweep_rad))

        if seeing_c:
            self.center_lost_bit_elapsed_s = 0.0
            self.center_side_reacquire_elapsed_s = 0.0
            self.center_reacq_progress_rad = 0.0
            self.center_reacq_last_yaw = self.last_yaw_rad
            if self.last_yaw_rad is not None:
                self.center_lock_yaw = float(self.last_yaw_rad)
            if rel == RelativeState.CENTER:
                self.center_seen_strong = True

            # Strategy-1 CENTER: constant vx + Z-trim while keeping 0x04.
            bit_c = int(self.cfg.bit_c)
            l = norm_token(obs.left_token, self.cfg.token_mask)
            r = norm_token(obs.right_token, self.cfg.token_mask)
            l_c = (l & bit_c) != 0
            r_c = (r & bit_c) != 0
            if l_c and not r_c:
                self.center_zz_sign = 1  # CCW: pull right receiver into beam
                self.center_zz_elapsed_s = 0.0
            elif r_c and not l_c:
                self.center_zz_sign = -1  # CW: pull left receiver into beam
                self.center_zz_elapsed_s = 0.0
            else:
                self.center_zz_elapsed_s += max(0.0, float(dt))
                half = float(CENTER_ZZ_HALF_PERIOD_S)
                if self.center_zz_elapsed_s >= half:
                    self.center_zz_sign = -1 if self.center_zz_sign > 0 else 1
                    self.center_zz_elapsed_s = 0.0
            wz = float(self.center_zz_sign) * 0.55 * center_wz
            return TwistCmd(center_vx, wz)

        # Lost 0x04.
        self.center_lost_bit_elapsed_s += max(0.0, float(dt))

        # Side IR returned: demote to BIAS quickly — better than dark spinning.
        if (
            coarse in (RelativeState.LEFT, RelativeState.RIGHT)
            and self.center_lost_bit_elapsed_s >= float(self.cfg.center_lost_to_bias_s)
        ):
            self.debug_events.append('center_lost_to_bias')
            if coarse == RelativeState.LEFT:
                return self.enter_bias_left()
            return self.enter_bias_right()

        if coarse in (RelativeState.LEFT, RelativeState.RIGHT):
            self.center_side_hint = 1 if coarse == RelativeState.LEFT else -1
            self.center_reacq_dir = -1 if coarse == RelativeState.LEFT else 1
            self.center_reacq_progress_rad = 0.0
            return TwistCmd(
                0.0, hunt_wz if coarse == RelativeState.LEFT else -hunt_wz
            )

        # Geometry assist while dark: yaw to cancel lateral error.
        geo_wz = self.lateral_trim_wz(obs, hunt_wz)
        if abs(geo_wz) > 1e-6 and self.center_lost_bit_elapsed_s < 2.0:
            return TwistCmd(0.0, geo_wz)

        # Return toward last lock yaw first.
        if (
            self.center_lock_yaw is not None
            and self.last_yaw_rad is not None
            and self.center_lost_bit_elapsed_s < 1.2
        ):
            err = wrap_pi(float(self.center_lock_yaw) - float(self.last_yaw_rad))
            if abs(err) > math.radians(1.5):
                return TwistCmd(0.0, hunt_wz if err > 0.0 else -hunt_wz)

        # Sweep ±sweep_lim, flipping direction at each end (not a fixed timer).
        if self.last_yaw_rad is not None:
            if self.center_reacq_last_yaw is None:
                self.center_reacq_last_yaw = float(self.last_yaw_rad)
            else:
                self.center_reacq_progress_rad += abs(
                    wrap_pi(float(self.last_yaw_rad) - float(self.center_reacq_last_yaw))
                )
                self.center_reacq_last_yaw = float(self.last_yaw_rad)
            if self.center_reacq_progress_rad >= sweep_lim:
                self.center_reacq_dir = -int(self.center_reacq_dir) or -1
                self.center_reacq_progress_rad = 0.0

        if self.center_side_hint > 0:
            wz = hunt_wz
        elif self.center_side_hint < 0:
            wz = -hunt_wz
        else:
            wz = float(self.center_reacq_dir) * hunt_wz

        lost_limit = max(
            float(self.cfg.center_lost_to_search_s),
            float(self.cfg.no_signal_search_timeout_s)
            + float(self.cfg.center_side_reacquire_hold_s),
        )
        if self.center_lost_bit_elapsed_s >= lost_limit and (not self.is_near_dock()):
            self.state = DockState.SEARCH
            self.reset_search_scan()
            self.debug_events.append('center_lost_to_search')
            return TwistCmd(0.0, 0.0)
        return TwistCmd(0.0, wz)

    def reset_search_scan(self) -> None:
        self.search_start_yaw = None
        self.search_last_yaw = None
        self.search_yaw_progress = 0.0
        self.search_spin_sign = -1.0
        self.search_worst_dist_m = None
        self.search_dist_worsen_s = 0.0
        self.search_hits = {}
        self.cnt_weak_center = 0

    def search_spin_wz(self, obs: Optional[DockObservation], dt: float) -> float:
        """Spin toward dock bearing when dark; flip if dist keeps growing (95.log)."""
        mag = abs(float(self.cfg.search_wz))
        sign = float(self.search_spin_sign)

        if obs is not None and obs.dist_m is not None:
            d = float(obs.dist_m)
            if self.search_worst_dist_m is None:
                self.search_worst_dist_m = d
            elif d > float(self.search_worst_dist_m) + 0.015:
                self.search_dist_worsen_s += max(0.0, float(dt))
                self.search_worst_dist_m = d
            else:
                self.search_dist_worsen_s = max(0.0, self.search_dist_worsen_s - 0.5 * dt)
                self.search_worst_dist_m = min(float(self.search_worst_dist_m), d)
            if self.search_dist_worsen_s >= 2.0:
                sign *= -1.0
                self.search_spin_sign = sign
                self.search_dist_worsen_s = 0.0
                self.search_worst_dist_m = d
                self.debug_events.append('search_flip_spin')

        if (
            obs is not None
            and obs.odom_x is not None
            and obs.odom_y is not None
            and obs.yaw_rad is not None
            and obs.dock_x is not None
            and obs.dock_y is not None
        ):
            dx = float(obs.dock_x) - float(obs.odom_x)
            dy = float(obs.dock_y) - float(obs.odom_y)
            bearing = wrap_pi(math.atan2(dy, dx) - float(obs.yaw_rad))
            if abs(bearing) > math.radians(8.0):
                sign = 1.0 if bearing > 0.0 else -1.0
                self.search_spin_sign = sign

        return sign * mag

    def search_near_dock_creep(self, obs: DockObservation) -> TwistCmd:
        """Very close but no IR: align approach heading and creep in."""
        creep = min(0.04, abs(float(self.cfg.center_vx)))
        align_cap = min(0.18, abs(float(self.cfg.search_wz)))
        if (
            obs.yaw_rad is None
            or obs.odom_x is None
            or obs.odom_y is None
            or obs.dock_x is None
            or obs.dock_y is None
        ):
            return TwistCmd(0.0, 0.0)

        dx = float(obs.dock_x) - float(obs.odom_x)
        dy = float(obs.dock_y) - float(obs.odom_y)
        approach_yaw = math.atan2(dy, dx)
        yaw_err = wrap_pi(approach_yaw - float(obs.yaw_rad))
        body_x = math.cos(float(obs.yaw_rad)) * dx + math.sin(float(obs.yaw_rad)) * dy
        dist = float(obs.dist_m) if obs.dist_m is not None else 999.0

        if dist < 0.055:
            wz = 0.0
            if obs.lateral_m is not None:
                lat = float(obs.lateral_m)
                wz = max(-0.08, min(0.08, -2.0 * lat))
            return TwistCmd(min(0.045, creep * 1.4), float(wz))

        if abs(yaw_err) > math.radians(8.0):
            wz = max(-align_cap, min(align_cap, 2.5 * yaw_err))
            vx = 0.15 * creep if body_x > 0.01 else 0.0
            return TwistCmd(float(vx), float(wz))

        vx = creep if body_x > 0.006 else 0.0
        return TwistCmd(float(vx), max(-align_cap, min(align_cap, 0.6 * yaw_err)))

    def update_search_yaw_progress(self, yaw_rad: Optional[float]) -> None:
        if yaw_rad is None:
            return
        yaw = float(yaw_rad)
        if self.search_start_yaw is None:
            self.search_start_yaw = yaw
            self.search_last_yaw = yaw
            self.search_yaw_progress = 0.0
            return
        if self.search_last_yaw is None:
            self.search_last_yaw = yaw
            return
        # SEARCH rotates CW (negative wz); accumulate absolute yaw travel.
        delta = wrap_pi(yaw - float(self.search_last_yaw))
        self.search_yaw_progress += abs(float(delta))
        self.search_last_yaw = yaw

    def record_search_hit(self, region: RelativeState) -> None:
        side = region_to_bias_side(region)
        if side is None:
            return
        self.search_hits[region] = int(self.search_hits.get(region, 0)) + 1

    def pick_bias_side_from_hits(self) -> Optional[str]:
        """Choose bias side after a full turn: left/right > far > overlap."""
        best_side: Optional[str] = None
        best_score = -1
        for region, count in self.search_hits.items():
            side = region_to_bias_side(region)
            if side is None or count <= 0:
                continue
            score = region_side_priority(region) * 1000 + int(count)
            if score > best_score:
                best_score = score
                best_side = side
        return best_side

    def handle_search(
        self,
        dt: float,
        rel: RelativeState,
        obs: Optional[DockObservation],
    ) -> TwistCmd:
        """Capture IR and route into CENTER or BIAS.

        Priority:
        1. Strong CENTER (or gated WEAK_CENTER) → CENTER
        2. Raw 0x04 while confirming → freeze
        3. Any confirmed side / far / overlap → BIAS immediately
        4. No signal → keep scanning; full turn with no hits → retry
        """
        yaw = None if obs is None else obs.yaw_rad
        if yaw is None:
            yaw = self.last_yaw_rad
        self.update_search_yaw_progress(yaw)

        entered = self.try_enter_center(rel, obs, source='search')
        if entered is not None:
            self.reset_search_scan()
            return entered

        # Raw 0x04 while confirming: freeze only if geometry already allows
        # CENTER. Otherwise treat the flash as a side guide via receiver.
        if obs is not None and self.sees_center_bit(obs.left_token, obs.right_token):
            if self.geometry_allows_center(obs):
                return TwistCmd(0.0, 0.0)
            side = self.receiver_side_observation(obs.left_token, obs.right_token)
            if side == 'left':
                self.debug_events.append('search_raw_c_to_bias_left')
                self.reset_search_scan()
                return self.enter_bias_left()
            if side == 'right':
                self.debug_events.append('search_raw_c_to_bias_right')
                self.reset_search_scan()
                return self.enter_bias_right()
            return TwistCmd(0.0, 0.0)

        # Prefer the latest fine region (may be fresher than smoothed rel).
        guide = self.last_fine_region if self.last_fine_region != RelativeState.UNKNOWN else rel
        side = region_to_bias_side(guide)
        if side is None:
            side = region_to_bias_side(rel)
        if side is not None:
            self.record_search_hit(guide if region_to_bias_side(guide) is not None else rel)
            far_guide = (
                guide if region_to_bias_side(guide) is not None else rel
            )
            if side == 'left':
                is_far = far_guide == RelativeState.FAR_LEFT
                self.debug_events.append(
                    'search_guide_bias_left_far' if is_far
                    else 'search_guide_bias_left'
                )
                self.reset_search_scan()
                return (
                    self.enter_bias_left_far() if is_far
                    else self.enter_bias_left()
                )
            is_far = far_guide == RelativeState.FAR_RIGHT
            self.debug_events.append(
                'search_guide_bias_right_far' if is_far
                else 'search_guide_bias_right'
            )
            self.reset_search_scan()
            return (
                self.enter_bias_right_far() if is_far
                else self.enter_bias_right()
            )

        # Dark SEARCH: unidirectional CW spin only. Geometry lateral trim here
        # caused 87.log in-place wz oscillation (sign flips as yaw changes
        # front_y_error) with vx=0 and never acquiring IR.
        full_turn = max(math.pi, float(self.cfg.search_full_turn_rad))
        if self.search_yaw_progress >= full_turn:
            picked = self.pick_bias_side_from_hits()
            if picked == 'left':
                self.debug_events.append('search_full_turn_bias_left')
                self.reset_search_scan()
                return self.enter_bias_left()
            if picked == 'right':
                self.debug_events.append('search_full_turn_bias_right')
                self.reset_search_scan()
                return self.enter_bias_right()
            self.debug_events.append('search_full_turn_retry')
            self.search_yaw_progress = 0.0
            self.search_start_yaw = yaw
            self.search_last_yaw = yaw
            self.search_hits = {}

        if obs is not None and obs.dist_m is not None and float(obs.dist_m) < 0.08:
            return self.search_near_dock_creep(obs)

        return TwistCmd(0.0, self.search_spin_wz(obs, dt))


    def handle_bias(
        self,
        dt: float,
        rel: RelativeState,
        obs: DockObservation,
        is_left: bool,
        is_far: bool = False,
    ) -> TwistCmd:
        """BIAS zigzag: IR edge scan + odom forward. FAR promotes to INNER only."""
        self.bias_switch_lock_elapsed_s += max(0.0, float(dt))

        entered = self.try_enter_center(rel, obs, source='bias')
        if entered is not None:
            return entered
        if self.sees_center_bit(obs.left_token, obs.right_token):
            if self.geometry_allows_center(obs) or rel == RelativeState.CENTER:
                return TwistCmd(0.0, 0.0)

        l = norm_token(obs.left_token, self.cfg.token_mask)
        r = norm_token(obs.right_token, self.cfg.token_mask)
        bits = int(l) | int(r)
        # FAR → INNER when inner side bit appears (0x02 / 0x08), via bit AND.
        if is_far:
            if is_left and (bits & int(self.cfg.bit_l2)) != 0:
                self.debug_events.append('bias_left_far_to_bias_left')
                return self.enter_bias_left()
            if (not is_left) and (bits & int(self.cfg.bit_r2)) != 0:
                self.debug_events.append('bias_right_far_to_bias_right')
                return self.enter_bias_right()

        allow_switch = self.current_bias_phase_allows_switch()
        switch_samples = max(2, int(self.bias_switch_confirm_samples))
        if allow_switch and self.has_opposite_inner_beam(obs, is_left):
            if is_left:
                self.bias_l2r_count += 1
                self.bias_r2l_count = 0
                if self.bias_l2r_count >= switch_samples:
                    return self.enter_bias_right()
            else:
                self.bias_r2l_count += 1
                self.bias_l2r_count = 0
                if self.bias_r2l_count >= switch_samples:
                    return self.enter_bias_left()
        else:
            self.bias_l2r_count = 0
            self.bias_r2l_count = 0

        return self.track_bias_side(dt, is_left, obs, force_far=is_far)

    def has_opposite_inner_beam(self, obs: DockObservation, is_left: bool) -> bool:
        """True when the opposite INNER side beam (left=0x02 / right=0x08) is seen."""
        bit_inner = int(self.cfg.bit_r2) if is_left else int(self.cfg.bit_l2)
        l = norm_token(obs.left_token, self.cfg.token_mask)
        r = norm_token(obs.right_token, self.cfg.token_mask)
        return ((l | r) & bit_inner) != 0

    def sees_far_only(self, obs: Optional[DockObservation] = None) -> bool:
        """True when IR is FAR beam only (no inner left/right/center bits)."""
        if obs is None:
            l, r = self.l_last, self.r_last
        else:
            l = norm_token(obs.left_token, self.cfg.token_mask)
            r = norm_token(obs.right_token, self.cfg.token_mask)
        bits = int(l) | int(r)
        if bits == 0:
            return False
        far = int(self.cfg.bit_l1) | int(self.cfg.bit_r1)
        inner = int(self.cfg.bit_l2) | int(self.cfg.bit_r2) | int(self.cfg.bit_c)
        return (bits & far) != 0 and (bits & inner) == 0

    def _bias_scan_need_frames(self, is_far: bool) -> int:
        if is_far and self.bias_zz_first_scan:
            return ZZ_DARK_FRAMES_FAR_FIRST
        return ZZ_DARK_FRAMES

    def _bias_forward_m(self, is_far: bool) -> float:
        return ZZ_FORWARD_M_FAR if is_far else ZZ_FORWARD_M

    def _advance_bias_scan_phase(self, next_phase: BiasZigzagPhase) -> TwistCmd:
        self.bias_zz_phase = next_phase
        self._reset_bias_scan_edge()
        self.clear_forward_task()
        return TwistCmd(0.0, 0.0)

    def track_bias_side(
        self,
        dt: float,
        is_left: bool,
        obs: Optional[DockObservation] = None,
        *,
        force_far: bool = False,
    ) -> TwistCmd:
        """Rotate to IR edge → odom forward → opposite edge (docking_strategy.md).

        LEFT*:  CCW watch right → fwd → CW watch left
        RIGHT*: CW watch left  → fwd → CCW watch right
        FAR uses 5-frame first edge and 10cm steps; INNER uses 3 / 5cm.
        Edge = named receiver had signal, then continuous N frames of 0x00.
        """
        raw_bits = 0
        if obs is not None:
            raw_bits = norm_token(obs.left_token, self.cfg.token_mask) | norm_token(
                obs.right_token, self.cfg.token_mask
            )
            if self.sees_center_bit(obs.left_token, obs.right_token):
                self.bias_zz_seen_center = True
        has_raw = raw_bits != 0
        if has_raw:
            self.bias_lost_signal_elapsed_s = 0.0
            if obs is not None and obs.dist_m is not None:
                d = float(obs.dist_m)
                if self.bias_best_dist_m is None:
                    self.bias_best_dist_m = d
                else:
                    self.bias_best_dist_m = min(float(self.bias_best_dist_m), d)
        else:
            self.bias_lost_signal_elapsed_s += max(0.0, float(dt))

        on_far = bool(force_far or self.sees_far_only(obs))
        track_wz = abs(float(self.cfg.bias_track_wz))
        if on_far:
            track_wz = abs(float(self.cfg.bias_far_scan_wz))

        if self.bias_zz_phase == BiasZigzagPhase.FORWARD:
            self.bias_lost_signal_elapsed_s = 0.0
            cmd = self.forward_distance(self._bias_forward_m(on_far))
            if not self.forward_task_active:
                return self._advance_bias_scan_phase(BiasZigzagPhase.SCAN_SECONDARY)
            return cmd

        # Lost IR while scanning → SEARCH.
        if not has_raw and self.bias_lost_signal_elapsed_s >= 1.8:
            self.state = DockState.SEARCH
            self.reset_search_scan()
            return TwistCmd(0.0, self.search_spin_wz(obs, dt))
        if (
            not has_raw
            and self.bias_lost_signal_elapsed_s >= 1.0
            and obs is not None
            and obs.dist_m is not None
            and self.bias_best_dist_m is not None
            and float(obs.dist_m) > float(self.bias_best_dist_m) + 0.05
        ):
            self.state = DockState.SEARCH
            self.reset_search_scan()
            return TwistCmd(0.0, self.search_spin_wz(obs, dt))

        primary = self.bias_zz_phase == BiasZigzagPhase.SCAN_PRIMARY
        l_tok = (
            norm_token(obs.left_token, self.cfg.token_mask) if obs is not None else 0
        )
        r_tok = (
            norm_token(obs.right_token, self.cfg.token_mask) if obs is not None else 0
        )
        # Table: (is_left, primary) → (wz_sign, watch_right_receiver)
        if primary:
            wz = track_wz if is_left else -track_wz
            watch_tok = r_tok if is_left else l_tok
        else:
            wz = -track_wz if is_left else track_wz
            watch_tok = l_tok if is_left else r_tok
        has_watch = watch_tok != 0

        if self.last_yaw_rad is not None:
            if self.bias_zz_last_scan_yaw is None:
                self.bias_zz_last_scan_yaw = float(self.last_yaw_rad)
            else:
                self.bias_zz_scan_progress += abs(
                    wrap_pi(float(self.last_yaw_rad) - float(self.bias_zz_last_scan_yaw))
                )
                self.bias_zz_last_scan_yaw = float(self.last_yaw_rad)

        if (
            (not self.bias_zz_seen_center)
            and self.bias_zz_scan_progress >= ZZ_MAX_SCAN_RAD
        ):
            return self._bias_reclassify_after_full_turn(obs)

        if has_watch:
            self.bias_zz_seen_watch = True
            self.bias_zz_dark_streak = 0
        elif self.bias_zz_seen_watch:
            self.bias_zz_dark_streak += 1
        else:
            self.bias_zz_dark_streak = 0

        need = self._bias_scan_need_frames(on_far)
        edge_hit = self.bias_zz_seen_watch and self.bias_zz_dark_streak >= need
        if edge_hit:
            if primary:
                self.bias_zz_first_scan = False
                self.bias_zz_phase = BiasZigzagPhase.FORWARD
                self._reset_bias_scan_edge()
                self.clear_forward_task()
                return self.forward_distance(self._bias_forward_m(on_far))
            return self._advance_bias_scan_phase(BiasZigzagPhase.SCAN_PRIMARY)

        return TwistCmd(0.0, wz)

    def forward_distance(self, distance: float) -> TwistCmd:
        """Moves forward for requested distance in meters using odom."""
        target = max(0.0, float(distance))
        if target <= 1e-6:
            self.clear_forward_task()
            return TwistCmd(0.0, 0.0)

        if self.odom_x is None or self.odom_y is None:
            return TwistCmd(0.0, 0.0)

        # Start or retarget forward task from current odom pose.
        if (not self.forward_task_active) or (abs(target - self.forward_task_target_m) > 1e-6):
            self.forward_task_active = True
            self.forward_task_target_m = target
            self.forward_task_progress_m = 0.0
            self.forward_start_x = float(self.odom_x)
            self.forward_start_y = float(self.odom_y)

        if self.forward_start_x is None or self.forward_start_y is None:
            return TwistCmd(0.0, 0.0)

        dx = float(self.odom_x) - float(self.forward_start_x)
        dy = float(self.odom_y) - float(self.forward_start_y)
        self.forward_task_progress_m = math.hypot(dx, dy)
        if self.forward_task_progress_m >= self.forward_task_target_m:
            self.forward_task_active = False
            self.forward_task_progress_m = self.forward_task_target_m
            self.forward_start_x = None
            self.forward_start_y = None
            return TwistCmd(0.0, 0.0)
        return TwistCmd(float(self.cfg.center_vx), 0.0)

