"""Shared type definitions for the docking stack."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
import math
from typing import Optional

from autocharge.common.geometry import wrap_pi, yaw_to_quat
from autocharge.docking.utils import (
    body_contact_point,
    distance_2d,
    dock_front_point,
    dock_pose_from_predock,
)
from geometry_msgs.msg import PoseStamped

# ---------------------------------------------------------------------------
# Strategy-1 constants (docking_strategy.md). Not exposed as ROS params.
# ---------------------------------------------------------------------------
ZZ_DARK_FRAMES = 3
ZZ_DARK_FRAMES_FAR_FIRST = 5
ZZ_FORWARD_M = 0.05
ZZ_FORWARD_M_FAR = 0.10
ZZ_MAX_SCAN_RAD = 2.0 * math.pi
BIAS_FAR_SCAN_WZ_SCALE = 1.35
CENTER_ZZ_HALF_PERIOD_S = 0.55


class MissionPhase(Enum):
    IDLE = auto()
    PREDOCK = auto()
    DOCKING = auto()


@dataclass
class ActionMissionContext:
    """Bookkeeping for an active dock_to_charger action goal."""

    active: bool = False
    timeout_s: float = 0.0
    start_time_ns: int = 0
    attempt_count: int = 0
    cancel_requested: bool = False
    timed_out: bool = False
    overrun_abort: bool = False


@dataclass
class MissionState:
    """Runtime mission bookkeeping for the docking node."""

    phase: MissionPhase = MissionPhase.IDLE
    active: bool = False
    charge_connected: bool = False
    abort: bool = False
    reset_pose_on_reset: bool = True
    action: ActionMissionContext = field(default_factory=ActionMissionContext)


class DockState(Enum):
    SEARCH = auto()
    CENTER = auto()
    BIAS_LEFT_FAR = auto()
    BIAS_RIGHT_FAR = auto()
    BIAS_LEFT = auto()
    BIAS_RIGHT = auto()
    SUCCESS = auto()
    ABORT = auto()


class RelativeState(Enum):
    UNKNOWN = auto()
    LEFT = auto()
    RIGHT = auto()
    CENTER = auto()
    WEAK_CENTER = auto()
    FAR_LEFT = auto()
    FAR_RIGHT = auto()
    OVERLAP_LEFT = auto()
    OVERLAP_RIGHT = auto()


class BiasZigzagPhase(Enum):
    """BIAS zigzag: scan IR edge → odom forward → opposite edge.

    LEFT / LEFT_FAR:   CCW→right dark → fwd → CW→left dark
    RIGHT / RIGHT_FAR: CW→left dark  → fwd → CCW→right dark
    FAR: first edge 5 frames, forward 10cm; INNER: 3 frames, 5cm.
    """

    SCAN_PRIMARY = auto()
    FORWARD = auto()
    SCAN_SECONDARY = auto()


@dataclass(frozen=True)
class DockingConfig:
    """ROS-tunable docking knobs. Timing/geometry constants are derived below."""

    token_mask: int = 0x1F
    # far_left=0x01, left=0x02, center=0x04, right=0x08, far_right=0x10
    bit_l1: int = 0x01
    bit_l2: int = 0x02
    bit_c: int = 0x04
    bit_r2: int = 0x08  # inner right
    bit_r1: int = 0x10  # far right

    # Speeds (primary motion knobs).
    center_vx: float = 0.05
    center_trim_wz: float = 0.05
    search_wz: float = 0.25
    bias_track_vx: float = 0.022
    bias_track_wz: float = 0.22

    # IR smoothing.
    ir_decision_period_s: float = 0.20
    state_confirm_samples: int = 2
    no_signal_search_timeout_s: float = 0.75

    # dt clamp for tick().
    min_dt_s: float = 0.02
    max_dt_s: float = 0.30

    @property
    def weak_center_confirm_samples(self) -> int:
        return max(1, int(self.state_confirm_samples))

    @property
    def search_full_turn_rad(self) -> float:
        return 2.0 * math.pi

    @property
    def center_hold_s(self) -> float:
        return 0.75

    @property
    def center_side_reacquire_hold_s(self) -> float:
        return 0.9

    @property
    def center_unknown_creep_vx(self) -> float:
        return 0.4 * abs(float(self.center_vx))

    @property
    def bias_switch_lock_s(self) -> float:
        return 1.5

    @property
    def bias_switch_confirm_samples(self) -> int:
        return 8

    @property
    def bias_far_scan_wz(self) -> float:
        """FAR-region scan angular speed (strategy 1)."""
        return BIAS_FAR_SCAN_WZ_SCALE * abs(float(self.bias_track_wz))

    @property
    def dock_ir_refine_y_m(self) -> float:
        return 0.08

    @property
    def dock_ir_refine_yaw_rad(self) -> float:
        return 0.06

    @property
    def dock_ir_refine_alpha(self) -> float:
        return 0.92

    @property
    def center_near_no_backup_dist_m(self) -> float:
        return 0.60

    @property
    def center_near_dist_m(self) -> float:
        return 0.18

    @property
    def center_near_vx(self) -> float:
        return 0.6 * abs(float(self.center_vx))

    @property
    def center_near_wz(self) -> float:
        return 0.3 * abs(float(self.center_trim_wz))

    @property
    def center_very_near_dist_m(self) -> float:
        return 0.10

    @property
    def center_very_near_vx(self) -> float:
        return 0.4 * abs(float(self.center_vx))

    @property
    def center_very_near_wz(self) -> float:
        return 0.16 * abs(float(self.center_trim_wz))

    @property
    def center_enter_lateral_m(self) -> float:
        return 0.04

    @property
    def center_enter_yaw_rad(self) -> float:
        return 0.20

    @property
    def center_reacquire_wz(self) -> float:
        return max(0.12, 0.85 * abs(float(self.bias_track_wz)))

    @property
    def center_reacquire_sweep_rad(self) -> float:
        return 0.70

    @property
    def center_lost_to_bias_s(self) -> float:
        return 0.8

    @property
    def center_lost_to_search_s(self) -> float:
        return 4.0


@dataclass
class DockGeometry:
    dock_x: float
    dock_y: float
    dock_yaw: float
    dock_front_x_offset_m: float = 0.0
    dock_front_y_offset_m: float = 0.0
    success_contact_x_m: float = 0.16
    success_contact_y_m: float = 0.0
    success_use_distance: bool = True
    _defaults: dict[str, float | bool] = field(default_factory=dict, repr=False)

    def capture_defaults(self) -> None:
        self._defaults = {
            'dock_x': self.dock_x,
            'dock_y': self.dock_y,
            'dock_yaw': self.dock_yaw,
            'dock_front_x_offset_m': self.dock_front_x_offset_m,
            'dock_front_y_offset_m': self.dock_front_y_offset_m,
            'success_contact_x_m': self.success_contact_x_m,
            'success_contact_y_m': self.success_contact_y_m,
            'success_use_distance': self.success_use_distance,
        }

    def restore_defaults(self) -> None:
        for key, value in self._defaults.items():
            setattr(self, key, value)

    def update_from_predock(
        self,
        predock_x: float,
        predock_y: float,
        predock_yaw: float,
        distance_m: float,
    ) -> None:
        self.dock_x, self.dock_y, self.dock_yaw = dock_pose_from_predock(
            predock_x, predock_y, predock_yaw, distance_m
        )

    def distance_to_dock(
        self, robot_x: float, robot_y: float, robot_yaw: float
    ) -> Optional[float]:
        if not self.success_use_distance:
            return None
        contact_x, contact_y = body_contact_point(
            robot_x,
            robot_y,
            robot_yaw,
            self.success_contact_x_m,
            self.success_contact_y_m,
        )
        dock_front_x, dock_front_y = dock_front_point(
            self.dock_x,
            self.dock_y,
            self.dock_yaw,
            self.dock_front_x_offset_m,
            self.dock_front_y_offset_m,
        )
        return distance_2d(contact_x, contact_y, dock_front_x, dock_front_y)

    def signed_gap_to_dock_front(
        self, robot_x: float, robot_y: float, robot_yaw: float
    ) -> Optional[float]:
        """Signed gap along dock +X (out toward robot).

        Positive: contact point is on the robot side of the dock front plane.
        Negative: contact has crossed the dock front (overrun).
        """
        if not self.success_use_distance:
            return None
        contact_x, contact_y = body_contact_point(
            robot_x,
            robot_y,
            robot_yaw,
            self.success_contact_x_m,
            self.success_contact_y_m,
        )
        dock_front_x, dock_front_y = dock_front_point(
            self.dock_x,
            self.dock_y,
            self.dock_yaw,
            self.dock_front_x_offset_m,
            self.dock_front_y_offset_m,
        )
        ux = math.cos(float(self.dock_yaw))
        uy = math.sin(float(self.dock_yaw))
        dx = float(contact_x) - float(dock_front_x)
        dy = float(contact_y) - float(dock_front_y)
        return dx * ux + dy * uy

    def front_y_error(self, robot_x: float, robot_y: float, robot_yaw: float) -> float:
        _, front_y = body_contact_point(
            robot_x,
            robot_y,
            robot_yaw,
            self.success_contact_x_m,
            self.success_contact_y_m,
        )
        return float(front_y) - float(self.dock_y)

    def yaw_error(self, robot_yaw: float) -> float:
        return wrap_pi(float(robot_yaw) - float(self.dock_yaw))


@dataclass(frozen=True)
class DockObservation:
    left_token: int
    right_token: int
    dist_m: Optional[float] = None
    yaw_rad: Optional[float] = None
    odom_x: Optional[float] = None
    odom_y: Optional[float] = None
    lateral_m: Optional[float] = None
    yaw_err_rad: Optional[float] = None
    dock_x: Optional[float] = None
    dock_y: Optional[float] = None
    dock_yaw: Optional[float] = None
    abort: bool = False
    charge_connected: bool = False


@dataclass(frozen=True)
class TwistCmd:
    vx: float
    wz: float


@dataclass(frozen=True)
class PredockGoal:
    """Target pose in front of the charging dock."""

    x: float
    y: float
    yaw: float

    def to_msg(self, *, frame_id: str, stamp) -> PoseStamped:
        qx, qy, qz, qw = yaw_to_quat(self.yaw)
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg


@dataclass(frozen=True)
class PredockNoise:
    """Gaussian noise applied when sampling a predock pose."""

    position_std_m: float = 0.005
    yaw_deg: float = 10.0


@dataclass(frozen=True)
class PredockLimits:
    """Tolerance and speed limits for predock navigation."""

    goal_pos_tol_m: float = 0.05
    goal_yaw_tol_deg: float = 5.0
    max_linear_speed: float = 0.08
    max_angular_speed: float = 0.20
    approach_slow_radius_m: float = 0.25
    dist_scale_ref_m: float = 0.50
    yaw_finish_pos_factor: float = 1.5
    yaw_finish_kp: float = 1.0
    min_linear_speed: float = 0.02
    near_goal_min_linear_speed: float = 0.02
