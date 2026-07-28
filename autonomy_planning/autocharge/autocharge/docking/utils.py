"""Docking helper utilities: IR tokens and geometry."""

from __future__ import annotations

import math
from typing import ClassVar, Optional

from autocharge.common.geometry import wrap_pi


class IrToken:
    """IR emitter bits (docking_strategy.md).

    far_left=0x01, left=0x02, center=0x04, right=0x08, far_right=0x10
    """

    BIT_FAR_LEFT = 0x01
    BIT_LEFT = 0x02
    BIT_CENTER = 0x04
    BIT_RIGHT = 0x08
    BIT_FAR_RIGHT = 0x10
    # Aliases matching DockingConfig field names / legacy ROS params.
    BIT_L1 = BIT_FAR_LEFT
    BIT_L2 = BIT_LEFT
    BIT_C = BIT_CENTER
    BIT_R1 = BIT_RIGHT  # inner right (ROS docking.bit_right)
    BIT_R2 = BIT_FAR_RIGHT  # far right (ROS docking.bit_right_ext)
    LOW_MASK = 0x1F
    known_tokens: ClassVar[Optional[frozenset[int]]] = None

    @classmethod
    def norm(cls, v: int, mask: int | None = None) -> int:
        return int(v) & int(mask if mask is not None else cls.LOW_MASK)

    @classmethod
    def known_token_set(cls) -> frozenset[int]:
        """All combinations of the five emitter bits (0x00..0x1F) are valid."""
        if cls.known_tokens is None:
            cls.known_tokens = frozenset(range(cls.LOW_MASK + 1))
        return cls.known_tokens

    @classmethod
    def format_combo(cls, v: int) -> str:
        """Format token as hex plus bit list, e.g. 0x1c=0x10|0x08|0x04."""
        t = cls.norm(v)
        if t == 0:
            return '0x00'
        parts: list[str] = []
        for bit in (
            cls.BIT_FAR_RIGHT,
            cls.BIT_RIGHT,
            cls.BIT_CENTER,
            cls.BIT_LEFT,
            cls.BIT_FAR_LEFT,
        ):
            if (t & bit) != 0:
                parts.append(f'0x{bit:02x}')
        if len(parts) == 1:
            return f'0x{t:02x}'
        return f'0x{t:02x}=' + '|'.join(parts)


def norm_token(v: int, mask: int) -> int:
    return IrToken.norm(v, mask)


def heading_error_to(
    target_x: float,
    target_y: float,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
) -> float:
    dx = float(target_x) - float(robot_x)
    dy = float(target_y) - float(robot_y)
    if (dx * dx + dy * dy) <= 1e-10:
        return 0.0
    to_target_yaw = math.atan2(dy, dx)
    return wrap_pi(to_target_yaw - float(robot_yaw))


def robot_back_facing_target(
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    target_x: float,
    target_y: float,
    *,
    back_reject_deg: float,
) -> bool:
    err = heading_error_to(target_x, target_y, robot_x, robot_y, robot_yaw)
    return abs(err) >= math.radians(float(back_reject_deg))


def body_contact_point(
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    contact_x_m: float,
    contact_y_m: float,
) -> tuple[float, float]:
    c = math.cos(robot_yaw)
    s = math.sin(robot_yaw)
    x = float(robot_x) + c * float(contact_x_m) - s * float(contact_y_m)
    y = float(robot_y) + s * float(contact_x_m) + c * float(contact_y_m)
    return x, y


def dock_front_point(
    dock_x: float,
    dock_y: float,
    dock_yaw: float,
    front_x_offset_m: float,
    front_y_offset_m: float,
) -> tuple[float, float]:
    dock_c = math.cos(float(dock_yaw))
    dock_s = math.sin(float(dock_yaw))
    x = float(dock_x) + dock_c * float(front_x_offset_m) - dock_s * float(front_y_offset_m)
    y = float(dock_y) + dock_s * float(front_x_offset_m) + dock_c * float(front_y_offset_m)
    return x, y


def dock_pose_from_predock(
    predock_x: float,
    predock_y: float,
    predock_yaw: float,
    distance_m: float,
) -> tuple[float, float, float]:
    dist = max(0.05, float(distance_m))
    approach_yaw = float(predock_yaw)
    c = math.cos(approach_yaw)
    s = math.sin(approach_yaw)
    dock_x = float(predock_x) + dist * c
    dock_y = float(predock_y) + dist * s
    dock_yaw = wrap_pi(approach_yaw + math.pi)
    return dock_x, dock_y, dock_yaw


def distance_2d(x0: float, y0: float, x1: float, y1: float) -> float:
    dx = float(x1) - float(x0)
    dy = float(y1) - float(y0)
    return math.hypot(dx, dy)


def is_center_pair_tokens(
    left_token: int,
    right_token: int,
    *,
    bit_l1: int,
    bit_l2: int,
    bit_c: int,
    bit_r2: int,
    bit_r1: int,
    token_mask: int = 0x1F,
) -> bool:
    l = norm_token(left_token, token_mask)
    r = norm_token(right_token, token_mask)
    left_req = bit_r2 | bit_c
    right_req = bit_r2 | bit_l2
    near_left_req = bit_r1 | bit_r2
    near_right_req = bit_l2 | bit_l1
    return (
        (((l & left_req) == left_req) and ((r & right_req) == right_req))
        or (((l & near_left_req) == near_left_req) and ((r & near_right_req) == near_right_req))
        or ((l == bit_c) and (r == 0))
        or ((r == bit_c) and (l == 0))
        or ((l == bit_c) and (r == bit_c))
    )


def classify_ir_region(
    left_token: int,
    right_token: int,
    *,
    bit_l1: int,
    bit_l2: int,
    bit_c: int,
    bit_r2: int,
    bit_r1: int,
    token_mask: int = 0x1F,
):
    """Classify IR pair into fine-grained docking regions."""
    from autocharge.docking.type import RelativeState

    l = norm_token(left_token, token_mask)
    r = norm_token(right_token, token_mask)
    if l == 0 and r == 0:
        return RelativeState.UNKNOWN

    if is_center_pair_tokens(
        l,
        r,
        bit_l1=bit_l1,
        bit_l2=bit_l2,
        bit_c=bit_c,
        bit_r2=bit_r2,
        bit_r1=bit_r1,
        token_mask=token_mask,
    ):
        if ((l == bit_c) and (r == 0)) or ((r == bit_c) and (l == 0)):
            return RelativeState.WEAK_CENTER
        return RelativeState.CENTER

    def has_bit(tok: int, bit: int) -> bool:
        if tok == 0 or bit == 0:
            return False
        return (tok & bit) != 0

    has_far_l = has_bit(l, bit_l1) or has_bit(r, bit_l1)
    has_left = has_bit(l, bit_l2) or has_bit(r, bit_l2)
    has_c = has_bit(l, bit_c) or has_bit(r, bit_c)
    has_right = has_bit(l, bit_r2) or has_bit(r, bit_r2)
    has_far_r = has_bit(l, bit_r1) or has_bit(r, bit_r1)

    if has_c:
        return RelativeState.WEAK_CENTER

    # Single-sided: side from emitter bits (not receiver index alone).
    # Receiver-only remap of far_right→LEFT caused wrong BIAS in 88.log.
    if l != 0 and r == 0:
        if has_far_l and has_left:
            return RelativeState.OVERLAP_LEFT
        if has_far_l and not has_left:
            return RelativeState.FAR_LEFT
        if has_left:
            return RelativeState.LEFT
        if has_far_r and not has_right:
            return RelativeState.FAR_RIGHT
        if has_right:
            return RelativeState.RIGHT
        return RelativeState.LEFT
    if r != 0 and l == 0:
        if has_far_r and has_right:
            return RelativeState.OVERLAP_RIGHT
        if has_far_r and not has_right:
            return RelativeState.FAR_RIGHT
        if has_right:
            return RelativeState.RIGHT
        if has_far_l and not has_left:
            return RelativeState.FAR_LEFT
        if has_left:
            return RelativeState.LEFT
        return RelativeState.RIGHT

    if has_far_l and has_left:
        return RelativeState.OVERLAP_LEFT
    if has_far_r and has_right:
        return RelativeState.OVERLAP_RIGHT

    if has_left and not has_right and not has_far_r:
        return RelativeState.LEFT
    if has_right and not has_left and not has_far_l:
        return RelativeState.RIGHT
    if has_far_l and not has_far_r:
        return RelativeState.FAR_LEFT
    if has_far_r and not has_far_l:
        return RelativeState.FAR_RIGHT
    if has_left and has_right:
        return RelativeState.OVERLAP_LEFT if has_bit(l, bit_l2) else RelativeState.OVERLAP_RIGHT
    return RelativeState.UNKNOWN


def region_to_bias_side(region) -> Optional[str]:
    """Map a fine region to bias side: 'left', 'right', or None for center/unknown."""
    from autocharge.docking.type import RelativeState

    if region in (
        RelativeState.LEFT,
        RelativeState.FAR_LEFT,
        RelativeState.OVERLAP_LEFT,
    ):
        return 'left'
    if region in (
        RelativeState.RIGHT,
        RelativeState.FAR_RIGHT,
        RelativeState.OVERLAP_RIGHT,
    ):
        return 'right'
    return None


def region_side_priority(region) -> int:
    """Higher is better when choosing a side after a full search turn."""
    from autocharge.docking.type import RelativeState

    if region in (RelativeState.LEFT, RelativeState.RIGHT):
        return 3
    if region in (RelativeState.FAR_LEFT, RelativeState.FAR_RIGHT):
        return 2
    if region in (RelativeState.OVERLAP_LEFT, RelativeState.OVERLAP_RIGHT):
        return 1
    return 0
