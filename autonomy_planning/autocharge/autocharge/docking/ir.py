"""IR token temporal filtering."""

from __future__ import annotations

from collections import Counter, deque

from autocharge.docking.type import DockingConfig
from autocharge.docking.utils import IrToken, robot_back_facing_target


class IrFilter:
    """Heading gate + region-stable IR token filtering."""

    def __init__(
        self,
        cfg: DockingConfig,
        *,
        heading_gate_enable: bool,
        back_reject_deg: float,
    ) -> None:
        self.cfg = cfg
        self.heading_gate_enable = bool(heading_gate_enable)
        self.back_reject_deg = max(90.0, float(back_reject_deg))
        self.pair_window: deque[tuple[int, int]] = deque(maxlen=5)
        self.l_eval = 0
        self.r_eval = 0

    def reset(self) -> None:
        self.pair_window.clear()
        self.l_eval = 0
        self.r_eval = 0

    def validate_raw(self, left_raw: int, right_raw: int) -> tuple[int, int, bool]:
        """Normalize low-5 bits. Combo tokens (e.g. 0x18) are valid."""
        l_val = IrToken.norm(left_raw)
        r_val = IrToken.norm(right_raw)
        # Only reject bits outside the five-emitter mask.
        unknown = ((int(left_raw) | int(right_raw)) & ~IrToken.LOW_MASK) != 0
        return l_val, r_val, unknown

    def eval_tokens(
        self,
        cycle: int,
        eval_every: int,
        ir_left: int,
        ir_right: int,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        dock_x: float,
        dock_y: float,
    ) -> tuple[int, int]:
        if cycle % eval_every == 0:
            l_raw, r_raw = self.filtered_tokens(
                ir_left, ir_right, robot_x, robot_y, robot_yaw, dock_x, dock_y
            )
            ir_l, ir_r = self.stable_tokens(l_raw, r_raw)
            self.l_eval = int(ir_l)
            self.r_eval = int(ir_r)
            return ir_l, ir_r
        return self.l_eval, self.r_eval

    def filtered_tokens(
        self,
        ir_left: int,
        ir_right: int,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        dock_x: float,
        dock_y: float,
    ) -> tuple[int, int]:
        l = int(ir_left)
        r = int(ir_right)
        if not self.heading_gate_enable:
            return (l, r)
        if not robot_back_facing_target(
            robot_x,
            robot_y,
            robot_yaw,
            dock_x,
            dock_y,
            back_reject_deg=self.back_reject_deg,
        ):
            return (l, r)
        if self.query_region(l, r) == 0:
            return (l, r)
        return (0, 0)

    def stable_tokens(self, l: int, r: int) -> tuple[int, int]:
        self.pair_window.append((int(l), int(r)))
        if self.query_region(int(l), int(r)) == 0:
            return (int(l), int(r))
        if len(self.pair_window) < self.pair_window.maxlen:
            return (int(l), int(r))
        regions = [self.query_region(int(li), int(ri)) for (li, ri) in self.pair_window]
        valid_regions = [rg for rg in regions if rg in (0, 1, 2)]
        if not valid_regions:
            return (int(l), int(r))
        region_top, region_count = Counter(valid_regions).most_common(1)[0]
        if int(region_count) < 3:
            return (int(l), int(r))
        pairs_in_region = [
            (int(li), int(ri))
            for (li, ri), rg in zip(self.pair_window, regions)
            if rg == region_top
        ]
        if not pairs_in_region:
            return (0, 0)
        pair_top, _ = Counter(pairs_in_region).most_common(1)[0]
        return (int(pair_top[0]), int(pair_top[1]))

    def query_region(self, l: int, r: int) -> int:
        l = IrToken.norm(l)
        r = IrToken.norm(r)
        bit_l1 = int(self.cfg.bit_l1)
        bit_l2 = int(self.cfg.bit_l2)
        bit_c = int(self.cfg.bit_c)
        bit_r2 = int(self.cfg.bit_r2)
        bit_r1 = int(self.cfg.bit_r1)
        if l == 0 and r == 0:
            return -1
        left_req = bit_r2 | bit_c
        right_req = bit_r2 | bit_l2
        near_left_req = bit_r1 | bit_r2
        near_right_req = bit_l2 | bit_l1
        if (
            (((l & left_req) == left_req) and ((r & right_req) == right_req))
            or (((l & near_left_req) == near_left_req) and ((r & near_right_req) == near_right_req))
            or ((l == bit_c) and (r == 0))
            or ((r == bit_c) and (l == 0))
            or ((l == bit_c) and (r == bit_c))
        ):
            return 0
        if ((l & bit_l2) != 0) and ((r & bit_l2) != 0):
            return 1
        if ((l & bit_r2) != 0) and ((r & bit_r2) != 0):
            return 2
        return -1
