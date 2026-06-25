"""Waypoint filtering."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Iterable

from nav_msgs.msg import OccupancyGrid

from autonomy_task.config import FilterConfig, Thresholds
from autonomy_task.waypoint import Status, Waypoint


@dataclass
class Pose:
    x: float
    y: float
    yaw: float = 0.0


def _bucket_label(lo: float, hi: float) -> str:
    return f'{lo:g}-{hi:g}'


def _summarize_reasons(reasons: dict[str, int]) -> str:
    if not reasons:
        return 'no_pending'
    top = sorted(reasons.items(), key=lambda item: -item[1])[:3]
    return ', '.join(f'{k}={v}' for k, v in top)


def bucket_counts_from_records(
    records: Iterable[Any],
    buckets: list[tuple[float, float]],
) -> dict[str, int]:
    """Count collected episodes per straight-line distance bucket."""
    counts = {_bucket_label(lo, hi): 0 for lo, hi in buckets}
    for rec in records:
        dist = float(getattr(rec, 'straight_line_m', 0.0))
        for lo, hi in buckets:
            if lo <= dist < hi:
                counts[_bucket_label(lo, hi)] += 1
                break
    return counts


def _map_cell(grid: OccupancyGrid, x: float, y: float) -> int | None:
    res = grid.info.resolution
    if res <= 0.0:
        return None
    col = int((x - grid.info.origin.position.x) / res)
    row = int((y - grid.info.origin.position.y) / res)
    if col < 0 or row < 0 or col >= grid.info.width or row >= grid.info.height:
        return None
    return int(grid.data[row * grid.info.width + col])


class Filter:
    """Select and reject waypoints for collection."""

    def __init__(self, thresholds: Thresholds, cfg: FilterConfig) -> None:
        self._t = thresholds
        self._f = cfg

    def dedupe(self, waypoints: list[Waypoint]) -> None:
        radius = self._t.dedupe_radius_m
        if radius <= 0.0:
            return
        kept: list[Waypoint] = []
        for wp in waypoints:
            dup = next((k for k in kept if wp.dist_to(k.x, k.y) < radius), None)
            if dup:
                wp.status = Status.SKIPPED
                wp.skip_reason = f'duplicate_of:{dup.id}'
            else:
                kept.append(wp)

    def apply_state(
        self,
        waypoints: list[Waypoint],
        collected: set[str],
        failed: dict[str, int],
        skipped: dict[str, str] | None = None,
    ) -> None:
        skipped = skipped or {}
        for wp in waypoints:
            if wp.id in collected:
                wp.status = Status.COLLECTED
                wp.skip_reason = ''
            elif wp.id in skipped:
                wp.status = Status.SKIPPED
                wp.skip_reason = skipped[wp.id]
            elif wp.id in failed:
                wp.attempts = failed[wp.id]
                if wp.attempts > self._t.max_retries:
                    wp.status = Status.FAILED
                    wp.skip_reason = 'max_retries'

    def prefilter(
        self,
        waypoints: list[Waypoint],
        grid: OccupancyGrid | None,
        *,
        only_ids: set[str] | None = None,
    ) -> None:
        if grid is None or not (self._f.enforce_map_bounds or self._f.require_navigable):
            return
        for wp in waypoints:
            if wp.status != Status.PENDING:
                continue
            if only_ids is not None and wp.id not in only_ids:
                continue
            reason = self._map_reason(wp.x, wp.y, grid)
            if reason:
                wp.status = Status.SKIPPED
                wp.skip_reason = reason

    def can_assign(
        self,
        wp: Waypoint,
        pose: Pose | None,
        done: Iterable[Waypoint],
        assigned: set[str],
        grid: OccupancyGrid | None,
        *,
        valid_graph_ids: set[str] | None = None,
        peer_poses: Iterable[Pose] | None = None,
        peer_targets: Iterable[Waypoint] | None = None,
        bucket_range: tuple[float, float] | None = None,
        relax_peer: bool = False,
    ) -> tuple[bool, str]:
        if wp.status in (
            Status.COLLECTED, Status.SKIPPED, Status.BLOCKED,
            Status.IN_PROGRESS, Status.FAILED,
        ):
            return False, f'status:{wp.status.value}'
        if wp.id in assigned:
            return False, 'assigned'
        if valid_graph_ids is not None and wp.id not in valid_graph_ids:
            return False, 'not_on_graph'
        if pose is None:
            return False, 'no_pose'
        dist = wp.dist_to(pose.x, pose.y)
        if dist < self._t.min_distance_m:
            return False, f'too_close:{dist:.2f}'
        if dist > self._t.max_distance_m:
            return False, f'too_far:{dist:.2f}'
        if bucket_range is not None:
            lo, hi = bucket_range
            if dist < lo or dist >= hi:
                return False, f'outside_bucket:{_bucket_label(lo, hi)}'
        for c in done:
            if self._t.min_spacing_m > 0.0 and self._t.spacing_vs_collected:
                if wp.dist_to(c.x, c.y) < self._t.min_spacing_m:
                    return False, f'near_done:{c.id}'
        peer_gap = 0.0 if relax_peer else self._t.min_peer_spacing_m
        if peer_gap > 0.0:
            for peer in peer_poses or ():
                d = wp.dist_to(peer.x, peer.y)
                if d < peer_gap:
                    return False, f'near_peer:{d:.2f}'
            for target in peer_targets or ():
                d = wp.dist_to(target.x, target.y)
                if d < peer_gap:
                    return False, f'near_peer_goal:{target.id}'
        if grid is not None:
            if self._f.enforce_map_bounds and not self.is_planner_reachable(wp.x, wp.y, grid):
                return False, 'outside_map'
            if self._f.require_navigable:
                on_graph = (
                    valid_graph_ids is not None and wp.id in valid_graph_ids)
                if not on_graph:
                    reason = self._occupancy_reason(wp.x, wp.y, grid)
                    if reason:
                        return False, reason
        elif self._f.enforce_map_bounds or self._f.require_navigable:
            return False, 'no_map'
        return True, ''

    def select(
        self,
        waypoints: list[Waypoint],
        pose: Pose | None,
        done: Iterable[Waypoint],
        assigned: set[str],
        grid: OccupancyGrid | None,
        *,
        valid_graph_ids: set[str] | None = None,
        peer_poses: Iterable[Pose] | None = None,
        peer_targets: Iterable[Waypoint] | None = None,
    ) -> Waypoint | None:
        wp, _ = self.select_with_reason(
            waypoints,
            pose,
            done,
            assigned,
            grid,
            valid_graph_ids=valid_graph_ids,
            peer_poses=peer_poses,
            peer_targets=peer_targets,
        )
        return wp

    def select_with_reason(
        self,
        waypoints: list[Waypoint],
        pose: Pose | None,
        done: Iterable[Waypoint],
        assigned: set[str],
        grid: OccupancyGrid | None,
        *,
        valid_graph_ids: set[str] | None = None,
        peer_poses: Iterable[Pose] | None = None,
        peer_targets: Iterable[Waypoint] | None = None,
        bucket_counts: dict[str, int] | None = None,
        relax_peer: bool = False,
    ) -> tuple[Waypoint | None, str]:
        bucket_ranges: list[tuple[float, float] | None]
        if self._t.assignment_strategy == 'bucket' and self._t.distance_buckets:
            bucket_ranges = self._bucket_try_order(bucket_counts or {})
        else:
            bucket_ranges = [None]

        merged_reasons: dict[str, int] = {}
        for bucket_range in bucket_ranges:
            wp, reasons = self._pick_best(
                waypoints,
                pose,
                done,
                assigned,
                grid,
                valid_graph_ids=valid_graph_ids,
                peer_poses=peer_poses,
                peer_targets=peer_targets,
                bucket_range=bucket_range,
                relax_peer=relax_peer,
            )
            for key, count in reasons.items():
                merged_reasons[key] = merged_reasons.get(key, 0) + count
            if wp is not None:
                return wp, ''

        if bucket_ranges != [None]:
            wp, reasons = self._pick_best(
                waypoints,
                pose,
                done,
                assigned,
                grid,
                valid_graph_ids=valid_graph_ids,
                peer_poses=peer_poses,
                peer_targets=peer_targets,
                bucket_range=None,
                relax_peer=relax_peer,
            )
            for key, count in reasons.items():
                merged_reasons[key] = merged_reasons.get(key, 0) + count
            if wp is not None:
                return wp, 'bucket_fallback'

        if pose is None:
            return None, 'no_pose'
        return None, _summarize_reasons(merged_reasons)

    def select_reposition(
        self,
        waypoints: list[Waypoint],
        pose: Pose | None,
        done: Iterable[Waypoint],
        assigned: set[str],
        grid: OccupancyGrid | None,
        *,
        valid_graph_ids: set[str] | None = None,
        peer_poses: Iterable[Pose] | None = None,
        peer_targets: Iterable[Waypoint] | None = None,
    ) -> tuple[Waypoint | None, str]:
        """Pick the farthest reachable waypoint to spread idle robots through the scene."""
        if pose is None:
            return None, 'no_pose'
        best: tuple[float, Waypoint] | None = None
        reasons: dict[str, int] = {}
        for wp in waypoints:
            ok, reason = self.can_assign(
                wp,
                pose,
                done,
                assigned,
                grid,
                valid_graph_ids=valid_graph_ids,
                peer_poses=peer_poses,
                peer_targets=peer_targets,
                bucket_range=None,
                relax_peer=True,
            )
            if wp.status == Status.PENDING:
                key = reason or 'eligible'
                reasons[key] = reasons.get(key, 0) + 1
            if ok:
                dist = wp.dist_to(pose.x, pose.y)
                if best is None or dist > best[0]:
                    best = (dist, wp)
        if best is not None:
            return best[1], 'reposition'
        return None, _summarize_reasons(reasons)

    def _pick_best(
        self,
        waypoints: list[Waypoint],
        pose: Pose | None,
        done: Iterable[Waypoint],
        assigned: set[str],
        grid: OccupancyGrid | None,
        *,
        valid_graph_ids: set[str] | None = None,
        peer_poses: Iterable[Pose] | None = None,
        peer_targets: Iterable[Waypoint] | None = None,
        bucket_range: tuple[float, float] | None = None,
        relax_peer: bool = False,
    ) -> tuple[Waypoint | None, dict[str, int]]:
        reasons: dict[str, int] = {}
        best: tuple[float, Waypoint] | None = None
        for wp in waypoints:
            ok, reason = self.can_assign(
                wp,
                pose,
                done,
                assigned,
                grid,
                valid_graph_ids=valid_graph_ids,
                peer_poses=peer_poses,
                peer_targets=peer_targets,
                bucket_range=bucket_range,
                relax_peer=relax_peer,
            )
            if wp.status == Status.PENDING:
                key = reason or 'eligible'
                reasons[key] = reasons.get(key, 0) + 1
            if ok and pose is not None:
                score = self._assignment_score(
                    wp, pose, peer_poses=peer_poses, peer_targets=peer_targets)
                if best is None or score > best[0]:
                    best = (score, wp)
        if best is None:
            return None, reasons
        return best[1], reasons

    def _bucket_try_order(
        self, bucket_counts: dict[str, int],
    ) -> list[tuple[float, float]]:
        """Buckets with the largest deficit vs per-bucket target, tried first."""
        target = self._t.per_bucket_target()
        scored: list[tuple[float, tuple[float, float]]] = []
        for lo, hi in self._t.distance_buckets:
            label = _bucket_label(lo, hi)
            deficit = target - bucket_counts.get(label, 0) if target > 0 else 0.0
            scored.append((deficit, (lo, hi)))
        scored.sort(key=lambda item: item[0], reverse=True)
        return [rng for _, rng in scored]

    def _assignment_score(
        self,
        wp: Waypoint,
        pose: Pose,
        *,
        peer_poses: Iterable[Pose] | None = None,
        peer_targets: Iterable[Waypoint] | None = None,
    ) -> float:
        d_self = wp.dist_to(pose.x, pose.y)
        strategy = self._t.assignment_strategy
        if strategy == 'nearest':
            return -d_self
        if strategy in ('dispersed', 'bucket'):
            peer_sep = float('inf')
            for peer in peer_poses or ():
                peer_sep = min(peer_sep, wp.dist_to(peer.x, peer.y))
            for target in peer_targets or ():
                peer_sep = min(peer_sep, wp.dist_to(target.x, target.y))
            if not math.isfinite(peer_sep):
                peer_sep = d_self
            return peer_sep / max(d_self, 1.0)
        return d_self

    def _effective_margin(self) -> float:
        return max(0.0, self._f.map_margin_m) + max(0.0, self._f.planner_margin_m)

    def is_planner_reachable(self, x: float, y: float, grid: OccupancyGrid) -> bool:
        """True when the pose is inside the map with room for Nav2 path search."""
        return self._inside_map_bounds(x, y, grid, margin=self._effective_margin())

    def _inside_map_bounds(
        self, x: float, y: float, grid: OccupancyGrid, *, margin: float | None = None,
    ) -> bool:
        res = grid.info.resolution
        if res <= 0.0:
            return False
        margin = max(0.0, self._f.map_margin_m if margin is None else margin)
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        max_x = ox + grid.info.width * res
        max_y = oy + grid.info.height * res
        if not (ox + margin <= x < max_x - margin and oy + margin <= y < max_y - margin):
            return False
        return _map_cell(grid, x, y) is not None

    def clamp_to_map(
        self, x: float, y: float, grid: OccupancyGrid | None,
    ) -> tuple[float, float] | None:
        """Snap a pose onto the map interior; None if it cannot be made valid."""
        if grid is None or not self._f.enforce_map_bounds:
            return x, y
        margin = self._effective_margin()
        res = grid.info.resolution
        if res <= 0.0:
            return None
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        min_x = ox + margin
        min_y = oy + margin
        max_x = ox + grid.info.width * res - margin
        max_y = oy + grid.info.height * res - margin
        if max_x <= min_x or max_y <= min_y:
            return None
        cx = min(max(x, min_x), max_x)
        cy = min(max(y, min_y), max_y)
        if _map_cell(grid, cx, cy) is None:
            return None
        return cx, cy

    def _occupancy_reason(self, x: float, y: float, grid: OccupancyGrid) -> str:
        cell = _map_cell(grid, x, y)
        if cell is None:
            return 'outside_map'
        if cell >= self._f.occupied_threshold:
            return f'occupied:{cell}'
        if cell < self._f.free_threshold:
            return f'unknown:{cell}'
        return ''

    def _map_reason(self, x: float, y: float, grid: OccupancyGrid | None) -> str:
        if grid is None:
            return 'no_map'
        if self._f.enforce_map_bounds and not self.is_planner_reachable(x, y, grid):
            return 'outside_map'
        if not self._f.require_navigable:
            return ''
        return self._occupancy_reason(x, y, grid)
