# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Episode boundary detection shared by online collection and offline bag conversion."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from autonomy_lerobot.collection_params import JDROBOT_EPISODE_TIMING


@dataclass(frozen=True)
class EpisodeSplitConfig:
    """Episode timing aligned with ``autonomy_task`` collection flow."""

    mode: str = 'nav'
    record_before_sec: float = JDROBOT_EPISODE_TIMING['record_before_sec']
    record_after_sec: float = JDROBOT_EPISODE_TIMING['record_after_sec']
    max_nav_sec: float = JDROBOT_EPISODE_TIMING['max_nav_sec']
    stall_move_m: float = JDROBOT_EPISODE_TIMING['stall_move_m']
    episode_seconds: float = 0.0

    @property
    def max_episode_seconds(self) -> float:
        return self.record_before_sec + self.max_nav_sec + self.record_after_sec

    @property
    def fixed_slice_seconds(self) -> float:
        if self.episode_seconds > 0.0:
            return min(self.episode_seconds, self.max_episode_seconds)
        return self.max_episode_seconds


def odom_xy(msg: Any) -> tuple[float, float]:
    pose = msg.pose.pose.position
    return float(pose.x), float(pose.y)


def _is_moving(
    dx: float,
    dy: float,
    dt: float,
    *,
    stall_move_m: float,
    speed_threshold: float = 0.05,
) -> bool:
    if dt <= 0.0:
        return False
    dist = math.hypot(dx, dy)
    return dist >= stall_move_m or dist / dt >= speed_threshold


def assign_fixed_episode_indices(
    timestamps: list[float],
    cfg: EpisodeSplitConfig,
) -> list[int]:
    """Split on fixed-duration windows (fallback when nav odom is unavailable)."""
    if not timestamps:
        return []
    slice_sec = cfg.fixed_slice_seconds
    indices: list[int] = []
    episode_index = 0
    episode_start_ts = timestamps[0]
    frames_in_episode = 0
    for ts in timestamps:
        if (
            slice_sec > 0.0
            and frames_in_episode > 0
            and (ts - episode_start_ts) >= slice_sec
        ):
            episode_index += 1
            episode_start_ts = ts
            frames_in_episode = 0
        indices.append(episode_index)
        frames_in_episode += 1
    return indices


def assign_nav_episode_indices(
    timestamps: list[float],
    odom_msgs: list[Any | None],
    cfg: EpisodeSplitConfig,
) -> list[int | None]:
    """Detect nav-like episodes: pre-record + motion + post-record.

    Mirrors ``autonomy_task`` PRE_RECORD → NAV → POST_RECORD. Samples outside any
    detected episode receive ``None`` and should be skipped.
    """
    count = len(timestamps)
    if count == 0:
        return []
    if count != len(odom_msgs):
        raise ValueError('timestamps and odom_msgs length mismatch')

    xs: list[float] = []
    ys: list[float] = []
    valid: list[bool] = []
    for msg in odom_msgs:
        if msg is None:
            valid.append(False)
            xs.append(0.0)
            ys.append(0.0)
            continue
        valid.append(True)
        x, y = odom_xy(msg)
        xs.append(x)
        ys.append(y)

    if not any(valid):
        return assign_fixed_episode_indices(timestamps, cfg)

    moving = [False] * count
    for index in range(1, count):
        if not valid[index] or not valid[index - 1]:
            continue
        dt = max(timestamps[index] - timestamps[index - 1], 1e-6)
        moving[index] = _is_moving(
            xs[index] - xs[index - 1],
            ys[index] - ys[index - 1],
            dt,
            stall_move_m=cfg.stall_move_m,
        )

    result: list[int | None] = [None] * count
    episode_index = 0
    cursor = 0
    while cursor < count:
        while cursor < count and not moving[cursor]:
            cursor += 1
        if cursor >= count:
            break

        motion_start = cursor
        start_ts = timestamps[motion_start] - cfg.record_before_sec
        ep_start = motion_start
        for index in range(count):
            if timestamps[index] >= start_ts:
                ep_start = index
                break

        scan = motion_start
        last_moving = motion_start
        ep_end = motion_start
        while scan < count:
            if moving[scan]:
                last_moving = scan
            elif scan > motion_start:
                stationary_for = timestamps[scan] - timestamps[last_moving]
                if stationary_for >= cfg.record_after_sec:
                    ep_end = scan
                    break
            if timestamps[scan] - timestamps[ep_start] >= cfg.max_episode_seconds:
                ep_end = scan
                break
            scan += 1
        else:
            ep_end = count - 1

        for index in range(ep_start, ep_end + 1):
            result[index] = episode_index
        episode_index += 1
        cursor = ep_end + 1

    return result


def assign_episode_indices(
    timestamps: list[float],
    odom_msgs: list[Any | None],
    cfg: EpisodeSplitConfig,
) -> list[int | None]:
    """Assign episode indices using the configured split mode."""
    if cfg.mode == 'fixed':
        return assign_fixed_episode_indices(timestamps, cfg)
    return assign_nav_episode_indices(timestamps, odom_msgs, cfg)
