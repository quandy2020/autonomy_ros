# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Tests for shared episode split logic."""

from __future__ import annotations

import unittest
from types import SimpleNamespace

from autonomy_lerobot.collection_params import (
    JDROBOT_EPISODE_TIMING,
    jdrobot_max_episode_seconds,
)
from autonomy_lerobot.episode_split import EpisodeSplitConfig, assign_episode_indices


def _odom(x: float, y: float) -> SimpleNamespace:
    return SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )
    )


class EpisodeSplitTest(unittest.TestCase):
    def test_max_episode_seconds_matches_collection_task(self) -> None:
        expected = (
            JDROBOT_EPISODE_TIMING['record_before_sec']
            + JDROBOT_EPISODE_TIMING['max_nav_sec']
            + JDROBOT_EPISODE_TIMING['record_after_sec']
        )
        self.assertAlmostEqual(jdrobot_max_episode_seconds(), expected)
        self.assertAlmostEqual(jdrobot_max_episode_seconds(), 41.5)

    def test_fixed_mode_splits_on_duration(self) -> None:
        cfg = EpisodeSplitConfig(mode='fixed', episode_seconds=2.0)
        timestamps = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5]
        odoms = [_odom(0.0, 0.0)] * len(timestamps)
        indices = assign_episode_indices(timestamps, odoms, cfg)
        self.assertEqual(indices, [0, 0, 0, 0, 1, 1])

    def test_nav_mode_detects_motion_segment(self) -> None:
        cfg = EpisodeSplitConfig(
            mode='nav',
            record_before_sec=0.5,
            record_after_sec=1.0,
            max_nav_sec=10.0,
            stall_move_m=0.3,
        )
        timestamps = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5]
        odoms = [
            _odom(0.0, 0.0),
            _odom(0.0, 0.0),
            _odom(1.0, 0.0),
            _odom(2.0, 0.0),
            _odom(3.0, 0.0),
            _odom(3.0, 0.0),
            _odom(3.0, 0.0),
            _odom(3.0, 0.0),
        ]
        indices = assign_episode_indices(timestamps, odoms, cfg)
        self.assertEqual(indices[0], None)
        self.assertEqual(indices[1], 0)
        self.assertEqual(indices[2], 0)
        self.assertEqual(set(indices[3:7]), {0})
        self.assertEqual(indices[7], None)

    def test_nav_mode_includes_pre_record_window(self) -> None:
        cfg = EpisodeSplitConfig(
            mode='nav',
            record_before_sec=1.0,
            record_after_sec=0.5,
            stall_move_m=0.3,
        )
        timestamps = [0.0, 0.5, 1.0, 1.5, 2.0]
        odoms = [
            _odom(0.0, 0.0),
            _odom(0.0, 0.0),
            _odom(1.0, 0.0),
            _odom(1.0, 0.0),
            _odom(1.0, 0.0),
        ]
        indices = assign_episode_indices(timestamps, odoms, cfg)
        self.assertEqual(indices[0], 0)
        self.assertEqual(indices[1], 0)


if __name__ == '__main__':
    unittest.main()
