"""Waypoint filter tests."""

from nav_msgs.msg import OccupancyGrid

from autonomy_task.config import FilterConfig, Thresholds
from autonomy_task.waypoint import Status, Waypoint
from autonomy_task.waypoint_filter import Filter, Pose


def _grid(width: int = 322, height: int = 166, res: float = 0.05) -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.info.resolution = res
    msg.info.width = width
    msg.info.height = height
    msg.info.origin.position.x = -8.05
    msg.info.origin.position.y = -4.15
    msg.data = [0] * (width * height)
    return msg


def _filter(**filter_kw) -> Filter:
    return Filter(
        Thresholds(min_distance_m=1.0, max_distance_m=10.0, min_spacing_m=1.5, dedupe_radius_m=0.3),
        FilterConfig(**filter_kw),
    )


def test_dedupe() -> None:
    f = _filter()
    wps = [Waypoint(id='a', x=0, y=0), Waypoint(id='b', x=0.1, y=0.1), Waypoint(id='c', x=3, y=3)]
    f.dedupe(wps)
    assert wps[0].status == Status.PENDING
    assert wps[1].status == Status.SKIPPED
    assert wps[2].status == Status.PENDING


def test_select_distance() -> None:
    f = _filter()
    wps = [
        Waypoint(id='near', x=0.5, y=0),
        Waypoint(id='ok', x=3.0, y=0),
        Waypoint(id='far', x=20.0, y=0),
    ]
    picked = f.select(wps, Pose(0, 0), [], set(), None)
    assert picked and picked.id == 'ok'


def test_near_collected() -> None:
    f = _filter()
    wp = Waypoint(id='c', x=2.0, y=0.0)
    done = [Waypoint(id='d', x=2.5, y=0.0, status=Status.COLLECTED)]
    ok, reason = f.can_assign(wp, Pose(0, 0), done, set(), None)
    assert not ok
    assert reason.startswith('near_done')


def test_graph_membership() -> None:
    f = _filter()
    wp = Waypoint(id='g_1', x=3.0, y=0.0)
    ok, reason = f.can_assign(
        wp, Pose(0, 0), [], set(), None, valid_graph_ids={'g_2'})
    assert not ok
    assert reason == 'not_on_graph'
    ok, _ = f.can_assign(
        wp, Pose(0, 0), [], set(), None, valid_graph_ids={'g_1', 'g_2'})
    assert ok


def test_select_farthest() -> None:
    f = Filter(
        Thresholds(
            min_distance_m=1.0,
            max_distance_m=10.0,
            min_spacing_m=1.0,
            assignment_strategy='farthest',
        ),
        FilterConfig(),
    )
    wps = [
        Waypoint(id='near', x=2.0, y=0),
        Waypoint(id='far', x=8.0, y=0),
    ]
    picked = f.select(wps, Pose(0, 0), [], set(), None)
    assert picked and picked.id == 'far'


def test_outside_map_bounds() -> None:
    grid = _grid()
    f = _filter(enforce_map_bounds=True, map_margin_m=0.25, require_navigable=False)
    # East edge: origin.x + width*res = -8.05 + 16.1 = 8.05
    edge_x = grid.info.origin.position.x + grid.info.width * grid.info.resolution - 0.1
    wp = Waypoint(id='edge', x=edge_x, y=0.0)
    ok, reason = f.can_assign(wp, Pose(0, 0), [], set(), grid, valid_graph_ids={'edge'})
    assert not ok
    assert reason == 'outside_map'

    center_x = grid.info.origin.position.x + grid.info.width * grid.info.resolution / 2
    wp_ok = Waypoint(id='mid', x=center_x, y=0.0)
    ok, reason = f.can_assign(wp_ok, Pose(0, 0), [], set(), grid, valid_graph_ids={'mid'})
    assert ok


def test_planner_margin_rejects_near_north_edge() -> None:
    grid = _grid()
    f = _filter(
        enforce_map_bounds=True,
        map_margin_m=0.5,
        planner_margin_m=1.0,
        require_navigable=False,
    )
    # North edge y ≈ 3.65 with map_margin only; y=4.0 is inside bbox but too close for planner.
    north_y = grid.info.origin.position.y + grid.info.height * grid.info.resolution - 0.55
    wp = Waypoint(id='north', x=0.0, y=north_y)
    ok, reason = f.can_assign(wp, Pose(0, 0), [], set(), grid, valid_graph_ids={'north'})
    assert not ok
    assert reason == 'outside_map'


def test_dispersed_balances_peer_sep_and_travel() -> None:
    f = Filter(
        Thresholds(
            min_distance_m=1.0,
            max_distance_m=50.0,
            min_spacing_m=1.0,
            min_peer_spacing_m=3.0,
            assignment_strategy='dispersed',
        ),
        FilterConfig(enforce_map_bounds=False),
    )
    origin = Pose(0, 0)
    peer_target = Waypoint(id='peer_wp', x=10.0, y=0.0)
    wps = [
        Waypoint(id='edge_far', x=-8.0, y=5.0),
        Waypoint(id='mid', x=3.0, y=1.0),
    ]
    picked = f.select(
        wps,
        origin,
        [],
        set(),
        None,
        peer_poses=[origin],
        peer_targets=[peer_target],
    )
    assert picked and picked.id == 'mid'


def test_dispersed_avoids_peer_goal() -> None:
    f = Filter(
        Thresholds(
            min_distance_m=1.0,
            max_distance_m=50.0,
            min_spacing_m=1.0,
            min_peer_spacing_m=3.0,
            assignment_strategy='dispersed',
        ),
        FilterConfig(enforce_map_bounds=False),
    )
    origin = Pose(0, 0)
    peer_target = Waypoint(id='peer_wp', x=10.0, y=0.0)
    wps = [
        Waypoint(id='near_peer', x=10.2, y=0.0),
        Waypoint(id='far_side', x=-8.0, y=5.0),
    ]
    picked = f.select(
        wps,
        origin,
        [],
        set(),
        None,
        peer_poses=[origin],
        peer_targets=[peer_target],
    )
    assert picked and picked.id == 'far_side'


def test_near_peer_goal_rejected() -> None:
    f = Filter(
        Thresholds(min_distance_m=1.0, max_distance_m=50.0, min_peer_spacing_m=3.0),
        FilterConfig(),
    )
    wp = Waypoint(id='close', x=10.5, y=0.0)
    peer_target = Waypoint(id='peer', x=10.0, y=0.0)
    ok, reason = f.can_assign(
        wp, Pose(0, 0), [], set(), None, peer_targets=[peer_target])
    assert not ok
    assert reason.startswith('near_peer_goal')


def test_apply_state_skipped() -> None:
    f = _filter()
    wps = [
        Waypoint(id='done', x=0, y=0),
        Waypoint(id='bad', x=1, y=0),
        Waypoint(id='retry', x=2, y=0),
    ]
    f.apply_state(
        wps,
        collected={'done'},
        failed={'retry': 3},
        skipped={'bad': 'nav_unreachable'},
    )
    assert wps[0].status == Status.COLLECTED
    assert wps[1].status == Status.SKIPPED
    assert wps[1].skip_reason == 'nav_unreachable'
    assert wps[2].status == Status.FAILED


def test_skipped_not_assignable() -> None:
    f = _filter()
    wp = Waypoint(id='bad', x=3.0, y=0.0, status=Status.SKIPPED, skip_reason='nav_unreachable')
    ok, reason = f.can_assign(wp, Pose(0, 0), [], set(), None)
    assert not ok
    assert reason == 'status:skipped'


def test_failed_not_assignable() -> None:
    f = _filter()
    wp = Waypoint(id='bad', x=3.0, y=0.0, status=Status.FAILED)
    ok, reason = f.can_assign(wp, Pose(0, 0), [], set(), None)
    assert not ok
    assert reason == 'status:failed'


def test_bucket_assignment() -> None:
    f = Filter(
        Thresholds(
            min_distance_m=0.5,
            max_distance_m=20.0,
            min_spacing_m=1.0,
            spacing_vs_collected=False,
            assignment_strategy='bucket',
            target_episodes=8,
            distance_buckets=[(1.0, 2.0), (2.0, 3.0)],
        ),
        FilterConfig(enforce_map_bounds=False),
    )
    wps = [
        Waypoint(id='near', x=1.5, y=0.0),
        Waypoint(id='mid', x=2.5, y=0.0),
    ]
    picked, _ = f.select_with_reason(
        wps, Pose(0, 0), [], set(), None, bucket_counts={'1-2': 0, '2-3': 0})
    assert picked and picked.id == 'near'

    picked, _ = f.select_with_reason(
        wps, Pose(0, 0), [], set(), None, bucket_counts={'1-2': 4, '2-3': 0})
    assert picked and picked.id == 'mid'


def test_bucket_fallback_when_priority_bucket_empty() -> None:
    f = Filter(
        Thresholds(
            min_distance_m=0.5,
            max_distance_m=20.0,
            min_spacing_m=1.0,
            spacing_vs_collected=False,
            assignment_strategy='bucket',
            target_episodes=8,
            distance_buckets=[(1.0, 2.0), (5.0, 7.0)],
        ),
        FilterConfig(enforce_map_bounds=False),
    )
    # 3.5 m is outside both configured buckets but within global distance limits.
    wps = [Waypoint(id='mid', x=3.5, y=0.0)]
    picked, reason = f.select_with_reason(
        wps, Pose(0, 0), [], set(), None, bucket_counts={'1-2': 0, '5-7': 0})
    assert picked and picked.id == 'mid'
    assert reason == 'bucket_fallback'


def test_relax_peer_spacing() -> None:
    f = Filter(
        Thresholds(min_distance_m=1.0, max_distance_m=50.0, min_peer_spacing_m=3.0),
        FilterConfig(enforce_map_bounds=False),
    )
    wp = Waypoint(id='close', x=10.5, y=0.0)
    peer_target = Waypoint(id='peer', x=10.0, y=0.0)
    ok, reason = f.can_assign(
        wp, Pose(0, 0), [], set(), None, peer_targets=[peer_target])
    assert not ok
    assert reason.startswith('near_peer_goal')
    ok, _ = f.can_assign(
        wp, Pose(0, 0), [], set(), None, peer_targets=[peer_target], relax_peer=True)
    assert ok


def test_no_waypoint_reason_summary() -> None:
    f = Filter(
        Thresholds(min_distance_m=1.0, max_distance_m=5.0),
        FilterConfig(),
    )
    wps = [
        Waypoint(id='near', x=0.2, y=0.0),
        Waypoint(id='far', x=20.0, y=0.0),
    ]
    picked, reason = f.select_with_reason(wps, Pose(0, 0), [], set(), None)
    assert picked is None
    assert 'too_close' in reason
    assert 'too_far' in reason


def test_select_reposition_picks_farthest() -> None:
    f = Filter(
        Thresholds(min_distance_m=1.0, max_distance_m=50.0, min_peer_spacing_m=3.0),
        FilterConfig(enforce_map_bounds=False),
    )
    wps = [
        Waypoint(id='near', x=3.0, y=0.0),
        Waypoint(id='far', x=12.0, y=0.0),
    ]
    picked, reason = f.select_reposition(wps, Pose(0, 0), [], set(), None)
    assert picked and picked.id == 'far'
    assert reason == 'reposition'


def test_spacing_vs_collected_disabled() -> None:
    f = Filter(
        Thresholds(min_distance_m=0.5, max_distance_m=10.0, min_spacing_m=2.0, spacing_vs_collected=False),
        FilterConfig(enforce_map_bounds=False),
    )
    wp = Waypoint(id='c', x=2.0, y=0.0)
    done = [Waypoint(id='d', x=2.1, y=0.0)]
    ok, _ = f.can_assign(wp, Pose(0, 0), done, set(), None)
    assert ok


def test_prefilter_skips_outside_map() -> None:
    grid = _grid()
    f = _filter(enforce_map_bounds=True, map_margin_m=0.25, require_navigable=False)
    edge_x = grid.info.origin.position.x + grid.info.width * grid.info.resolution - 0.1
    wps = [Waypoint(id='edge', x=edge_x, y=0.0)]
    f.prefilter(wps, grid)
    assert wps[0].status == Status.SKIPPED
    assert wps[0].skip_reason == 'outside_map'
