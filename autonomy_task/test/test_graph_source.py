"""Graph waypoint source tests."""

from types import SimpleNamespace

from autonomy_task.graph_source import from_graph, merge_waypoints
from autonomy_task.waypoint import Status, Waypoint


def _graph() -> SimpleNamespace:
    return SimpleNamespace(
        kind=1,
        header=SimpleNamespace(frame_id='map'),
        nodes=[
            SimpleNamespace(id=0, position=SimpleNamespace(x=0.0, y=0.0, z=0.0), flags=0),
            SimpleNamespace(id=1, position=SimpleNamespace(x=3.0, y=0.0, z=0.0), flags=0),
        ],
        edges=[
            SimpleNamespace(from_id=0, to_id=1),
        ],
    )


def test_from_graph() -> None:
    wps = from_graph(_graph())
    assert len(wps) == 2
    assert wps[0].id == 'g_0'
    assert wps[0].x == 0.0
    assert wps[1].id == 'g_1'
    assert wps[0].yaw == 0.0


def test_merge_preserves_status() -> None:
    old = [Waypoint(id='g_0', x=0, y=0, status=Status.COLLECTED, collector='robot1')]
    new = from_graph(_graph())
    merged = merge_waypoints(old, new)
    assert merged[0].status == Status.COLLECTED
    assert merged[0].collector == 'robot1'
    assert merged[1].status == Status.PENDING
