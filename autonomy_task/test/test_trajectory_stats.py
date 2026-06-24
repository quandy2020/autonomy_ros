"""Trajectory statistics tests."""

from autonomy_task.state import Record, State
from autonomy_task.trajectory_stats import summarize_records


def test_summarize_records() -> None:
    records = [
        Record('g_1', 'robot1', 't1', 1.0, 2.0, 0.0, 10.5, 8.0, 12.0),
        Record('g_2', 'robot2', 't2', 3.0, 4.0, 0.0, 5.5, 4.0, 6.0),
        Record('g_3', 'robot1', 't3', 5.0, 6.0, 0.0, 7.0, 6.5, 8.0),
    ]
    summary = summarize_records(records)
    assert summary['episode_count'] == 3
    assert summary['total_path_m'] == 23.0
    assert summary['per_robot']['robot1']['count'] == 2
    assert summary['per_robot']['robot2']['count'] == 1
    assert len(summary['episodes']) == 3


def test_state_trajectory_summary() -> None:
    state = State()
    state.collected.append(
        Record('g_1', 'robot1', 't', 0, 0, 0, 3.0, 2.0, 4.0))
    traj = state.trajectory_summary()
    assert traj['episode_count'] == 1
    assert traj['episodes'][0]['path_length_m'] == 3.0
