"""Persistent task state."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from autonomy_task.waypoint import Status, Waypoint


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


@dataclass
class Record:
    waypoint_id: str
    robot: str
    at: str
    x: float
    y: float
    yaw: float
    path_length_m: float = 0.0
    straight_line_m: float = 0.0
    duration_sec: float = 0.0


@dataclass
class State:
    collected: list[Record] = field(default_factory=list)
    failed: dict[str, int] = field(default_factory=dict)
    skipped: dict[str, str] = field(default_factory=dict)
    updated_at: str = field(default_factory=_now)

    @property
    def collected_ids(self) -> set[str]:
        return {r.waypoint_id for r in self.collected}

    def add_collected(
        self,
        wp: Waypoint,
        robot: str,
        *,
        path_length_m: float = 0.0,
        straight_line_m: float = 0.0,
        duration_sec: float = 0.0,
    ) -> None:
        self.collected.append(Record(
            wp.id, robot, _now(), wp.x, wp.y, wp.yaw,
            path_length_m=path_length_m,
            straight_line_m=straight_line_m,
            duration_sec=duration_sec,
        ))
        self.failed.pop(wp.id, None)
        self.skipped.pop(wp.id, None)
        self.updated_at = _now()

    def add_failed(self, wp_id: str) -> int:
        self.failed[wp_id] = self.failed.get(wp_id, 0) + 1
        self.updated_at = _now()
        return self.failed[wp_id]

    def add_skipped(self, wp_id: str, reason: str) -> None:
        """Persist a waypoint that must not be assigned again."""
        self.skipped[wp_id] = reason
        self.failed.pop(wp_id, None)
        self.updated_at = _now()

    def summary(self, waypoints: list[Waypoint]) -> dict[str, Any]:
        from autonomy_task.trajectory_stats import summarize_records

        counts = {s.value: 0 for s in Status}
        for wp in waypoints:
            counts[wp.status.value] += 1
        traj = summarize_records(self.collected)
        return {
            'updated_at': self.updated_at,
            'total': len(waypoints),
            'counts': counts,
            'collected': len(self.collected),
            'failed': dict(self.failed),
            'skipped': dict(self.skipped),
            'trajectories': traj,
        }

    def trajectory_summary(self) -> dict[str, Any]:
        from autonomy_task.trajectory_stats import summarize_records

        return summarize_records(self.collected)

    def as_dict(self) -> dict[str, Any]:
        return {
            'updated_at': self.updated_at,
            'collected': [
                {
                    'waypoint_id': r.waypoint_id,
                    'robot': r.robot,
                    'collected_at': r.at,
                    'x': r.x, 'y': r.y, 'yaw': r.yaw,
                    'path_length_m': r.path_length_m,
                    'straight_line_m': r.straight_line_m,
                    'duration_sec': r.duration_sec,
                }
                for r in self.collected
            ],
            'failed_attempts': self.failed,
            'skipped': self.skipped,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> State:
        records = []
        for e in data.get('collected', []):
            if isinstance(e, dict):
                records.append(Record(
                    str(e['waypoint_id']),
                    str(e.get('robot', '')),
                    str(e.get('collected_at', e.get('at', ''))),
                    float(e.get('x', 0.0)),
                    float(e.get('y', 0.0)),
                    float(e.get('yaw', 0.0)),
                    path_length_m=float(e.get('path_length_m', 0.0)),
                    straight_line_m=float(e.get('straight_line_m', 0.0)),
                    duration_sec=float(e.get('duration_sec', 0.0)),
                ))
        return cls(
            collected=records,
            failed={str(k): int(v) for k, v in dict(data.get('failed_attempts', data.get('failed', {}))).items()},
            skipped={str(k): str(v) for k, v in dict(data.get('skipped', {})).items()},
            updated_at=str(data.get('updated_at', _now())),
        )


def load(path: str | Path) -> State:
    p = Path(path)
    if not p.exists():
        return State()
    data = json.loads(p.read_text(encoding='utf-8'))
    return State.from_dict(data) if isinstance(data, dict) else State()


def save(path: str | Path, state: State) -> None:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    state.updated_at = _now()
    p.write_text(json.dumps(state.as_dict(), indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
