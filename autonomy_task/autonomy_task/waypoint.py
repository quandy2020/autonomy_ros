"""Waypoint models."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class Status(str, Enum):
    PENDING = 'pending'
    IN_PROGRESS = 'in_progress'
    COLLECTED = 'collected'
    FAILED = 'failed'
    SKIPPED = 'skipped'
    BLOCKED = 'blocked'


@dataclass
class Waypoint:
    id: str
    x: float
    y: float
    yaw: float = 0.0
    z: float = 0.0
    label: str = ''
    status: Status = Status.PENDING
    skip_reason: str = ''
    robot: str = ''
    attempts: int = 0
    collector: str = ''
    meta: dict[str, Any] = field(default_factory=dict)

    def dist_to(self, x: float, y: float) -> float:
        return math.hypot(self.x - x, self.y - y)

    def as_dict(self) -> dict[str, Any]:
        return {
            'id': self.id,
            'x': self.x,
            'y': self.y,
            'z': self.z,
            'yaw': self.yaw,
            'label': self.label,
            'status': self.status.value,
            'skip_reason': self.skip_reason,
            'robot': self.robot,
            'attempts': self.attempts,
            'collector': self.collector,
            'meta': self.meta,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Waypoint:
        return cls(
            id=str(data['id']),
            x=float(data['x']),
            y=float(data['y']),
            yaw=float(data.get('yaw', 0.0)),
            z=float(data.get('z', 0.0)),
            label=str(data.get('label', '')),
            status=Status(data.get('status', Status.PENDING.value)),
            skip_reason=str(data.get('skip_reason', '')),
            robot=str(data.get('robot', data.get('assigned_robot', ''))),
            attempts=int(data.get('attempts', 0)),
            collector=str(data.get('collector', data.get('collected_by', ''))),
            meta=dict(data.get('meta', data.get('metadata', {}))),
        )
