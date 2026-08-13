"""Goal-distance curriculum adapted from jdrobot navigation curriculums."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class CurriculumStage:
    steps: int
    goal_distance_scale: float


@dataclass
class GoalDistanceCurriculum:
    """Expand goal sampling radius as training progresses."""

    enabled: bool
    stages: list[CurriculumStage]
    base_goal_distance_m: float

    @classmethod
    def from_dict(cls, task_cfg: dict[str, Any]) -> GoalDistanceCurriculum:
        cur = task_cfg.get('curriculum', {})
        stages_raw = cur.get('stages', [
            {'steps': 0, 'goal_distance_scale': 0.35},
            {'steps': 500_000, 'goal_distance_scale': 0.65},
            {'steps': 1_500_000, 'goal_distance_scale': 1.0},
        ])
        stages = [
            CurriculumStage(int(s['steps']), float(s['goal_distance_scale']))
            for s in stages_raw
        ]
        stages.sort(key=lambda item: item.steps)
        return cls(
            enabled=bool(cur.get('enabled', False)),
            stages=stages,
            base_goal_distance_m=float(task_cfg.get('goal_distance_m', 8.0)),
        )

    def goal_radius(self, global_step: int) -> float:
        if not self.enabled or not self.stages:
            return self.base_goal_distance_m * 0.5

        scale = self.stages[0].goal_distance_scale
        for stage in self.stages:
            if global_step >= stage.steps:
                scale = stage.goal_distance_scale
        return self.base_goal_distance_m * 0.5 * scale
