"""Per-robot graph and costmap context for waypoint selection."""

from __future__ import annotations

from dataclasses import dataclass, field

from nav_msgs.msg import OccupancyGrid


@dataclass
class RobotContext:
    """Latest graph/costmap data for one robot namespace."""

    graph_waypoint_ids: set[str] = field(default_factory=set)
    graph_ready: bool = False
    goal_frame: str = ''
    grid: OccupancyGrid | None = None

    def graph_ids(self) -> set[str] | None:
        """None if graph unused; empty set blocks assignment until graph arrives."""
        if not self.graph_ready:
            return set()
        return self.graph_waypoint_ids
