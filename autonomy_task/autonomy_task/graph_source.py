"""Convert autonomy_msgs/Graph to collection waypoints."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from autonomy_task.waypoint import Waypoint

if TYPE_CHECKING:
    from autonomy_msgs.msg import Graph


def _yaw_to(ax: float, ay: float, bx: float, by: float) -> float:
    return math.atan2(by - ay, bx - ax)


def _node_yaw(msg: Graph, node_id: int, pos) -> float:
    positions = {n.id: n.position for n in msg.nodes}
    for edge in msg.edges:
        if edge.from_id == node_id and edge.to_id in positions:
            p = positions[edge.to_id]
            return _yaw_to(pos.x, pos.y, p.x, p.y)
        if edge.to_id == node_id and edge.from_id in positions:
            p = positions[edge.from_id]
            return _yaw_to(pos.x, pos.y, p.x, p.y)
    return 0.0


def from_graph(msg: Graph, max_nodes: int = 0) -> list[Waypoint]:
    """Build waypoints from Graph.nodes (navmesh / visibility graph)."""
    nodes = msg.nodes
    if max_nodes > 0:
        nodes = nodes[:max_nodes]
    out: list[Waypoint] = []
    for node in nodes:
        wp_id = f'g_{node.id}'
        out.append(Waypoint(
            id=wp_id,
            x=float(node.position.x),
            y=float(node.position.y),
            z=float(node.position.z),
            yaw=_node_yaw(msg, node.id, node.position),
            label=wp_id,
            meta={
                'graph_node_id': int(node.id),
                'flags': int(node.flags),
                'graph_kind': int(msg.kind),
            },
        ))
    return out


def merge_waypoints(current: list[Waypoint], new: list[Waypoint]) -> list[Waypoint]:
    """Keep collection status when graph is refreshed."""
    old = {wp.id: wp for wp in current}
    out: list[Waypoint] = []
    for wp in new:
        prev = old.get(wp.id)
        if prev is None:
            out.append(wp)
            continue
        wp.status = prev.status
        wp.skip_reason = prev.skip_reason
        wp.attempts = prev.attempts
        wp.collector = prev.collector
        wp.robot = prev.robot
        out.append(wp)
    return out
