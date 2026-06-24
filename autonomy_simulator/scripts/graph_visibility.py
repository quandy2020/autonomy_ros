#
# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

"""Helpers to build autonomy_msgs/Graph visibility graphs."""

from __future__ import annotations

import math

from autonomy_msgs.msg import Graph, GraphEdge, GraphNode
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


def euclidean_weight(a: Point, b: Point) -> float:
    return math.sqrt(
        (a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def build_visibility_graph(
    frame_id: str,
    nodes: list[tuple[int, Point, int]],
    edges: list[tuple[int, int]],
    *,
    edge_weights: dict[tuple[int, int], float] | None = None,
    z: float = 0.0,
) -> Graph:
    """Build a visibility graph message.

    Args:
        frame_id: TF frame for Graph.header.frame_id.
        nodes: (id, position, flags) tuples. Use GraphNode.FLAG_* constants.
        edges: (from_id, to_id) visibility segments.
        edge_weights: optional weight lookup; defaults to Euclidean distance.
        z: default z when node positions are 2D (y only).
    """
    node_by_id: dict[int, GraphNode] = {}
    for node_id, position, flags in nodes:
        point = Point(x=position.x, y=position.y, z=position.z if position.z else z)
        node_by_id[node_id] = GraphNode(
            id=node_id,
            position=point,
            color=ColorRGBA(),
            scale=0.0,
            flags=flags,
        )

    graph = Graph()
    graph.header.frame_id = frame_id
    graph.kind = Graph.KIND_VISIBILITY
    graph.nodes = list(node_by_id.values())

    for from_id, to_id in edges:
        from_node = node_by_id[from_id]
        to_node = node_by_id[to_id]
        key = (min(from_id, to_id), max(from_id, to_id))
        weight = (
            edge_weights[key]
            if edge_weights is not None and key in edge_weights
            else euclidean_weight(from_node.position, to_node.position)
        )
        graph.edges.append(GraphEdge(
            from_id=from_id,
            to_id=to_id,
            type=GraphEdge.TYPE_UNDIRECTED,
            link_shape=GraphEdge.LINK_SHAPE_LINE,
            weight=weight,
            color=ColorRGBA(r=0.15, g=0.9, b=0.25, a=1.0),
            valid=True,
        ))
    return graph
