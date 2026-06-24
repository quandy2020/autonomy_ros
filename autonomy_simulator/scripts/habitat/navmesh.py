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

"""Nav mesh topology graph for RViz (autonomy_msgs/Graph on habitat/graph)."""

from __future__ import annotations

import math
from collections import defaultdict

import rclpy
from autonomy_msgs.msg import Graph, GraphEdge, GraphFace, GraphNode
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_system_default,
)
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from habitat.config import Config
from habitat.sim import Session


class NavMeshPublisher:
    """Publish PathFinder navigability as a Graph for RViz."""

    _RED = ColorRGBA(r=0.95, g=0.15, b=0.15, a=1.0)
    _GREEN = ColorRGBA(r=0.15, g=0.9, b=0.25, a=1.0)
    _FACE = ColorRGBA(r=0.15, g=0.9, b=0.25, a=1.0)
    _MERGE_NORMAL_DOT = 0.95
    _EDGE_PRECISION = 1e-4

    def __init__(self, node: Node, cfg: Config, session: Session) -> None:
        self._cfg = cfg
        self._graph: Graph | None = None
        self._mesh_markers: MarkerArray | None = None
        self._pub = node.create_publisher(
            Graph, cfg.navmesh_topic, self._qos(cfg.navmesh_rate_hz))
        self._mesh_pub = None
        if cfg.navmesh_show_mesh:
            self._mesh_pub = node.create_publisher(
                MarkerArray, cfg.navmesh_mesh_topic, self._qos(cfg.navmesh_rate_hz))

        pathfinder = session.pathfinder
        if not pathfinder.is_loaded:
            node.get_logger().warning('NavMesh not loaded; skip navmesh visualization')
            return

        try:
            self._graph = self._build_graph(pathfinder, session.floor_height)
            if cfg.navmesh_show_mesh:
                self._mesh_markers = self._build_mesh_markers(pathfinder)
        except (RuntimeError, ValueError) as exc:
            node.get_logger().error(f'Failed to build navmesh graph: {exc}')
            return

        node_count = len(self._graph.nodes) if self._graph is not None else 0
        edge_count = len(self._graph.edges) if self._graph is not None else 0
        node.get_logger().info(
            f'NavMesh ({cfg.navmesh_topology}): {node_count} nodes, {edge_count} edges on '
            f'{cfg.navmesh_topic} (frame={cfg.navmesh_frame})')

        if cfg.navmesh_rate_hz <= 0.0:
            self.publish(node.get_clock().now())

    def publish(self, stamp: rclpy.time.Time) -> None:
        if self._graph is not None:
            self._graph.header.stamp = stamp.to_msg()
            self._pub.publish(self._graph)
        if self._mesh_markers is not None and self._mesh_pub is not None:
            for marker in self._mesh_markers.markers:
                marker.header.stamp = stamp.to_msg()
            self._mesh_pub.publish(self._mesh_markers)

    @staticmethod
    def _qos(rate_hz: float) -> QoSProfile:
        if rate_hz > 0.0:
            return qos_profile_system_default
        return QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

    def _build_graph(self, pathfinder, floor_height: float) -> Graph:
        cfg = self._cfg
        z = float(cfg.navmesh_z)

        if cfg.navmesh_topology == 'grid':
            link_shape = GraphEdge.LINK_SHAPE_RECTANGLE
            face_shape = GraphFace.SHAPE_RECTANGLE
            node_points, edge_pairs, face_points = self._collect_grid(
                pathfinder, floor_height, z)
        elif cfg.navmesh_topology == 'polygon':
            link_shape = GraphEdge.LINK_SHAPE_POLYGON
            face_shape = GraphFace.SHAPE_POLYGON
            node_points, edge_pairs, face_points = self._collect_polygons(
                pathfinder, z)
        else:
            link_shape = GraphEdge.LINK_SHAPE_TRIANGLE
            face_shape = GraphFace.SHAPE_TRIANGLE
            node_points, edge_pairs, face_points = self._collect_triangles(
                pathfinder, z)

        graph = Graph()
        graph.header.frame_id = cfg.navmesh_frame
        graph.kind = Graph.KIND_TOPOLOGICAL
        node_scale = float(cfg.navmesh_node_scale)

        if cfg.navmesh_show_nodes:
            for node_id, point in enumerate(node_points):
                graph.nodes.append(GraphNode(
                    id=node_id,
                    position=point,
                    color=self._RED,
                    scale=node_scale,
                ))

        if cfg.navmesh_show_edges:
            for from_id, to_id in edge_pairs:
                graph.edges.append(GraphEdge(
                    from_id=from_id,
                    to_id=to_id,
                    type=GraphEdge.TYPE_UNDIRECTED,
                    link_shape=link_shape,
                    weight=1.0,
                    color=self._GREEN,
                    valid=True,
                ))

        if cfg.navmesh_show_faces:
            face_alpha = float(cfg.navmesh_face_alpha)
            for points in face_points:
                if len(points) < 3:
                    continue
                graph.faces.append(GraphFace(
                    shape=face_shape,
                    points=points,
                    color=self._FACE,
                    alpha=face_alpha,
                ))
        return graph

    @staticmethod
    def _habitat_point(vertices, vertex_id: int, z: float) -> Point:
        vertex = vertices[vertex_id]
        return Point(x=float(vertex[0]), y=-float(vertex[2]), z=z)

    @staticmethod
    def _order_polygon_points(points: list[Point]) -> list[Point]:
        if len(points) < 3:
            return points
        cx = sum(point.x for point in points) / len(points)
        cy = sum(point.y for point in points) / len(points)
        return sorted(points, key=lambda point: math.atan2(point.y - cy, point.x - cx))

    def _load_mesh(
        self,
        pathfinder,
    ) -> tuple[list, list[tuple[int, int, int]], dict[tuple, list[int]]] | None:
        vertices = pathfinder.build_navmesh_vertices()
        indices = pathfinder.build_navmesh_vertex_indices()
        if not vertices or not indices:
            return None

        n_tris = len(indices) // 3
        tri_verts = [
            (indices[3 * tri], indices[3 * tri + 1], indices[3 * tri + 2])
            for tri in range(n_tris)
        ]
        edge_tris: dict[tuple, list[int]] = defaultdict(list)
        for tri, (i0, i1, i2) in enumerate(tri_verts):
            for a, b in ((i0, i1), (i1, i2), (i2, i0)):
                edge_tris[self._spatial_edge_key(vertices, a, b)].append(tri)
        return vertices, tri_verts, edge_tris

    def _collect_triangles(
        self,
        pathfinder,
        z: float,
    ) -> tuple[list[Point], list[tuple[int, int]], list[list[Point]]]:
        """One node per navmesh triangle; edges on shared triangle sides."""
        mesh = self._load_mesh(pathfinder)
        if mesh is None:
            return [], [], []

        vertices, tri_verts, edge_tris = mesh
        n_tris = len(tri_verts)
        limit = min(n_tris, self._cfg.navmesh_max_nodes)

        node_points: list[Point] = []
        face_points: list[list[Point]] = []
        for tri in range(limit):
            habitat_x = habitat_y = habitat_z = 0.0
            corners: list[Point] = []
            for vid in tri_verts[tri]:
                vertex = vertices[vid]
                habitat_x += float(vertex[0])
                habitat_y += float(vertex[1])
                habitat_z += float(vertex[2])
                corners.append(self._habitat_point(vertices, vid, z))
            habitat_x /= 3.0
            habitat_y /= 3.0
            habitat_z /= 3.0
            node_points.append(Point(x=habitat_x, y=-habitat_z, z=z))
            face_points.append(corners)

        edge_pairs: list[tuple[int, int]] = []
        seen: set[tuple[int, int]] = set()
        for tris in edge_tris.values():
            if len(tris) != 2:
                continue
            t0, t1 = tris
            if t0 >= limit or t1 >= limit:
                continue
            key = (min(t0, t1), max(t0, t1))
            if key in seen:
                continue
            seen.add(key)
            edge_pairs.append(key)
        return node_points, edge_pairs, face_points

    def _collect_polygons(
        self,
        pathfinder,
        z: float,
    ) -> tuple[list[Point], list[tuple[int, int]], list[list[Point]]]:
        """Merge coplanar triangles into convex polygons (coarse graph)."""
        mesh = self._load_mesh(pathfinder)
        if mesh is None:
            return [], [], []

        vertices, tri_verts, edge_tris = mesh
        n_tris = len(tri_verts)
        parent = list(range(n_tris))

        def find(tri: int) -> int:
            while parent[tri] != tri:
                parent[tri] = parent[parent[tri]]
                tri = parent[tri]
            return tri

        def union(left: int, right: int) -> None:
            root_left = find(left)
            root_right = find(right)
            if root_left != root_right:
                parent[root_right] = root_left

        for tris in edge_tris.values():
            if len(tris) != 2:
                continue
            t0, t1 = tris
            if self._merge_triangles(vertices, tri_verts[t0], tri_verts[t1]):
                union(t0, t1)

        poly_tris: dict[int, list[int]] = defaultdict(list)
        for tri in range(n_tris):
            poly_tris[find(tri)].append(tri)

        poly_items = sorted(poly_tris.items(), key=lambda item: -len(item[1]))
        if len(poly_items) > self._cfg.navmesh_max_nodes:
            poly_items = poly_items[: self._cfg.navmesh_max_nodes]

        root_to_index: dict[int, int] = {}
        node_points: list[Point] = []
        face_points: list[list[Point]] = []
        for root, tris in poly_items:
            vertex_ids = set()
            for tri in tris:
                vertex_ids.update(tri_verts[tri])
            habitat_x = habitat_y = habitat_z = 0.0
            corners = [self._habitat_point(vertices, vid, z) for vid in vertex_ids]
            for vid in vertex_ids:
                vertex = vertices[vid]
                habitat_x += float(vertex[0])
                habitat_y += float(vertex[1])
                habitat_z += float(vertex[2])
            count = len(vertex_ids)
            habitat_x /= count
            habitat_y /= count
            habitat_z /= count
            root_to_index[root] = len(node_points)
            node_points.append(Point(x=habitat_x, y=-habitat_z, z=z))
            face_points.append(self._order_polygon_points(corners))

        edge_pairs: list[tuple[int, int]] = []
        seen_edges: set[tuple[int, int]] = set()
        for tris in edge_tris.values():
            if len(tris) != 2:
                continue
            root_a = find(tris[0])
            root_b = find(tris[1])
            if root_a == root_b:
                continue
            if root_a not in root_to_index or root_b not in root_to_index:
                continue
            key = (min(root_a, root_b), max(root_a, root_b))
            if key in seen_edges:
                continue
            seen_edges.add(key)
            edge_pairs.append((root_to_index[key[0]], root_to_index[key[1]]))
        return node_points, edge_pairs, face_points

    @classmethod
    def _spatial_vertex_key(cls, vertices, index: int) -> tuple[float, float, float]:
        vertex = vertices[index]
        step = cls._EDGE_PRECISION
        return (
            round(float(vertex[0]) / step) * step,
            round(float(vertex[1]) / step) * step,
            round(float(vertex[2]) / step) * step,
        )

    @classmethod
    def _spatial_edge_key(cls, vertices, a: int, b: int) -> tuple:
        key_a = cls._spatial_vertex_key(vertices, a)
        key_b = cls._spatial_vertex_key(vertices, b)
        return (key_a, key_b) if key_a < key_b else (key_b, key_a)

    def _collect_grid(
        self,
        pathfinder,
        floor_height: float,
        z: float,
    ) -> tuple[list[Point], list[tuple[int, int]], list[list[Point]]]:
        cfg = self._cfg
        mpp = float(cfg.navmesh_meters_per_pixel)
        height = float(cfg.navmesh_height or floor_height)
        eps = float(cfg.navmesh_eps)
        stride = max(1, int(cfg.navmesh_stride))

        navigable = pathfinder.get_topdown_view(mpp, height, eps)
        lo, hi = pathfinder.get_bounds()
        origin_x = min(float(lo[0]), float(hi[0]))
        origin_z = min(float(lo[2]), float(hi[2]))

        node_ids, node_points, cell_corners = self._collect_grid_nodes(
            navigable, origin_x, origin_z, mpp, z, stride)
        edge_pairs = self._collect_grid_edges(node_ids, stride)
        face_points = [
            [
                Point(x=origin_x + col * mpp, y=-(origin_z + row * mpp), z=z),
                Point(x=origin_x + (col + stride) * mpp, y=-(origin_z + row * mpp), z=z),
                Point(x=origin_x + (col + stride) * mpp, y=-(origin_z + (row + stride) * mpp), z=z),
                Point(x=origin_x + col * mpp, y=-(origin_z + (row + stride) * mpp), z=z),
            ]
            for (row, col) in cell_corners
        ]
        return node_points, edge_pairs, face_points

    def _collect_grid_nodes(
        self,
        navigable,
        origin_x: float,
        origin_z: float,
        mpp: float,
        z: float,
        stride: int,
    ) -> tuple[dict[tuple[int, int], int], list[Point], list[tuple[int, int]]]:
        cfg = self._cfg
        rows, cols = navigable.shape
        node_ids: dict[tuple[int, int], int] = {}
        cell_corners: list[tuple[int, int]] = []
        points: list[Point] = []

        for row in range(0, rows, stride):
            for col in range(0, cols, stride):
                if not navigable[row, col] or len(points) >= cfg.navmesh_max_nodes:
                    continue
                map_x = origin_x + (col + 0.5) * mpp
                map_y = -(origin_z + (row + 0.5) * mpp)
                node_ids[(row, col)] = len(points)
                cell_corners.append((row, col))
                points.append(Point(x=map_x, y=map_y, z=z))
            if len(points) >= cfg.navmesh_max_nodes:
                break
        return node_ids, points, cell_corners

    def _collect_grid_edges(
        self,
        node_ids: dict[tuple[int, int], int],
        stride: int,
    ) -> list[tuple[int, int]]:
        cfg = self._cfg
        steps = ((0, stride), (stride, 0))
        if cfg.navmesh_connect_diagonal:
            steps = steps + ((stride, stride), (stride, -stride))

        edge_pairs: list[tuple[int, int]] = []
        for (row, col), index in node_ids.items():
            for drow, dcol in steps:
                neighbor = (row + drow, col + dcol)
                if neighbor not in node_ids:
                    continue
                edge_pairs.append((index, node_ids[neighbor]))
        return edge_pairs

    def _merge_triangles(self, vertices, tri_a: tuple[int, int, int], tri_b) -> bool:
        normal_a = self._triangle_normal(vertices, tri_a)
        normal_b = self._triangle_normal(vertices, tri_b)
        if normal_a is None or normal_b is None:
            return False
        dot = (
            normal_a[0] * normal_b[0]
            + normal_a[1] * normal_b[1]
            + normal_a[2] * normal_b[2]
        )
        return dot >= self._MERGE_NORMAL_DOT

    @staticmethod
    def _triangle_normal(
        vertices,
        tri: tuple[int, int, int],
    ) -> tuple[float, float, float] | None:
        a = vertices[tri[0]]
        b = vertices[tri[1]]
        c = vertices[tri[2]]
        ux = float(b[0]) - float(a[0])
        uy = float(b[1]) - float(a[1])
        uz = float(b[2]) - float(a[2])
        vx = float(c[0]) - float(a[0])
        vy = float(c[1]) - float(a[1])
        vz = float(c[2]) - float(a[2])
        nx = uy * vz - uz * vy
        ny = uz * vx - ux * vz
        nz = ux * vy - uy * vx
        length = math.sqrt(nx * nx + ny * ny + nz * nz)
        if length < 1e-9:
            return None
        return nx / length, ny / length, nz / length

    def _build_mesh_markers(self, pathfinder) -> MarkerArray | None:
        cfg = self._cfg
        z = float(cfg.navmesh_z)
        vertices = pathfinder.build_navmesh_vertices()
        indices = pathfinder.build_navmesh_vertex_indices()
        if not vertices or not indices:
            return None

        points: list[Point] = []
        for tri in range(0, len(indices), 3):
            for corner in range(3):
                a = vertices[indices[tri + corner]]
                b = vertices[indices[tri + (corner + 1) % 3]]
                points.append(Point(x=float(a[0]), y=-float(a[2]), z=z))
                points.append(Point(x=float(b[0]), y=-float(b[2]), z=z))

        msg = Marker()
        msg.header.frame_id = cfg.navmesh_frame
        msg.ns = 'navmesh_mesh_edges'
        msg.type = Marker.LINE_LIST
        msg.action = Marker.ADD
        msg.pose.orientation.w = 1.0
        msg.scale.x = max(0.005, float(cfg.navmesh_line_width) * 0.5)
        msg.color = ColorRGBA(r=0.15, g=0.9, b=0.25, a=0.45)
        msg.points = points
        return MarkerArray(markers=[msg])
