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

"""RViz marker helpers for NavDP diffusion trajectories."""

from __future__ import annotations

import numpy as np
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from autonomy_internnav.baselines.navdp.colormap import critic_value_to_rgb


def _color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    return ColorRGBA(r=r, g=g, b=b, a=a)


def _set_lifetime(marker: Marker, lifetime_sec: float) -> None:
    if lifetime_sec > 0.0:
        sec = int(lifetime_sec)
        nsec = int((lifetime_sec - sec) * 1e9)
        marker.lifetime = Duration(sec=sec, nanosec=nsec)


def _trajectory_points_xy(trajectory: np.ndarray) -> list[Point]:
    """NavDP rows are (x, y, yaw); RViz markers need ground-plane (x, y, z=0)."""
    points: list[Point] = []
    for row in np.asarray(trajectory):
        pt = Point()
        pt.x = float(row[0])
        pt.y = float(row[1])
        pt.z = 0.0
        points.append(pt)
    return points


def _line_strip(
    header: Header,
    marker_id: int,
    points: np.ndarray,
    color: ColorRGBA,
    scale: float,
    ns: str,
    lifetime_sec: float,
) -> Marker:
    marker = Marker()
    marker.header = header
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = scale
    marker.color = color
    marker.pose.orientation.w = 1.0
    marker.points = _trajectory_points_xy(points)
    _set_lifetime(marker, lifetime_sec)
    return marker


def _flatten_critic_values(values: np.ndarray | None) -> np.ndarray | None:
    if values is None:
        return None
    arr = np.asarray(values, dtype=np.float64).reshape(-1)
    return arr if arr.size > 0 else None


def build_navdp_markers(
    header: Header,
    selected: np.ndarray,
    candidates: np.ndarray | None = None,
    values: np.ndarray | None = None,
    goal: PoseStamped | None = None,
    lookahead_index: int = 3,
    lifetime_sec: float = 0.5,
) -> MarkerArray:
    """Build RViz markers for NavDP diffusion samples and the critic-selected path.

    Trajectories should be published in the robot body frame (``base_link``) so they
    stay fixed relative to the robot and do not jitter from map/odom reprojection.
    """
    out = MarkerArray()
    critic_values = _flatten_critic_values(values)
    marker_id = 0

    if candidates is not None and candidates.size > 0:
        traj_batch = np.asarray(candidates)
        if traj_batch.ndim == 4:
            traj_batch = traj_batch[0]
        n_samples = len(traj_batch)
        if critic_values is not None and len(critic_values) != n_samples:
            critic_values = critic_values[:n_samples]

        order = np.arange(n_samples)
        if critic_values is not None:
            order = np.argsort(critic_values)

        for idx in order:
            value = critic_values[idx] if critic_values is not None else 0.0
            r, g, b = critic_value_to_rgb(value) if critic_values is not None else (0.2, 0.5, 1.0)
            out.markers.append(_line_strip(
                header,
                marker_id,
                traj_batch[idx],
                _color(r, g, b, 0.5),
                0.02,
                'diffusion_samples',
                lifetime_sec,
            ))
            marker_id += 1

    if selected.size > 0:
        if critic_values is not None:
            best_idx = int(np.argmax(critic_values))
            r, g, b = critic_value_to_rgb(critic_values[best_idx])
        else:
            r, g, b = 0.1, 0.98, 0.2
        out.markers.append(_line_strip(
            header, marker_id, selected, _color(r, g, b, 1.0), 0.08, 'selected',
            lifetime_sec,
        ))
        marker_id += 1

        idx = min(max(lookahead_index, 0), len(selected) - 1)
        target = selected[idx]
        lookahead = Marker()
        lookahead.header = header
        lookahead.ns = 'lookahead'
        lookahead.id = marker_id
        marker_id += 1
        lookahead.type = Marker.SPHERE
        lookahead.action = Marker.ADD
        lookahead.pose.position.x = float(target[0])
        lookahead.pose.position.y = float(target[1])
        lookahead.pose.position.z = 0.1
        lookahead.pose.orientation.w = 1.0
        lookahead.scale.x = lookahead.scale.y = lookahead.scale.z = 0.12
        lookahead.color = _color(1.0, 0.85, 0.1, 1.0)
        _set_lifetime(lookahead, lifetime_sec)
        out.markers.append(lookahead)

    if goal is not None:
        goal_marker = Marker()
        goal_marker.header = goal.header
        goal_marker.ns = 'goal'
        goal_marker.id = marker_id
        marker_id += 1
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose = goal.pose
        goal_marker.pose.position.z = 0.15
        goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.25
        goal_marker.color = _color(0.95, 0.2, 0.2, 0.9)
        _set_lifetime(goal_marker, lifetime_sec)
        out.markers.append(goal_marker)

    return out
