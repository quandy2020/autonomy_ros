# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""RViz marker builders for NavDP diffusion trajectories."""

from __future__ import annotations

import numpy as np
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from autonomy_internnav.navdp.colormap import critic_value_to_rgb


def build_navdp_markers(
    header: Header,
    selected: np.ndarray,
    candidates: np.ndarray | None = None,
    values: np.ndarray | None = None,
    goal: PoseStamped | None = None,
    lookahead_index: int = 3,
    lifetime_sec: float = 0.0,
    show_all_samples: bool = False,
) -> MarkerArray:
    """Build RViz markers for diffusion candidates and the selected trajectory."""
    out = MarkerArray()
    marker_id = 0
    lifetime = None
    if lifetime_sec > 0.0:
        sec, nsec = int(lifetime_sec), int((lifetime_sec % 1) * 1e9)
        lifetime = Duration(sec=sec, nanosec=nsec)

    critic = None
    if values is not None:
        flat = np.asarray(values, dtype=np.float64).reshape(-1)
        critic = flat if flat.size > 0 else None

    if show_all_samples and candidates is not None and np.asarray(candidates).size > 0:
        batch = np.asarray(candidates)
        batch = batch[0] if batch.ndim == 4 else batch
        n = len(batch)
        if critic is not None and len(critic) != n:
            critic = critic[:n]
        for idx in (np.argsort(critic) if critic is not None else np.arange(n)):
            value = float(critic[idx]) if critic is not None else 0.0
            rgb = critic_value_to_rgb(value) if critic is not None else (0.2, 0.5, 1.0)
            line = Marker()
            line.header, line.ns, line.id = header, 'diffusion_samples', marker_id
            line.type, line.action = Marker.LINE_STRIP, Marker.ADD
            line.scale.x, line.color = 0.02, ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=0.5)
            line.pose.orientation.w = 1.0
            line.points = [Point(x=float(r[0]), y=float(r[1]), z=0.0) for r in batch[idx]]
            if lifetime is not None:
                line.lifetime = lifetime
            out.markers.append(line)
            marker_id += 1

    if selected.size == 0:
        return out

    sel = Marker()
    sel.header, sel.ns, sel.id = header, 'selected', marker_id
    sel.type, sel.action = Marker.LINE_STRIP, Marker.ADD
    sel.scale.x = 0.08
    sel.color = ColorRGBA(r=0.1, g=0.98, b=0.2, a=1.0)
    sel.pose.orientation.w = 1.0
    sel.points = [Point(x=float(r[0]), y=float(r[1]), z=0.0) for r in selected]
    if lifetime is not None:
        sel.lifetime = lifetime
    out.markers.append(sel)
    marker_id += 1

    idx = min(max(lookahead_index, 0), len(selected) - 1)
    t = selected[idx]
    look = Marker()
    look.header, look.ns, look.id = header, 'lookahead', marker_id
    look.type, look.action = Marker.SPHERE, Marker.ADD
    look.pose.position.x, look.pose.position.y, look.pose.position.z = float(t[0]), float(t[1]), 0.1
    look.pose.orientation.w = 1.0
    look.scale.x = look.scale.y = look.scale.z = 0.12
    look.color = ColorRGBA(r=1.0, g=0.85, b=0.1, a=1.0)
    if lifetime is not None:
        look.lifetime = lifetime
    out.markers.append(look)
    marker_id += 1

    if goal is not None:
        g = Marker()
        g.header, g.ns, g.id = goal.header, 'goal', marker_id
        g.type, g.action = Marker.SPHERE, Marker.ADD
        g.pose.position.x, g.pose.position.y, g.pose.position.z = (
            goal.pose.position.x, goal.pose.position.y, 0.15,
        )
        g.pose.orientation.w = 1.0
        g.scale.x = g.scale.y = g.scale.z = 0.25
        g.color = ColorRGBA(r=0.95, g=0.2, b=0.2, a=0.9)
        if lifetime is not None:
            g.lifetime = lifetime
        out.markers.append(g)

    return out
