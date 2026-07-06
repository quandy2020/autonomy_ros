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

"""RViz Interactive Marker driving the Habitat topdown pinhole camera."""

from __future__ import annotations

from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
    MenuEntry,
)

from habitat.config import Config
from habitat.sim import Session

MENU_ZOOM_IN = 1
MENU_ZOOM_OUT = 2
MENU_FOV_WIDE = 3
MENU_FOV_NARROW = 4


def _quat_from_msg(q: Quaternion) -> np.quaternion:
    return np.quaternion(q.w, q.x, q.y, q.z)


def _pose_to_arrays(pose: Pose) -> tuple[np.ndarray, np.quaternion]:
    pos = np.array(
        [pose.position.x, pose.position.y, pose.position.z],
        dtype=np.float32,
    )
    return pos, _quat_from_msg(pose.orientation)


class TopdownInteractive:
    """6-DOF marker in map frame; updates Session topdown sensor each drag."""

    def __init__(
        self,
        node: rclpy.node.Node,
        cfg: Config,
        session: Session,
        logger: Any,
    ) -> None:
        self._cfg = cfg
        self._session = session
        self._logger = logger
        pos, rot, hfov = session.ensure_interactive_topdown()
        self._server = InteractiveMarkerServer(node, 'habitat_topdown_cam')
        self._make_marker(pos, rot)
        self._server.insert(
            self._marker,
            feedback_callback=self._on_feedback,
        )
        self._server.applyChanges()
        self._session.set_interactive_topdown(pos, rot, hfov)
        self._logger.info(
            '[habitat] Interactive topdown: drag the 6-DOF marker in RViz 3D view '
            '(Displays → InteractiveMarkers); right-click menu for zoom / FOV',
        )

    @property
    def _marker(self) -> InteractiveMarker:
        return self._im

    def _make_marker(self, pos: np.ndarray, rot: np.quaternion) -> None:
        im = InteractiveMarker()
        im.header.frame_id = self._cfg.topdown_frame
        im.name = 'topdown_camera'
        im.description = 'Habitat overview camera (drag to orbit, menu: zoom/FOV)'
        im.scale = 1.2
        im.pose.position.x = float(pos[0])
        im.pose.position.y = float(pos[1])
        im.pose.position.z = float(pos[2])
        im.pose.orientation.w = float(rot.w)
        im.pose.orientation.x = float(rot.x)
        im.pose.orientation.y = float(rot.y)
        im.pose.orientation.z = float(rot.z)

        body = InteractiveMarkerControl()
        body.always_visible = True
        body.interaction_mode = InteractiveMarkerControl.NONE
        body.markers.append(self._camera_frustum_marker())
        im.controls.append(body)

        move = InteractiveMarkerControl()
        move.name = 'move_rotate'
        move.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
        im.controls.append(move)

        for axis, mode in (
            ('x', InteractiveMarkerControl.ROTATE_AXIS),
            ('y', InteractiveMarkerControl.ROTATE_AXIS),
            ('z', InteractiveMarkerControl.ROTATE_AXIS),
        ):
            c = InteractiveMarkerControl()
            c.name = f'rotate_{axis}'
            c.interaction_mode = mode
            if axis == 'x':
                c.orientation.w = 1.0
                c.orientation.x = 1.0
            elif axis == 'y':
                c.orientation.w = 1.0
                c.orientation.y = 1.0
            else:
                c.orientation.w = 1.0
                c.orientation.z = 1.0
            im.controls.append(c)

        for entry_id, title in (
            (MENU_ZOOM_IN, 'Zoom in (narrower FOV)'),
            (MENU_ZOOM_OUT, 'Zoom out (wider FOV)'),
            (MENU_FOV_WIDE, 'Wider FOV (+10°)'),
            (MENU_FOV_NARROW, 'Narrower FOV (-10°)'),
        ):
            menu = MenuEntry()
            menu.id = entry_id
            menu.parent_id = 0
            menu.title = title
            im.menu_entries.append(menu)

        self._im = im

    @staticmethod
    def _camera_frustum_marker() -> Marker:
        m = Marker()
        m.type = Marker.LINE_LIST
        m.scale.x = 0.04
        m.color.r = 0.1
        m.color.g = 0.75
        m.color.b = 1.0
        m.color.a = 0.95
        # Simple camera icon: apex at origin, square base at -Z (Habitat forward).
        apex = Point(x=0.0, y=0.0, z=0.0)
        base = 0.35
        depth = 0.55
        corners = [
            Point(x=-base, y=base, z=-depth),
            Point(x=base, y=base, z=-depth),
            Point(x=base, y=-base, z=-depth),
            Point(x=-base, y=-base, z=-depth),
        ]
        for c in corners:
            m.points.append(apex)
            m.points.append(c)
        for i in range(4):
            m.points.append(corners[i])
            m.points.append(corners[(i + 1) % 4])
        return m

    def _on_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            self._on_menu(feedback.menu_entry_id)
            return
        if feedback.event_type not in (
            InteractiveMarkerFeedback.POSE_UPDATE,
            InteractiveMarkerFeedback.MOUSE_UP,
        ):
            return
        pos, rot = _pose_to_arrays(feedback.pose)
        _, _, hfov = self._session.ensure_interactive_topdown()
        self._session.set_interactive_topdown(pos, rot, hfov)

    def _on_menu(self, entry_id: int) -> None:
        pos, rot, hfov = self._session.ensure_interactive_topdown()
        if entry_id == MENU_ZOOM_IN:
            hfov = max(20.0, hfov * 0.82)
        elif entry_id == MENU_ZOOM_OUT:
            hfov = min(120.0, hfov * 1.22)
        elif entry_id == MENU_FOV_WIDE:
            hfov = min(120.0, hfov + 10.0)
        elif entry_id == MENU_FOV_NARROW:
            hfov = max(20.0, hfov - 10.0)
        self._session.set_interactive_topdown(pos, rot, hfov)
