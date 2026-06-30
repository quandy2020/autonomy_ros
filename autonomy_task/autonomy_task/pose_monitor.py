"""Map-frame pose from TF (map → base_link / base_footprint)."""

from __future__ import annotations

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from autonomy_task.waypoint_filter import Pose


def resolve_tf_frame(node: Node, frame: str) -> str:
    """Prefix frame with node namespace when not already qualified."""
    fid = frame.strip('/')
    if '/' in fid:
        return fid
    ns = node.get_namespace().strip('/')
    if ns:
        return f'{ns}/{fid}'
    return fid


class MapPoseMonitor:
    """Lookup robot pose in the map frame via TF."""

    def __init__(
        self,
        node: Node,
        *,
        map_frame: str = 'map',
        base_frame: str = 'base_footprint',
        namespace: str = '',
    ) -> None:
        self._node = node
        self._map_frame = resolve_tf_frame(node, map_frame) if not namespace else (
            f'{namespace.strip("/")}/{map_frame.strip("/")}'
        )
        self._base_frame = resolve_tf_frame(node, base_frame) if not namespace else (
            f'{namespace.strip("/")}/{base_frame.strip("/")}'
        )
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, node)

    @property
    def map_frame(self) -> str:
        return self._map_frame

    @property
    def base_frame(self) -> str:
        return self._base_frame

    def pose(self, timeout_sec: float = 0.1) -> Pose | None:
        try:
            transform = self._buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=timeout_sec),
            )
        except TransformException as exc:
            self._node.get_logger().debug(
                f'TF {self._map_frame}→{self._base_frame}: {exc}')
            return None
        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return Pose(x=t.x, y=t.y, yaw=yaw)
