"""TF helpers for robot pose reset."""

from __future__ import annotations

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def resolve_tf_frame(node: Node, frame: str) -> str:
    fid = frame.strip('/')
    if '/' in fid:
        return fid
    ns = node.get_namespace().strip('/')
    if ns:
        return f'{ns}/{fid}'
    return fid


class MapBaseTransform:
    """Lookup map → base_link/base_footprint transform."""

    def __init__(self, node: Node, *, map_frame: str, base_frame: str) -> None:
        self._node = node
        self._map_frame = resolve_tf_frame(node, map_frame)
        self._base_frame = resolve_tf_frame(node, base_frame)
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, node)

    def lookup_pose(self, timeout_sec: float = 0.2) -> PoseStamped | None:
        try:
            transform = self._buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=timeout_sec),
            )
        except TransformException as exc:
            self._node.get_logger().warning(
                f'TF {self._map_frame}→{self._base_frame} failed: {exc}')
            return None
        pose = PoseStamped()
        pose.header.stamp = transform.header.stamp
        pose.header.frame_id = self._map_frame.split('/')[-1]
        pose.pose.position = transform.transform.translation
        pose.pose.orientation = transform.transform.rotation
        return pose
