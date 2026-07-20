"""Publish dynamic robots as one RViz RobotModel description + TF forest."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from habitat.config import Config
from habitat.robot.paths import materialize_robot_model_urdf, robot_joint_pose_preset


@dataclass
class _JointTf:
    name: str
    parent: str
    child: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]
    axis: tuple[float, float, float]
    joint_type: str
    position: float


@dataclass
class _RobotModelEntry:
    prefix: str
    asset_name: str
    root_link: str
    joint_tfs: list[_JointTf]


def _quat_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _parse_xyz(raw: str | None) -> tuple[float, float, float]:
    parts = (raw or '0 0 0').split()
    vals = [float(part) for part in parts[:3]]
    while len(vals) < 3:
        vals.append(0.0)
    return (vals[0], vals[1], vals[2])


def _quat_multiply(
    q1: tuple[float, float, float, float],
    q2: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


class RobotModelPublisher:
    """Publish one combined RobotModel for all dynamic robots."""

    def __init__(self, node: Node, cfg: Config) -> None:
        self._node = node
        self._cfg = cfg
        self._tf = TransformBroadcaster(node)
        self._description_topic = '/dynamic_robot_description'
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._desc_pub = node.create_publisher(String, self._description_topic, qos)
        self._entries: list[_RobotModelEntry] = []

    @property
    def description_topic(self) -> str:
        return self._description_topic

    def set_assets(self, asset_names: list[str]) -> None:
        clean_assets = [str(name).strip() for name in asset_names if str(name).strip()]
        if not clean_assets:
            self._entries = []
            return
        if [entry.asset_name for entry in self._entries] == clean_assets:
            return

        entries: list[_RobotModelEntry] = []
        robot_root = ET.Element('robot', {'name': 'dynamic_robot_fleet'})
        material_names: set[str] = set()
        for idx, asset_name in enumerate(clean_assets):
            prefix = f'dynamic_robot_{idx}_'
            urdf_path = Path(materialize_robot_model_urdf(self._cfg, asset_name, prefix=prefix))
            text = urdf_path.read_text(encoding='utf-8')
            root = ET.fromstring(text)
            root_link, joint_tfs = self._parse_joint_tfs(asset_name, text, prefix)
            for material in root.findall('material'):
                name = str(material.attrib.get('name', '')).strip()
                if name and name not in material_names:
                    robot_root.append(material)
                    material_names.add(name)
            for tag in ('link', 'joint'):
                for elem in root.findall(tag):
                    robot_root.append(elem)
            entries.append(_RobotModelEntry(prefix, asset_name, root_link, joint_tfs))
        self._entries = entries
        self._desc_pub.publish(String(data=ET.tostring(robot_root, encoding='unicode')))

    def publish_poses(
        self,
        stamp: rclpy.time.Time,
        poses: list[tuple[float, float, float, float]],
        joint_states: list[dict[str, float]] | None = None,
    ) -> None:
        if not self._entries:
            return
        transforms: list[TransformStamped] = []
        for idx, (entry, (map_x, map_y, floor, yaw)) in enumerate(zip(self._entries, poses)):
            joint_state = joint_states[idx] if joint_states is not None and idx < len(joint_states) else {}
            base = TransformStamped()
            base.header.stamp = stamp.to_msg()
            base.header.frame_id = self._cfg.map_frame
            base.child_frame_id = entry.root_link
            base.transform.translation.x = float(map_x)
            base.transform.translation.y = float(map_y)
            base.transform.translation.z = float(floor)
            qx, qy, qz, qw = _quat_from_euler(0.0, 0.0, yaw)
            base.transform.rotation.x = qx
            base.transform.rotation.y = qy
            base.transform.rotation.z = qz
            base.transform.rotation.w = qw
            transforms.append(base)
            for joint in entry.joint_tfs:
                msg = TransformStamped()
                msg.header.stamp = stamp.to_msg()
                msg.header.frame_id = joint.parent
                msg.child_frame_id = joint.child
                msg.transform.translation.x = joint.xyz[0]
                msg.transform.translation.y = joint.xyz[1]
                msg.transform.translation.z = joint.xyz[2]
                qx, qy, qz, qw = _quat_from_euler(*joint.rpy)
                joint_position = float(joint_state.get(joint.name, joint.position))
                if joint.joint_type in {'revolute', 'continuous'} and abs(joint_position) > 1e-9:
                    half = joint_position * 0.5
                    ax, ay, az = joint.axis
                    norm = math.sqrt(ax * ax + ay * ay + az * az)
                    if norm > 1e-9:
                        ax /= norm
                        ay /= norm
                        az /= norm
                        q_joint = (
                            ax * math.sin(half),
                            ay * math.sin(half),
                            az * math.sin(half),
                            math.cos(half),
                        )
                        qx, qy, qz, qw = _quat_multiply((qx, qy, qz, qw), q_joint)
                msg.transform.rotation.x = qx
                msg.transform.rotation.y = qy
                msg.transform.rotation.z = qz
                msg.transform.rotation.w = qw
                transforms.append(msg)
        if transforms:
            self._tf.sendTransform(transforms)

    def _parse_joint_tfs(self, asset_name: str, text: str, prefix: str) -> tuple[str, list[_JointTf]]:
        root = ET.fromstring(text)
        preset = robot_joint_pose_preset(asset_name)
        links = [link.attrib.get('name', '').strip() for link in root.findall('link')]
        child_links = set()
        joints: list[_JointTf] = []
        for joint in root.findall('joint'):
            parent = joint.find('parent')
            child = joint.find('child')
            if parent is None or child is None:
                continue
            parent_link = str(parent.attrib.get('link', '')).strip()
            child_link = str(child.attrib.get('link', '')).strip()
            if not parent_link or not child_link:
                continue
            origin = joint.find('origin')
            xyz = _parse_xyz(origin.attrib.get('xyz') if origin is not None else None)
            rpy = _parse_xyz(origin.attrib.get('rpy') if origin is not None else None)
            axis_elem = joint.find('axis')
            axis = _parse_xyz(axis_elem.attrib.get('xyz') if axis_elem is not None else '0 0 1')
            joint_name = str(joint.attrib.get('name', '')).strip()
            lookup_name = (
                joint_name[len(prefix):]
                if prefix and joint_name.startswith(prefix)
                else joint_name
            )
            joint_type = str(joint.attrib.get('type', 'fixed')).strip().lower()
            joints.append(_JointTf(
                lookup_name,
                parent_link,
                child_link,
                xyz,
                rpy,
                axis,
                joint_type,
                float(preset.get(lookup_name, 0.0)),
            ))
            child_links.add(child_link)
        roots = [name for name in links if name and name not in child_links]
        root_link = roots[0] if roots else ''
        return root_link, joints
