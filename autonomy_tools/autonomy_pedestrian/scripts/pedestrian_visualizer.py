#!/usr/bin/env python3
# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0

"""Bridge pedestrian simulator output to pedsim_msgs for pedsim_rviz_plugin."""

from __future__ import annotations

import math
from typing import Dict, List, Tuple

import rclpy
from derived_object_msgs.msg import ObjectArray
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from pedsim_msgs.msg import (
    SocialActivity,
    SocialActivities,
    SocialRelation,
    SocialRelations,
    TrackedGroup,
    TrackedGroups,
    TrackedPerson,
    TrackedPersons,
)
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

PedState = Tuple[int, float, float, float, float]  # id, x, y, vx, vy
UNKNOWN_VARIANCE = 1e6


class PedestrianVisualizer(Node):
    def __init__(self) -> None:
        super().__init__('pedestrian_visualizer')
        self.declare_parameter('input_topic', '/pedestrian_simulator/pedestrians')
        self.declare_parameter('tracked_persons_topic', '/pedestrian_visualizer/tracked_persons')
        self.declare_parameter('tracked_groups_topic', '/pedestrian_visualizer/tracked_groups')
        self.declare_parameter('social_relations_topic', '/pedestrian_visualizer/social_relations')
        self.declare_parameter('social_activities_topic', '/pedestrian_visualizer/social_activities')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('group_distance', 2.5)
        self.declare_parameter('relation_distance', 3.0)

        input_topic = self.get_parameter('input_topic').value
        self._frame_id = self.get_parameter('frame_id').value
        self._group_distance = float(self.get_parameter('group_distance').value)
        self._relation_distance = float(self.get_parameter('relation_distance').value)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pub_persons = self.create_publisher(
            TrackedPersons, self.get_parameter('tracked_persons_topic').value, qos)
        self._pub_groups = self.create_publisher(
            TrackedGroups, self.get_parameter('tracked_groups_topic').value, qos)
        self._pub_relations = self.create_publisher(
            SocialRelations, self.get_parameter('social_relations_topic').value, qos)
        self._pub_activities = self.create_publisher(
            SocialActivities, self.get_parameter('social_activities_topic').value, qos)
        self.create_subscription(ObjectArray, input_topic, self._on_pedestrians, qos)
        self.get_logger().info(
            f'Bridging {input_topic} -> pedsim_rviz_plugin topics'
        )

    def _on_pedestrians(self, msg: ObjectArray) -> None:
        states = self._parse_states(msg)
        header = msg.header
        header.frame_id = self._frame_id

        self._pub_persons.publish(self._build_tracked_persons(header, states))
        groups = self._cluster_groups(states)
        self._pub_groups.publish(self._build_tracked_groups(header, groups, states))
        self._pub_relations.publish(self._build_social_relations(header, states, groups))
        self._pub_activities.publish(self._build_social_activities(header, groups))

    @staticmethod
    def _finite_velocity(value: float, max_speed: float = 5.0) -> float:
        if not math.isfinite(value) or abs(value) > max_speed:
            return 0.0
        return value

    @staticmethod
    def _parse_states(msg: ObjectArray) -> List[PedState]:
        states: List[PedState] = []
        for obj in msg.objects:
            states.append((
                int(obj.id),
                float(obj.pose.position.x),
                float(obj.pose.position.y),
                PedestrianVisualizer._finite_velocity(float(obj.twist.linear.x)),
                PedestrianVisualizer._finite_velocity(float(obj.twist.linear.y)),
            ))
        return states

    @staticmethod
    def _build_tracked_persons(header, states: List[PedState]) -> TrackedPersons:
        tracked = TrackedPersons()
        tracked.header = header
        for ped_id, x, y, vx, vy in states:
            person = TrackedPerson()
            person.track_id = ped_id
            person.is_occluded = False
            person.is_matched = True
            person.detection_id = ped_id
            person.pose = PoseWithCovariance()
            person.pose.pose.position.x = x
            person.pose.pose.position.y = y
            person.pose.covariance[0] = 0.25
            person.pose.covariance[7] = 0.25
            person.pose.covariance[14] = UNKNOWN_VARIANCE
            person.pose.covariance[35] = UNKNOWN_VARIANCE
            person.twist = TwistWithCovariance()
            person.twist.twist.linear.x = vx
            person.twist.twist.linear.y = vy
            person.twist.covariance[0] = 0.25
            person.twist.covariance[7] = 0.25
            person.twist.covariance[14] = UNKNOWN_VARIANCE
            for idx in range(3, 6):
                person.twist.covariance[idx + idx * 6] = float('inf')
            tracked.tracks.append(person)
        return tracked

    def _cluster_groups(self, states: List[PedState]) -> Dict[int, List[int]]:
        if not states:
            return {}

        n = len(states)
        parent = list(range(n))

        def find(i: int) -> int:
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i

        def union(i: int, j: int) -> None:
            ri, rj = find(i), find(j)
            if ri != rj:
                parent[rj] = ri

        for i in range(n):
            for j in range(i + 1, n):
                dx = states[i][1] - states[j][1]
                dy = states[i][2] - states[j][2]
                if math.hypot(dx, dy) <= self._group_distance:
                    union(i, j)

        clusters: Dict[int, List[int]] = {}
        for idx, state in enumerate(states):
            root = find(idx)
            clusters.setdefault(root, []).append(state[0])
        return {gid: members for gid, members in enumerate(clusters.values())}

    @staticmethod
    def _build_tracked_groups(header, groups: Dict[int, List[int]],
                              states: List[PedState]) -> TrackedGroups:
        pos_by_id = {s[0]: (s[1], s[2]) for s in states}
        msg = TrackedGroups()
        msg.header = header
        for group_id, member_ids in groups.items():
            group = TrackedGroup()
            group.group_id = group_id
            group.track_ids = [int(m) for m in member_ids]
            cx = cy = 0.0
            for mid in member_ids:
                x, y = pos_by_id[mid]
                cx += x
                cy += y
            n = max(1, len(member_ids))
            group.center_of_gravity = PoseWithCovariance()
            group.center_of_gravity.pose.position.x = cx / n
            group.center_of_gravity.pose.position.y = cy / n
            msg.groups.append(group)
        return msg

    def _build_social_relations(self, header, states: List[PedState],
                                groups: Dict[int, List[int]]) -> SocialRelations:
        pos_by_id = {s[0]: (s[1], s[2]) for s in states}
        msg = SocialRelations()
        msg.header = header

        for member_ids in groups.values():
            if len(member_ids) < 2:
                continue
            for i, id1 in enumerate(member_ids):
                for id2 in member_ids[i + 1:]:
                    x1, y1 = pos_by_id[id1]
                    x2, y2 = pos_by_id[id2]
                    dist = math.hypot(x1 - x2, y1 - y2)
                    if dist > self._relation_distance:
                        continue
                    rel = SocialRelation()
                    rel.type = SocialRelation.TYPE_SPATIAL
                    rel.track1_id = int(id1)
                    rel.track2_id = int(id2)
                    rel.strength = max(0.0, min(1.0, 1.0 - dist / self._relation_distance))
                    msg.elements.append(rel)
        return msg

    @staticmethod
    def _build_social_activities(header, groups: Dict[int, List[int]]) -> SocialActivities:
        msg = SocialActivities()
        msg.header = header

        for member_ids in groups.values():
            activity = SocialActivity()
            activity.confidence = 1.0
            activity.track_ids = [int(m) for m in member_ids]
            if len(member_ids) > 1:
                activity.type = SocialActivity.TYPE_GROUP_MOVING
            else:
                activity.type = SocialActivity.TYPE_INDIVIDUAL_MOVING
            msg.elements.append(activity)
        return msg


def main() -> None:
    rclpy.init()
    node = PedestrianVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
