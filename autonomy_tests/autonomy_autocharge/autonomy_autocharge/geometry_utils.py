"""Shared geometry helpers for autocharge demo."""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseStamped, Quaternion


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def make_pose_stamped(
    frame_id: str,
    x: float,
    y: float,
    yaw: float,
    stamp,
) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation = yaw_to_quaternion(yaw)
    return pose


def predock_from_charger(
    charger_x: float,
    charger_y: float,
    charger_yaw: float,
    distance_m: float,
) -> tuple[float, float, float]:
    """Deterministic predock pose in front of the charger (map/odom frame)."""
    c = math.cos(float(charger_yaw))
    s = math.sin(float(charger_yaw))
    x = float(charger_x) + float(distance_m) * c
    y = float(charger_y) + float(distance_m) * s
    yaw = math.atan2(float(charger_y) - y, float(charger_x) - x)
    return (x, y, yaw)


def resolve_predock_pose(
    *,
    use_explicit_predock: bool,
    predock_x: float,
    predock_y: float,
    predock_yaw: float,
    charger_x: float,
    charger_y: float,
    charger_yaw: float,
    predock_distance_m: float,
) -> tuple[float, float, float]:
    """Return predock (x, y, yaw); explicit map pose or derived from charger."""
    if use_explicit_predock:
        return (float(predock_x), float(predock_y), float(predock_yaw))
    return predock_from_charger(
        charger_x, charger_y, charger_yaw, predock_distance_m
    )
