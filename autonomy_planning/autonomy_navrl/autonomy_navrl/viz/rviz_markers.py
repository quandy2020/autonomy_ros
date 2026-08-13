"""RViz marker builders for deployment visualization."""

from __future__ import annotations

import math

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from autonomy_navrl.core.types import VelocityCommand


def build_goal_and_velocity_markers(
    odom: Odometry,
    goal: PoseStamped,
    command: VelocityCommand,
    map_frame: str,
    base_frame: str,
) -> MarkerArray:
    """Build goal sphere and velocity arrow markers."""
    stamp = odom.header.stamp
    frame = map_frame or odom.header.frame_id or 'map'

    goal_marker = Marker()
    goal_marker.header.stamp = stamp
    goal_marker.header.frame_id = frame
    goal_marker.ns = 'navrl'
    goal_marker.id = 0
    goal_marker.type = Marker.SPHERE
    goal_marker.action = Marker.ADD
    goal_marker.pose = goal.pose
    goal_marker.scale.x = 0.35
    goal_marker.scale.y = 0.35
    goal_marker.scale.z = 0.35
    goal_marker.color = ColorRGBA(r=0.1, g=0.8, b=0.2, a=0.9)

    arrow = Marker()
    arrow.header.stamp = stamp
    arrow.header.frame_id = base_frame or odom.child_frame_id or 'base_link'
    arrow.ns = 'navrl'
    arrow.id = 1
    arrow.type = Marker.ARROW
    arrow.action = Marker.ADD
    arrow.pose.position.x = 0.0
    arrow.pose.position.y = 0.0
    arrow.pose.position.z = 0.2
    yaw = math.atan2(command.vy, command.vx) if abs(command.vx) + abs(command.vy) > 1e-3 else 0.0
    arrow.pose.orientation.z = math.sin(yaw / 2.0)
    arrow.pose.orientation.w = math.cos(yaw / 2.0)
    speed = math.hypot(command.vx, command.vy)
    arrow.scale.x = max(speed, 0.1)
    arrow.scale.y = 0.08
    arrow.scale.z = 0.08
    arrow.color = ColorRGBA(r=0.9, g=0.3, b=0.1, a=0.9)
    arrow.points = [
        Point(x=0.0, y=0.0, z=0.2),
        Point(x=float(command.vx), y=float(command.vy), z=0.2),
    ]

    array = MarkerArray()
    array.markers = [goal_marker, arrow]
    return array
