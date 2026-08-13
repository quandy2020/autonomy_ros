"""Pluggable ROS command publishers for deployment."""

from __future__ import annotations

from abc import ABC, abstractmethod

from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.publisher import Publisher

from autonomy_navrl.core.types import DeployConfig, VelocityCommand


class CommandPublisher(ABC):
    """Publish high-level velocity commands to the robot."""

    @abstractmethod
    def publish(self, command: VelocityCommand) -> None:
        """Send one velocity command."""


class TwistCommandPublisher(CommandPublisher):
    """Standard geometry_msgs/Twist publisher."""

    def __init__(self, publisher: Publisher) -> None:
        self._publisher = publisher

    def publish(self, command: VelocityCommand) -> None:
        twist = Twist()
        twist.linear.x = float(command.vx)
        twist.linear.y = float(command.vy)
        twist.angular.z = float(command.w)
        self._publisher.publish(twist)


class SkateCommandPublisher(CommandPublisher):
    """jdbot_interfaces/SkateCmd publisher."""

    def __init__(self, publisher: Publisher, skate_msg_type) -> None:
        self._publisher = publisher
        self._skate_msg_type = skate_msg_type

    def publish(self, command: VelocityCommand) -> None:
        msg = self._skate_msg_type()
        msg.xline_vel = float(command.vx)
        msg.yline_vel = float(command.vy)
        msg.wline_vel = float(command.w)
        msg.height = 0.0
        self._publisher.publish(msg)


def create_command_publisher(node: Node, cfg: DeployConfig) -> CommandPublisher:
    """Build a command publisher from deploy config (Twist or SkateCmd)."""
    if cfg.cmd_msg_type.lower() == 'skate':
        try:
            from jdbot_interfaces.msg import SkateCmd

            pub = node.create_publisher(SkateCmd, cfg.cmd_vel_topic, 10)
            return SkateCommandPublisher(pub, SkateCmd)
        except ImportError:
            node.get_logger().warn(
                'jdbot_interfaces not available; falling back to Twist publisher.'
            )
    pub = node.create_publisher(Twist, cfg.cmd_vel_topic, 10)
    return TwistCommandPublisher(pub)
