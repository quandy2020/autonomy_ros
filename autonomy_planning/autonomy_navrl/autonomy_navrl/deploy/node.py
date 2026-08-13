"""ROS 2 deployment node for quadruped navigation."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu
from visualization_msgs.msg import MarkerArray

from autonomy_navrl.deploy.config import load_deploy_config
from autonomy_navrl.deploy.image_utils import depth_to_meters, image_to_numpy
from autonomy_navrl.deploy.inference import PolicyInference
from autonomy_navrl.deploy.publishers import create_command_publisher
from autonomy_navrl.deploy.state_builder import build_deploy_state_vector
from autonomy_navrl.viz.rviz_markers import build_goal_and_velocity_markers


class NavrlNode(Node):
    """Subscribe RGBD/IMU/odom/goal and publish (vx, vy, w)."""

    def __init__(self) -> None:
        super().__init__(
            'navrl_node',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self._cfg = load_deploy_config(self)
        self._inference = PolicyInference(self._cfg)
        self._command_publisher = create_command_publisher(self, self._cfg)
        self._rgb: Image | None = None
        self._depth: Image | None = None
        self._imu: Imu | None = None
        self._odom: Odometry | None = None
        self._goal: PoseStamped | None = None

        self.create_subscription(
            Image, self._cfg.rgb_topic, self._on_rgb, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, self._cfg.depth_topic, self._on_depth, qos_profile_sensor_data
        )
        self.create_subscription(Imu, self._cfg.imu_topic, self._on_imu, 10)
        self.create_subscription(Odometry, self._cfg.odom_topic, self._on_odom, 10)
        self.create_subscription(PoseStamped, self._cfg.goal_topic, self._on_goal, 10)

        self._marker_pub = self.create_publisher(
            MarkerArray, self._cfg.markers_topic, 10
        )
        self._path_pub = self.create_publisher(Path, self._cfg.path_topic, 10)
        period = 1.0 / max(self._cfg.inference_rate_hz, 1.0)
        self.create_timer(period, self._on_timer)
        self.get_logger().info(
            f'NavrlNode ready. checkpoint={self._cfg.checkpoint}'
        )

    def _on_rgb(self, msg: Image) -> None:
        self._rgb = msg

    def _on_depth(self, msg: Image) -> None:
        self._depth = msg

    def _on_imu(self, msg: Imu) -> None:
        self._imu = msg

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_goal(self, msg: PoseStamped) -> None:
        self._goal = msg

    def _on_timer(self) -> None:
        if self._rgb is None or self._depth is None or self._odom is None or self._goal is None:
            return
        try:
            rgb = image_to_numpy(self._rgb)
            depth = depth_to_meters(self._depth)
            state = build_deploy_state_vector(
                self._odom, self._goal, imu=self._imu,
            )
            command = self._inference.infer(rgb, depth, state)
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f'Inference failed: {exc}')
            return

        self._command_publisher.publish(command)
        self._publish_markers(command)

    def _publish_markers(self, command) -> None:
        if self._odom is None or self._goal is None:
            return
        markers = build_goal_and_velocity_markers(
            self._odom,
            self._goal,
            command,
            self._cfg.map_frame,
            self._cfg.base_frame,
        )
        self._marker_pub.publish(markers)


def main() -> None:
    rclpy.init()
    node = NavrlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
