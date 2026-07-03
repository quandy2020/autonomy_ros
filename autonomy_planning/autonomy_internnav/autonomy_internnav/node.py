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

"""InternNav ROS 2 bridge node with multi-policy inference."""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from autonomy_internnav.config import load
from autonomy_internnav.conversions import (
    bgr_array_to_image_msg,
    camera_info_to_intrinsic,
    depth_to_meters,
    goal_to_robot_xy,
    image_to_bgr,
    trajectories_body_to_map_xy,
    trajectory_body_to_map_xy,
    trajectory_to_path,
    trajectory_to_twist,
)
from autonomy_internnav.inference import PolicyInference, create_policy_inference
from autonomy_internnav.visualization import build_navdp_markers


class InternNavNode(Node):
    """ROS bridge: RGB-D + goal -> policy -> cmd_vel / path."""

    def __init__(self) -> None:
        super().__init__(
            'internnav_node',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self._cfg = load(self)
        self._goal_type = self._cfg.goal_type.lower().strip()

        self._rgb: Image | None = None
        self._depth: Image | None = None
        self._odom: Odometry | None = None
        self._goal: PoseStamped | None = None
        self._image_goal: Image | None = None
        self._pixel_goal: PointStamped | None = None
        self._intrinsic = None

        self._inference: PolicyInference | None = None
        self._inference_error: str | None = None
        self._waiting_logged = False
        self._model_loading_logged = False
        self._goal_behind_warned = False
        sensor_cb = ReentrantCallbackGroup()

        self.create_subscription(
            Image, self._cfg.rgb_topic, self._on_rgb, qos_profile_sensor_data,
            callback_group=sensor_cb)
        self.create_subscription(
            Image, self._cfg.depth_topic, self._on_depth, qos_profile_sensor_data,
            callback_group=sensor_cb)
        self.create_subscription(
            Odometry, self._cfg.odom_topic, self._on_odom, 10,
            callback_group=sensor_cb)
        self.create_subscription(
            CameraInfo, self._cfg.camera_info_topic, self._on_camera_info, 10,
            callback_group=sensor_cb)

        if self._goal_type == 'point':
            self.create_subscription(
                PoseStamped, self._cfg.goal_topic, self._on_goal, 10,
                callback_group=sensor_cb)
        elif self._goal_type == 'image':
            self.create_subscription(
                Image, self._cfg.image_goal_topic, self._on_image_goal,
                qos_profile_sensor_data, callback_group=sensor_cb)
        elif self._goal_type == 'pixel':
            self.create_subscription(
                PointStamped, self._cfg.pixel_goal_topic, self._on_pixel_goal, 10,
                callback_group=sensor_cb)
        elif self._goal_type == 'point_image':
            self.create_subscription(
                PoseStamped, self._cfg.goal_topic, self._on_goal, 10,
                callback_group=sensor_cb)
            self.create_subscription(
                Image, self._cfg.image_goal_topic, self._on_image_goal,
                qos_profile_sensor_data, callback_group=sensor_cb)

        self._cmd_pub = self.create_publisher(Twist, self._cfg.cmd_vel_topic, 10)
        self._path_pub = self.create_publisher(Path, self._cfg.path_topic, 10)
        self._markers_pub = self.create_publisher(
            MarkerArray, self._cfg.markers_topic, 10)
        self._overlay_pub = self.create_publisher(
            Image, self._cfg.overlay_topic, 10)

        service_cb = ReentrantCallbackGroup()
        self.create_service(
            Trigger, '~/reset', self._on_reset, callback_group=service_cb)

        period = 1.0 / max(self._cfg.inference_rate_hz, 0.1)
        self.create_timer(period, self._on_timer, callback_group=sensor_cb)

        self.get_logger().info(
            f'InternNav ready policy={self._cfg.policy} goal_type={self._goal_type} '
            f'(rgb={self._cfg.rgb_topic}, depth={self._cfg.depth_topic})')

    def _get_inference(self) -> PolicyInference:
        if self._inference is None:
            self._inference = create_policy_inference(self._cfg, self._intrinsic)
        return self._inference

    def _on_rgb(self, msg: Image) -> None:
        self._rgb = msg

    def _on_depth(self, msg: Image) -> None:
        self._depth = msg

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_goal(self, msg: PoseStamped) -> None:
        self._goal = msg
        self._goal_behind_warned = False
        self.get_logger().info(
            f'Goal updated: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    def _on_image_goal(self, msg: Image) -> None:
        self._image_goal = msg
        self.get_logger().info('Image goal updated')

    def _on_pixel_goal(self, msg: PointStamped) -> None:
        self._pixel_goal = msg
        self.get_logger().info(
            f'Pixel goal updated: ({msg.point.x:.0f}, {msg.point.y:.0f})')

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._intrinsic = camera_info_to_intrinsic(msg)
        if self._inference is not None:
            self._inference.intrinsic = self._intrinsic

    def _on_reset(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self._inference is not None:
            self._inference.reset()
        self._goal = None
        self._image_goal = None
        self._pixel_goal = None
        self._inference_error = None
        self._cmd_pub.publish(Twist())
        self._markers_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
        response.success = True
        response.message = f'{self._cfg.policy} memory reset'
        return response

    def _inputs_ready(self) -> bool:
        if self._rgb is None or self._depth is None or self._odom is None:
            return False
        if self._goal_type == 'point':
            return self._goal is not None
        if self._goal_type == 'image':
            return self._image_goal is not None
        if self._goal_type == 'pixel':
            return self._pixel_goal is not None
        if self._goal_type == 'point_image':
            return self._goal is not None and self._image_goal is not None
        return True

    def _missing_inputs(self) -> list[str]:
        missing = []
        if self._rgb is None:
            missing.append('rgb')
        if self._depth is None:
            missing.append('depth')
        if self._odom is None:
            missing.append('odom')
        if self._goal_type == 'point' and self._goal is None:
            missing.append('goal')
        if self._goal_type == 'image' and self._image_goal is None:
            missing.append('image_goal')
        if self._goal_type == 'pixel' and self._pixel_goal is None:
            missing.append('pixel_goal')
        if self._goal_type == 'point_image':
            if self._goal is None:
                missing.append('goal')
            if self._image_goal is None:
                missing.append('image_goal')
        return missing

    def _run_inference(self, rgb, depth):
        inference = self._get_inference()
        if self._goal_type == 'point':
            goal_xy, rel = goal_to_robot_xy(self._goal, self._odom)
            if rel[0] < 0.0 and not self._goal_behind_warned:
                self.get_logger().warn(
                    f'Goal behind robot (body x={rel[0]:.2f}); NavDP clips x to 0 — '
                    'set goal in front of the robot in RViz')
                self._goal_behind_warned = True
            return inference.step_pointgoal(goal_xy, rgb, depth), rel
        if self._goal_type == 'image':
            goal_bgr = image_to_bgr(self._image_goal)
            return inference.step_imagegoal(goal_bgr, rgb, depth), None
        if self._goal_type == 'pixel':
            uv = np.array(
                [self._pixel_goal.point.x, self._pixel_goal.point.y],
                dtype=np.float32,
            )
            rel = uv
            return inference.step_pixelgoal(uv, rgb, depth), rel
        if self._goal_type == 'point_image':
            goal_xy, rel = goal_to_robot_xy(self._goal, self._odom)
            goal_bgr = image_to_bgr(self._image_goal)
            return inference.step_point_image_goal(goal_xy, goal_bgr, rgb, depth), rel
        return inference.step_nogoal(rgb, depth), None

    def _on_timer(self) -> None:
        if not self._inputs_ready():
            if not self._waiting_logged:
                self.get_logger().info(
                    f'Waiting for: {", ".join(self._missing_inputs())}')
                self._waiting_logged = True
            return
        self._waiting_logged = False
        if self._inference_error is not None:
            return

        try:
            if self._inference is None and not self._model_loading_logged:
                self.get_logger().info(
                    f'Loading {self._cfg.policy} model (first inference may take ~30s) ...')
                self._model_loading_logged = True

            rgb = image_to_bgr(self._rgb)
            depth = depth_to_meters(self._depth, self._cfg.depth_encoding)
            result, goal_rel = self._run_inference(rgb, depth)
            trajectory = result.trajectory[0] if result.trajectory.ndim == 3 else result.trajectory
            all_traj = result.all_trajectory
            values = result.values

            stamp = self.get_clock().now().to_msg()
            map_traj = trajectory_body_to_map_xy(trajectory, self._odom)
            map_candidates = (
                trajectories_body_to_map_xy(all_traj, self._odom)
                if all_traj is not None and np.asarray(all_traj).size > 0
                else None
            )
            path = trajectory_to_path(map_traj, stamp, self._cfg.map_frame)
            self._path_pub.publish(path)

            cmd = trajectory_to_twist(
                trajectory,
                self._cfg.linear_speed,
                self._cfg.angular_speed,
                self._cfg.lookahead_index,
            )
            if result.stopped:
                cmd = Twist()
            self._cmd_pub.publish(cmd)

            robot_xy = (
                self._odom.pose.pose.position.x,
                self._odom.pose.pose.position.y,
            )
            marker_header = Header(stamp=stamp, frame_id=self._cfg.map_frame)
            self._markers_pub.publish(build_navdp_markers(
                marker_header,
                selected=map_traj,
                candidates=map_candidates,
                values=values,
                goal=self._goal,
                cmd=cmd,
                lookahead_index=self._cfg.lookahead_index,
                robot_xy=robot_xy,
            ))

            if result.trajectory_mask is not None:
                overlay_frame = (
                    self._rgb.header.frame_id
                    if self._rgb.header.frame_id
                    else self._cfg.base_frame
                )
                self._overlay_pub.publish(bgr_array_to_image_msg(
                    result.trajectory_mask,
                    stamp,
                    overlay_frame,
                ))
            goal_msg = ''
            if goal_rel is not None:
                if self._goal_type == 'pixel':
                    goal_msg = f', pixel=({goal_rel[0]:.0f},{goal_rel[1]:.0f})'
                else:
                    goal_msg = f', goal_body=({goal_rel[0]:.2f},{goal_rel[1]:.2f})'
            end = map_traj[-1] if len(map_traj) else (0.0, 0.0, 0.0)
            n_samples = len(map_candidates) if map_candidates is not None else 0
            critic_msg = ''
            if values is not None and np.asarray(values).size > 0:
                flat = np.asarray(values, dtype=np.float64).reshape(-1)
                critic_msg = (
                    f', samples={n_samples}, critic=[{flat.min():.2f},{flat.max():.2f}]'
                )
            stop_msg = ', STOP' if result.stopped else ''
            self.get_logger().info(
                f'{self._cfg.policy} plan: selected {len(path.poses)} pts{goal_msg}'
                f'{critic_msg}{stop_msg}, end_map=({end[0]:.2f},{end[1]:.2f}), '
                f'cmd=({cmd.linear.x:.2f},{cmd.angular.z:.2f})',
                throttle_duration_sec=2.0,
            )
        except Exception as exc:  # noqa: BLE001
            self._inference_error = str(exc)
            self.get_logger().error(f'{self._cfg.policy} inference failed: {exc}')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = InternNavNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
