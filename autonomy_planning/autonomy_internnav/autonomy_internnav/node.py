# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""InternNav ROS 2 bridge node with multi-policy inference."""

from __future__ import annotations

import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from autonomy_internnav.config import load
from autonomy_internnav.conversions import (
    bgr_array_to_image_msg,
    camera_info_to_intrinsic,
    depth_to_meters,
    goal_to_body_pose,
    goal_to_robot_xy,
    image_to_bgr,
    trajectory_to_path,
    trajectory_to_twist,
)
from autonomy_internnav.inference import InferenceResult, create_policy_inference
from autonomy_internnav.visualization import build_navdp_markers

_GOAL_FIELDS: dict[str, tuple[str, ...]] = {
    'point': ('goal',),
    'image': ('image_goal',),
    'pixel': ('pixel_goal',),
    'point_image': ('goal', 'image_goal'),
    'nogoal': (),
}


class InternNavNode(Node):
    """ROS bridge: RGB-D + goal -> policy -> cmd_vel / path / markers / overlay."""

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
        self._intrinsic: np.ndarray | None = None

        self._inference = None
        self._inference_error: str | None = None
        self._waiting_logged = False
        self._model_loading_logged = False
        self._goal_behind_warned = False
        self._overlay_all_traj: np.ndarray | None = None
        self._overlay_values: np.ndarray | None = None
        self._intrinsic_warned = False

        sensor_cb = ReentrantCallbackGroup()
        infer_cb = MutuallyExclusiveCallbackGroup()
        overlay_cb = MutuallyExclusiveCallbackGroup()
        qos_sensor = qos_profile_sensor_data

        self.create_subscription(
            Image, self._cfg.rgb_topic, self._on_rgb, qos_sensor,
            callback_group=sensor_cb,
        )
        self.create_subscription(
            Image, self._cfg.depth_topic, self._on_depth, qos_sensor,
            callback_group=sensor_cb,
        )
        self.create_subscription(
            Odometry, self._cfg.odom_topic, self._on_odom, 10,
            callback_group=sensor_cb,
        )
        self.create_subscription(
            CameraInfo, self._cfg.camera_info_topic, self._on_camera_info, 10,
            callback_group=sensor_cb,
        )
        if self._goal_type in ('point', 'point_image'):
            self.create_subscription(
                PoseStamped, self._cfg.goal_topic, self._on_goal, 10,
                callback_group=sensor_cb,
            )
        if self._goal_type in ('image', 'point_image'):
            self.create_subscription(
                Image, self._cfg.image_goal_topic, self._on_image_goal,
                qos_sensor, callback_group=sensor_cb,
            )
        if self._goal_type == 'pixel':
            self.create_subscription(
                PointStamped, self._cfg.pixel_goal_topic, self._on_pixel_goal, 10,
                callback_group=sensor_cb,
            )

        self._cmd_pub = self.create_publisher(Twist, self._cfg.cmd_vel_topic, 10)
        self._path_pub = self.create_publisher(Path, self._cfg.path_topic, 10)
        self._markers_pub = self.create_publisher(
            MarkerArray, self._cfg.markers_topic, 10,
        )
        self._overlay_pub = self.create_publisher(
            Image, self._cfg.overlay_topic, qos_profile_system_default,
        )
        self.create_service(Trigger, '~/reset', self._on_reset, callback_group=sensor_cb)

        infer_period = 1.0 / max(self._cfg.inference_rate_hz, 0.1)
        self.create_timer(infer_period, self._on_timer, callback_group=infer_cb)
        if self._cfg.overlay_decouple and self._cfg.overlay_rate_hz > 0.0:
            self.create_timer(
                1.0 / self._cfg.overlay_rate_hz, self._on_overlay_timer,
                callback_group=overlay_cb,
            )

        self.get_logger().info(
            f'InternNav ready policy={self._cfg.policy} goal_type={self._goal_type} '
            f'inference_rate={self._cfg.inference_rate_hz}Hz '
            f'(rgb={self._cfg.rgb_topic}, depth={self._cfg.depth_topic})',
        )

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
            f'Goal updated: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})',
        )

    def _on_image_goal(self, msg: Image) -> None:
        self._image_goal = msg
        self.get_logger().info('Image goal updated')

    def _on_pixel_goal(self, msg: PointStamped) -> None:
        self._pixel_goal = msg
        self.get_logger().info(
            f'Pixel goal updated: ({msg.point.x:.0f}, {msg.point.y:.0f})',
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._intrinsic = camera_info_to_intrinsic(msg)
        if self._inference is not None:
            self._inference.intrinsic = self._intrinsic
        self._intrinsic_warned = False

    def _on_reset(self, _request, response):
        if self._inference is not None:
            self._inference.reset()
        self._goal = None
        self._image_goal = None
        self._pixel_goal = None
        self._overlay_all_traj = None
        self._overlay_values = None
        self._inference_error = None
        self._cmd_pub.publish(Twist())
        self._markers_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
        response.success = True
        response.message = f'{self._cfg.policy} memory reset'
        return response

    def _missing_inputs(self) -> list[str]:
        fields = {
            'goal': self._goal,
            'image_goal': self._image_goal,
            'pixel_goal': self._pixel_goal,
        }
        missing: list[str] = []
        if self._rgb is None:
            missing.append('rgb')
        if self._depth is None:
            missing.append('depth')
        if self._odom is None:
            missing.append('odom')
        for name in _GOAL_FIELDS.get(self._goal_type, ()):
            if fields[name] is None:
                missing.append(name)
        return missing

    def _run_inference(self, rgb, depth) -> tuple[InferenceResult, np.ndarray | None]:
        if self._inference is None:
            self._inference = create_policy_inference(self._cfg, self._intrinsic)
        inference = self._inference
        goal_rel = None

        if self._goal_type == 'point':
            goal_xy, goal_rel = goal_to_robot_xy(self._goal, self._odom)
            if goal_rel[0] < 0.0 and not self._goal_behind_warned:
                self.get_logger().warn(
                    f'Goal behind robot (body x={goal_rel[0]:.2f}); NavDP clips x to 0 — '
                    'set goal in front of the robot in RViz',
                )
                self._goal_behind_warned = True
            return inference.step_pointgoal(goal_xy, rgb, depth), goal_rel

        if self._goal_type == 'image':
            return inference.step_imagegoal(image_to_bgr(self._image_goal), rgb, depth), None

        if self._goal_type == 'pixel':
            goal_rel = np.array(
                [self._pixel_goal.point.x, self._pixel_goal.point.y], dtype=np.float32,
            )
            return inference.step_pixelgoal(goal_rel, rgb, depth), goal_rel

        if self._goal_type == 'point_image':
            goal_xy, goal_rel = goal_to_robot_xy(self._goal, self._odom)
            return inference.step_point_image_goal(
                goal_xy, image_to_bgr(self._image_goal), rgb, depth,
            ), goal_rel

        return inference.step_nogoal(rgb, depth), None

    def _on_overlay_timer(self) -> None:
        if (
            not self._cfg.overlay_decouple
            or self._overlay_all_traj is None
            or self._overlay_values is None
            or self._rgb is None
            or self._inference is None
        ):
            return
        if self._intrinsic is None and not self._intrinsic_warned:
            self.get_logger().warn(
                'camera_info not received yet; overlay projection uses default intrinsic',
                throttle_duration_sec=10.0,
            )
            self._intrinsic_warned = True
        try:
            mask = self._inference.render_overlay(
                image_to_bgr(self._rgb), self._overlay_all_traj, self._overlay_values,
            )
            if mask is not None:
                self._overlay_pub.publish(bgr_array_to_image_msg(
                    mask,
                    self._rgb.header.stamp,
                    self._rgb.header.frame_id or self._cfg.base_frame,
                    encoding='rgb8',
                ))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f'overlay render failed: {exc}', throttle_duration_sec=5.0,
            )

    def _on_timer(self) -> None:
        missing = self._missing_inputs()
        if missing:
            if not self._waiting_logged:
                self.get_logger().info(f'Waiting for: {", ".join(missing)}')
                self._waiting_logged = True
            return

        self._waiting_logged = False
        if self._inference_error is not None:
            return

        t0 = time.perf_counter()
        try:
            if self._inference is None and not self._model_loading_logged:
                self.get_logger().info(
                    f'Loading {self._cfg.policy} model '
                    '(first inference may take ~30s) ...',
                )
                self._model_loading_logged = True

            rgb = image_to_bgr(self._rgb)
            depth = depth_to_meters(self._depth, self._cfg.depth_encoding)
            result, goal_rel = self._run_inference(rgb, depth)

            trajectory = result.trajectory
            if trajectory.ndim == 3:
                trajectory = trajectory[0]

            if self._cfg.markers_use_latest_tf:
                stamp = Header().stamp
            elif self._odom is not None:
                stamp = self._odom.header.stamp
            else:
                stamp = self.get_clock().now().to_msg()
            markers_frame = self._cfg.markers_frame or self._cfg.base_frame
            path_frame = self._cfg.path_frame or self._cfg.base_frame

            all_traj = result.all_trajectory
            values = result.values
            self._overlay_all_traj = all_traj
            self._overlay_values = values

            candidates = None
            if all_traj is not None and np.asarray(all_traj).size > 0:
                arr = np.asarray(all_traj)
                candidates = arr[0] if arr.ndim == 4 else arr

            path = trajectory_to_path(trajectory, stamp, path_frame)
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

            goal_marker = None
            if self._goal is not None and self._odom is not None:
                goal_marker = goal_to_body_pose(
                    self._goal, self._odom, markers_frame, stamp,
                )
            self._markers_pub.publish(build_navdp_markers(
                Header(stamp=stamp, frame_id=markers_frame),
                selected=trajectory,
                candidates=candidates,
                values=values,
                goal=goal_marker,
                lookahead_index=self._cfg.lookahead_index,
                lifetime_sec=self._cfg.marker_lifetime_sec,
                show_all_samples=self._cfg.markers_show_all_samples,
            ))

            if not self._cfg.overlay_decouple and result.trajectory_mask is not None:
                self._overlay_pub.publish(bgr_array_to_image_msg(
                    result.trajectory_mask,
                    self._rgb.header.stamp,
                    self._rgb.header.frame_id or self._cfg.base_frame,
                    encoding='rgb8',
                ))

            goal_msg = ''
            if goal_rel is not None:
                if self._goal_type == 'pixel':
                    goal_msg = f', pixel=({goal_rel[0]:.0f},{goal_rel[1]:.0f})'
                else:
                    goal_msg = f', goal_body=({goal_rel[0]:.2f},{goal_rel[1]:.2f})'
            end = trajectory[-1] if len(trajectory) else (0.0, 0.0, 0.0)
            n_samples = len(candidates) if candidates is not None else 0
            critic_msg = ''
            if values is not None and np.asarray(values).size > 0:
                flat = np.asarray(values, dtype=np.float64).reshape(-1)
                critic_msg = (
                    f', samples={n_samples}, critic=[{flat.min():.2f},{flat.max():.2f}]'
                )
            stop_msg = ', STOP' if result.stopped else ''
            infer_ms = (time.perf_counter() - t0) * 1000.0
            self.get_logger().info(
                f'{self._cfg.policy} plan: selected {len(path.poses)} pts{goal_msg}'
                f'{critic_msg}{stop_msg}, end_body=({end[0]:.2f},{end[1]:.2f}), '
                f'cmd=({cmd.linear.x:.2f},{cmd.angular.z:.2f}), infer={infer_ms:.0f}ms',
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
