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

"""ROS 2 node bridging Habitat + Nav2 to LeRobot."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool, Trigger

from autonomy_lerobot.config import Config, load
from autonomy_lerobot.conversions import odom_to_state, pose_to_state
from autonomy_lerobot.observation import Latest, build_frame
from autonomy_lerobot.recorder import DatasetRecorder

_MAP_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class BridgeNode(Node):
    """Subscribe to Habitat and Nav2 topics; record LeRobot dataset frames."""

    def __init__(self) -> None:
        super().__init__('lerobot_bridge_node')
        self._cfg = load(self)
        self._latest = Latest()
        self._recording = False
        self._record_fatal: str | None = None
        self._subscribe_all(self._cfg)

        self._recorder = DatasetRecorder(
            repo_id=self._cfg.dataset_repo_id,
            fps=self._cfg.record_fps,
            root=self._cfg.dataset_root,
            logger=self.get_logger(),
            video_vcodec=self._cfg.video_vcodec,
            streaming_encoding=self._cfg.streaming_encoding,
            parallel_video_encoding=self._cfg.parallel_video_encoding,
            overwrite_dataset=self._cfg.overwrite_dataset,
        )
        if self._cfg.record_fps > 0.0:
            self.create_timer(1.0 / self._cfg.record_fps, self._on_record_tick)

        service_cb = ReentrantCallbackGroup()
        self.create_service(
            SetBool, '~/set_recording', self._on_set_recording,
            callback_group=service_cb)
        self.create_service(
            Trigger, '~/save_episode', self._on_save_episode,
            callback_group=service_cb)
        self.get_logger().info(
            f'[lerobot] recording at {self._cfg.record_fps:.1f} Hz -> {self._cfg.dataset_repo_id}')

    def _subscribe_all(self, cfg: Config) -> None:
        qos = qos_profile_sensor_data
        self.create_subscription(Image, cfg.rgb_topic, self._on_rgb, qos)
        self.create_subscription(Odometry, cfg.odom_topic, self._on_odom, 10)
        self.create_subscription(Twist, cfg.cmd_vel_topic, self._on_cmd_vel, 10)
        self._cmd_vel_pub = self.create_publisher(Twist, cfg.cmd_vel_topic, 10)

        if cfg.use_depth:
            self.create_subscription(Image, cfg.depth_topic, self._on_depth, qos)
        if cfg.record_semantic:
            self.create_subscription(Image, cfg.semantic_topic, self._on_semantic, qos)
        if cfg.record_camera_info:
            self.create_subscription(
                CameraInfo, cfg.camera_info_topic, self._on_camera_info, qos)
        if cfg.record_map:
            self.create_subscription(
                OccupancyGrid, cfg.map_topic, self._on_map, _MAP_QOS)
        if cfg.record_pointcloud:
            self.create_subscription(
                PointCloud2, cfg.pointcloud_topic, self._on_pointcloud, qos)
        if cfg.record_nav2:
            self.create_subscription(Path, cfg.global_plan_topic, self._on_global_plan, 10)
            self._local_plan_topics: set[str] = {cfg.local_plan_topic}
            # DWB / RegulatedPurePursuit publish local_plan; MPPI uses transformed_global_plan.
            self._local_plan_topics.add('local_plan')
            self._local_plan_topics.add('transformed_global_plan')
            for topic in sorted(self._local_plan_topics):
                self.create_subscription(Path, topic, self._on_local_plan, 10)
            self.create_subscription(
                OccupancyGrid, cfg.global_costmap_topic, self._on_global_costmap, _MAP_QOS)
            self.create_subscription(
                OccupancyGrid, cfg.local_costmap_topic, self._on_local_costmap, _MAP_QOS)

    def _on_rgb(self, msg: Image) -> None:
        self._latest.rgb = msg

    def _on_depth(self, msg: Image) -> None:
        self._latest.depth = msg

    def _on_semantic(self, msg: Image) -> None:
        self._latest.semantic = msg

    def _on_odom(self, msg: Odometry) -> None:
        self._latest.odom = msg

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._latest.cmd_vel = msg

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._latest.camera_info = msg

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._latest.map_grid = msg

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        self._latest.pointcloud = msg

    def _on_global_plan(self, msg: Path) -> None:
        self._latest.global_plan = msg

    def _on_local_plan(self, msg: Path) -> None:
        if msg.poses:
            self._latest.local_plan = msg

    def _on_global_costmap(self, msg: OccupancyGrid) -> None:
        self._latest.global_costmap = msg

    def _on_local_costmap(self, msg: OccupancyGrid) -> None:
        self._latest.local_costmap = msg

    def _on_set_recording(self, request: SetBool.Request, response: SetBool.Response):
        if self._recording and not request.data:
            saved = self._recorder.save_episode()
            if saved != 'no frames to save':
                self.get_logger().info(f'auto-saved on stop: {saved}')
            else:
                self.get_logger().info('recording stopped (no buffered frames)')
        elif request.data:
            self._record_fatal = None
            try:
                self._recorder.prepare_for_recording()
            except Exception as exc:
                self._record_fatal = str(exc)
                self.get_logger().error(f'recording disabled: {exc}')
                response.success = False
                response.message = str(exc)
                return response
        self._recording = request.data
        state = 'started' if self._recording else 'stopped'
        self.get_logger().info(
            f'recording {state} (buffered={self._recorder.buffered_frames})')
        response.success = True
        response.message = f'recording {state}'
        return response

    def _on_save_episode(self, _request: Trigger.Request, response: Trigger.Response):
        response.success = True
        response.message = f'saved episode to {self._recorder.save_episode()}'
        self.get_logger().info(response.message)
        return response

    def _on_record_tick(self) -> None:
        if not self._recording or not self._latest.ready():
            return
        if self._record_fatal is not None:
            return
        try:
            self._recorder.add_frame(build_frame(self._latest, self._cfg))
        except (ValueError, KeyError) as exc:
            self.get_logger().warning(f'skip frame: {exc}')
        except Exception as exc:
            self._record_fatal = str(exc)
            self.get_logger().error(f'recording disabled: {exc}')
            self._recording = False
            self._recorder.prepare_for_recording()

    def get_observation(self) -> dict:
        """Return the latest observation for policy inference."""
        obs: dict = {}
        if self._latest.odom is not None:
            obs['state'] = odom_to_state(self._latest.odom).tolist()
        if self._latest.rgb is not None:
            obs['image_height'] = self._latest.rgb.height
            obs['image_width'] = self._latest.rgb.width
        if self._latest.agent_pose is not None:
            obs['agent_pose'] = pose_to_state(self._latest.agent_pose).tolist()
        return obs

    def send_action(self, action: Float32MultiArray | Twist) -> None:
        """Publish cmd_vel from a LeRobot policy output."""
        twist = action if isinstance(action, Twist) else Twist()
        if isinstance(action, Float32MultiArray):
            data = action.data
            if len(data) >= 1:
                twist.linear.x = float(data[0])
            if len(data) >= 2:
                twist.angular.z = float(data[1])
        self._latest.cmd_vel = twist
        self._cmd_vel_pub.publish(twist)

    def destroy_node(self) -> bool:
        self._recorder.finalize()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
