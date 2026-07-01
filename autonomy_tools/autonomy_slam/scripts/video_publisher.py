#!/usr/bin/env python3
"""Publish frames from an MP4 file as sensor_msgs/Image."""

import sys

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Empty


class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.declare_parameter('video_path', '')
        self.declare_parameter('topic', 'camera/image_raw')
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('frame_skip', 1)
        self.declare_parameter('realtime', True)
        self.declare_parameter('loop', False)
        self.declare_parameter('use_backpressure', True)
        self.declare_parameter('frame_processed_topic', 'frame_processed')

        video_path = self.get_parameter('video_path').get_parameter_value().string_value
        if not video_path:
            raise RuntimeError('video_path parameter is required')

        self._topic = self.get_parameter('topic').get_parameter_value().string_value
        self._frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self._frame_skip = max(1, self.get_parameter('frame_skip').get_parameter_value().integer_value)
        self._loop = self.get_parameter('loop').get_parameter_value().bool_value
        self._realtime = self.get_parameter('realtime').get_parameter_value().bool_value
        self._use_backpressure = self.get_parameter('use_backpressure').get_parameter_value().bool_value
        fps = self.get_parameter('fps').get_parameter_value().double_value
        publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        ack_topic = self.get_parameter('frame_processed_topic').get_parameter_value().string_value

        if publish_rate_hz <= 0.0:
            publish_rate_hz = fps if fps > 0.0 else 30.0
        if not self._realtime:
            publish_rate_hz = 0.0
        self._publish_period = (1.0 / publish_rate_hz) if publish_rate_hz > 0.0 else 0.0
        self._publish_rate_hz = publish_rate_hz

        self._cap = cv2.VideoCapture(video_path)
        if not self._cap.isOpened():
            raise RuntimeError(f'Cannot open video: {video_path}')

        # Backpressure needs reliable delivery; sensor_data (BEST_EFFORT) can drop the
        # first frame before DDS matching completes and deadlock the pipeline.
        if self._use_backpressure:
            image_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        else:
            image_qos = qos_profile_sensor_data
        self._pub = self.create_publisher(Image, self._topic, image_qos)
        ack_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._last_msg = None
        self._pending_ack = False
        self._last_publish_time = None
        self._rate_timer = None
        if self._use_backpressure:
            self._ack_sub = self.create_subscription(
                Empty, ack_topic, self._on_frame_processed, ack_qos)
            self._waiting_for_subscriber = True
            self._bootstrap_timer = self.create_timer(0.2, self._try_bootstrap_publish)
            self.get_logger().info(
                f'Backpressure mode: publish next frame after {ack_topic}')
        elif self._publish_period > 0.0:
            self._timer = self.create_timer(self._publish_period, self._on_timer)
        else:
            self._timer = self.create_timer(0.01, self._on_timer)

        rate_desc = f'{self._publish_rate_hz:.1f} Hz' if self._publish_period > 0.0 else 'max speed'
        self.get_logger().info(
            f'Publishing {video_path} -> {self._topic} '
            f'(frame_skip={self._frame_skip}, rate={rate_desc}, '
            f'backpressure={self._use_backpressure})')

    def _read_frame(self):
        frame = None
        for _ in range(self._frame_skip):
            ok, frame = self._cap.read()
            if not ok:
                if self._loop:
                    self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ok, frame = self._cap.read()
                if not ok:
                    return None
        return frame

    def _cancel_rate_timer(self):
        if self._rate_timer is not None:
            self._rate_timer.cancel()
            self._rate_timer = None

    def _schedule_next_publish(self):
        if self._publish_period <= 0.0:
            self._publish_one_frame()
            return
        now = self.get_clock().now()
        delay = self._publish_period
        if self._last_publish_time is not None:
            elapsed = (now - self._last_publish_time).nanoseconds * 1e-9
            delay = max(0.0, self._publish_period - elapsed)
        if delay <= 1e-6:
            self._publish_one_frame()
            return
        self._cancel_rate_timer()
        self._rate_timer = self.create_timer(delay, self._rate_publish_once)

    def _rate_publish_once(self):
        self._cancel_rate_timer()
        self._publish_one_frame()

    def _publish_one_frame(self):
        frame = self._read_frame()
        if frame is None:
            self.get_logger().info('End of video')
            self._cancel_rate_timer()
            if hasattr(self, '_timer'):
                self._timer.cancel()
            if hasattr(self, '_bootstrap_timer'):
                self._bootstrap_timer.cancel()
            rclpy.shutdown()
            return

        msg = Image()
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = frame.strides[0]
        msg.data = frame.tobytes()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        self._last_msg = msg
        self._pending_ack = True
        self._last_publish_time = self.get_clock().now()
        self._pub.publish(msg)

    def _try_bootstrap_publish(self):
        if not self._waiting_for_subscriber:
            return
        if self._pub.get_subscription_count() == 0:
            return
        self._waiting_for_subscriber = False
        self._bootstrap_timer.cancel()
        self.get_logger().info('run_slam subscribed, publishing first frame in 0.5s')
        self._bootstrap_publish_timer = self.create_timer(
            0.5, self._bootstrap_publish_once)

    def _bootstrap_publish_once(self):
        self._bootstrap_publish_timer.cancel()
        self._publish_one_frame()

    def _on_frame_processed(self, _msg):
        self._pending_ack = False
        self._schedule_next_publish()

    def _on_timer(self):
        self._publish_one_frame()


def main():
    rclpy.init(args=sys.argv)
    node = VideoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
