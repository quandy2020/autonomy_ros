/*
 * Copyright 2026 autonomy_ros contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file
 * @brief Publishes time-aligned autodriver snapshots as ROS 2 sensor topics.
 */

#ifndef AUTONOMY_DRIVER_ROS_SNAPSHOT_PUBLISHER_HPP_
#define AUTONOMY_DRIVER_ROS_SNAPSHOT_PUBLISHER_HPP_

#include <memory>
#include <string>

#include "autodriver/sync/aligned_snapshot.hpp"
#include "autodriver/types/camera_frame.hpp"
#include "autodriver/types/sensor_sample.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace autonomy_driver {

/**
 * @struct RosSnapshotTopicNames
 * @brief Output topic names; empty strings skip that modality.
 */
struct RosSnapshotTopicNames
{
  std::string imu;
  std::string gps;
  std::string odom;
  std::string laser;
  std::string range;
  std::string camera_color;
  std::string camera_depth;
};

/**
 * @class RosSnapshotPublisher
 * @brief Converts AlignedSnapshot samples to ROS messages and publishes them.
 */
class RosSnapshotPublisher
{
public:
  RosSnapshotPublisher(
    rclcpp::Node * node,
    RosSnapshotTopicNames topics);

  void Publish(const autodriver::AlignedSnapshot & snapshot);

  /** @brief Publishes high-rate samples (e.g. multiple camera streams). */
  void PublishRaw(const autodriver::SensorSample & sample);

private:
  void PublishCameraFrame(const autodriver::CameraFrame & frame);

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_depth_pub_;
};

}  // namespace autonomy_driver

#endif  // AUTONOMY_DRIVER_ROS_SNAPSHOT_PUBLISHER_HPP_
