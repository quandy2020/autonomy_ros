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
 * @brief Implements RosSnapshotPublisher.
 */

#include "autonomy_driver/ros_snapshot_publisher.hpp"

#include "autodriver/bridge/autonomy/sample_converter.hpp"
#include "autodriver/types/camera_frame.hpp"
#include "autodriver/types/gps_sample.hpp"
#include "autodriver/types/imu_sample.hpp"
#include "autodriver/types/lidar_scan.hpp"
#include "autodriver/types/range_sample.hpp"
#include "autodriver/types/sensor_type.hpp"
#include "autodriver/types/wheel_odometry_sample.hpp"
#include "autonomy_ros/conversions/planning_msgs.hpp"
#include "autonomy_ros/conversions/sensor_msgs.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/range.hpp"

#include <algorithm>

namespace autonomy_driver {
namespace {

sensor_msgs::msg::NavSatFix ToRosNavSatFix(
  const automsgs::msgs::sensor_msgs::NavSatFix & from)
{
  sensor_msgs::msg::NavSatFix to;
  to.header.stamp.sec = from.header().stamp().sec();
  to.header.stamp.nanosec = from.header().stamp().nanosec();
  to.header.frame_id = from.header().frame_id();
  to.status.status = static_cast<int8_t>(from.status().status());
  to.status.service = static_cast<uint16_t>(from.status().service());
  to.latitude = from.latitude();
  to.longitude = from.longitude();
  to.altitude = from.altitude();
  if (from.position_covariance_size() >=
    static_cast<int>(to.position_covariance.size()))
  {
    std::copy_n(
      from.position_covariance().begin(),
      to.position_covariance.size(),
      to.position_covariance.begin());
  }
  to.position_covariance_type =
    static_cast<uint8_t>(from.position_covariance_type());
  return to;
}

bool IsDepthFrame(const autodriver::CameraFrame & frame)
{
  return frame.encoding == "16UC1" ||
         frame.sensor_id().find("depth") != std::string::npos;
}

}  // namespace

RosSnapshotPublisher::RosSnapshotPublisher(
  rclcpp::Node * node,
  RosSnapshotTopicNames topics)
{
  if (!topics.imu.empty()) {
    imu_pub_ = node->create_publisher<sensor_msgs::msg::Imu>(
      topics.imu, rclcpp::SensorDataQoS());
  }
  if (!topics.gps.empty()) {
    gps_pub_ = node->create_publisher<sensor_msgs::msg::NavSatFix>(
      topics.gps, rclcpp::SensorDataQoS());
  }
  if (!topics.odom.empty()) {
    odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>(
      topics.odom, rclcpp::SensorDataQoS());
  }
  if (!topics.laser.empty()) {
    laser_pub_ = node->create_publisher<sensor_msgs::msg::LaserScan>(
      topics.laser, rclcpp::SensorDataQoS());
  }
  if (!topics.range.empty()) {
    range_pub_ = node->create_publisher<sensor_msgs::msg::Range>(
      topics.range, rclcpp::SensorDataQoS());
  }
  if (!topics.camera_color.empty()) {
    camera_color_pub_ = node->create_publisher<sensor_msgs::msg::Image>(
      topics.camera_color, rclcpp::SensorDataQoS());
  }
  if (!topics.camera_depth.empty()) {
    camera_depth_pub_ = node->create_publisher<sensor_msgs::msg::Image>(
      topics.camera_depth, rclcpp::SensorDataQoS());
  }
}

void RosSnapshotPublisher::PublishCameraFrame(const autodriver::CameraFrame & frame)
{
  auto image = autodriver::bridge::ToAutonomyImage(frame);
  if (frame.encoding == "16UC1") {
    image.set_step(frame.width * 2);
  } else if (frame.encoding == "mono8") {
    image.set_step(frame.width);
  } else {
    image.set_step(frame.width * 3);
  }

  const auto ros_image = autonomy_ros::toRos(image);
  if (IsDepthFrame(frame)) {
    if (camera_depth_pub_) {
      camera_depth_pub_->publish(ros_image);
    }
  } else if (camera_color_pub_) {
    camera_color_pub_->publish(ros_image);
  }
}

void RosSnapshotPublisher::PublishRaw(const autodriver::SensorSample & sample)
{
  if (sample.type() == autodriver::SensorType::kCamera) {
    const auto * frame = dynamic_cast<const autodriver::CameraFrame *>(&sample);
    if (frame) {
      PublishCameraFrame(*frame);
    }
  }
}

void RosSnapshotPublisher::Publish(const autodriver::AlignedSnapshot & snapshot)
{
  if (imu_pub_) {
    const auto * imu = snapshot.Get<autodriver::ImuSample>(
      autodriver::SensorType::kImu);
    if (imu) {
      imu_pub_->publish(
        autonomy_ros::toRos(autodriver::bridge::ToAutonomyImu(*imu)));
    }
  }

  if (gps_pub_) {
    const auto * gps = snapshot.Get<autodriver::GpsSample>(
      autodriver::SensorType::kGps);
    if (gps) {
      gps_pub_->publish(
        ToRosNavSatFix(autodriver::bridge::ToAutonomyNavSatFix(*gps)));
    }
  }

  if (odom_pub_) {
    const auto * odom = snapshot.Get<autodriver::WheelOdometrySample>(
      autodriver::SensorType::kWheelOdometry);
    if (odom) {
      odom_pub_->publish(
        autonomy_ros::toRos(autodriver::bridge::ToAutonomyOdometry(*odom)));
    }
  }

  if (laser_pub_) {
    const auto * scan = snapshot.Get<autodriver::LidarScan>(
      autodriver::SensorType::kLidar);
    if (scan) {
      laser_pub_->publish(
        autonomy_ros::toRos(autodriver::bridge::ToAutonomyLaserScan(*scan)));
    }
  }

  if (range_pub_) {
    const auto * range = snapshot.Get<autodriver::RangeSample>(
      autodriver::SensorType::kRangeFinder);
    if (range) {
      range_pub_->publish(
        autonomy_ros::toRos(autodriver::bridge::ToAutonomyRange(*range)));
    }
  }

  if (camera_color_pub_ || camera_depth_pub_) {
    const auto * camera = snapshot.Get<autodriver::CameraFrame>(
      autodriver::SensorType::kCamera);
    if (camera) {
      PublishCameraFrame(*camera);
    }
  }
}

}  // namespace autonomy_driver
