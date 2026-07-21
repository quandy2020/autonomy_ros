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
 * @brief Implements ROS 2 topic sensor drivers.
 */

#include "autonomy_driver/ros_sensor_drivers.hpp"

#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "autodriver/common/time.hpp"
#include "autodriver/types/camera_frame.hpp"
#include "autodriver/types/gps_sample.hpp"
#include "autodriver/types/imu_sample.hpp"
#include "autodriver/types/lidar_scan.hpp"
#include "autodriver/types/range_sample.hpp"
#include "autodriver/types/wheel_odometry_sample.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace autonomy_driver {
namespace {

autodriver::Timestamp FromRosTime(const builtin_interfaces::msg::Time & stamp)
{
  const int64_t nanos =
    static_cast<int64_t>(stamp.sec) * 1'000'000'000LL +
    static_cast<int64_t>(stamp.nanosec);
  return autodriver::FromNanoseconds(nanos);
}

autodriver::SensorId ResolveSensorId(
  const autodriver::SensorId & sensor_id,
  const std::string & frame_id)
{
  if (!sensor_id.empty()) {
    return sensor_id;
  }
  return frame_id.empty() ? autodriver::SensorId{"ros_sensor"} : frame_id;
}

class RosSensorDriverBase : public autodriver::SensorDriver
{
public:
  RosSensorDriverBase(
    autodriver::SensorType type,
    autodriver::SensorId sensor_id)
  : type_(type),
    sensor_id_(std::move(sensor_id))
  {
  }

  autodriver::SensorType GetType() const override { return type_; }
  const autodriver::SensorId & GetSensorId() const override { return sensor_id_; }

  bool Start() override
  {
    running_ = true;
    return true;
  }

  void Stop() override { running_ = false; }

  bool IsRunning() const override { return running_; }

  void SetSampleCallback(SampleCallback callback) override
  {
    callback_ = std::move(callback);
  }

protected:
  void Emit(std::unique_ptr<autodriver::SensorSample> sample)
  {
    if (running_ && callback_ && sample) {
      callback_(std::move(sample));
    }
  }

  autodriver::SensorType type_;
  autodriver::SensorId sensor_id_;
  SampleCallback callback_;
  bool running_{false};
};

class RosImuDriver : public RosSensorDriverBase
{
public:
  RosImuDriver(
    rclcpp::Node * node,
    const std::string & topic,
    autodriver::SensorId sensor_id)
  : RosSensorDriverBase(autodriver::SensorType::kImu, std::move(sensor_id)),
    sub_(node->create_subscription<sensor_msgs::msg::Imu>(
        topic, rclcpp::SensorDataQoS(),
        std::bind(&RosImuDriver::OnMessage, this, std::placeholders::_1)))
  {
  }

private:
  void OnMessage(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    const autodriver::SensorId id = ResolveSensorId(
      sensor_id_, msg->header.frame_id);
    Emit(std::make_unique<autodriver::ImuSample>(
        id,
        FromRosTime(msg->header.stamp),
        std::array<double, 3>{
          msg->angular_velocity.x,
          msg->angular_velocity.y,
          msg->angular_velocity.z},
        std::array<double, 3>{
          msg->linear_acceleration.x,
          msg->linear_acceleration.y,
          msg->linear_acceleration.z}));
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
};

class RosLaserDriver : public RosSensorDriverBase
{
public:
  RosLaserDriver(
    rclcpp::Node * node,
    const std::string & topic,
    autodriver::SensorId sensor_id)
  : RosSensorDriverBase(autodriver::SensorType::kLidar, std::move(sensor_id)),
    sub_(node->create_subscription<sensor_msgs::msg::LaserScan>(
        topic, rclcpp::SensorDataQoS(),
        std::bind(&RosLaserDriver::OnMessage, this, std::placeholders::_1)))
  {
  }

private:
  void OnMessage(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    std::vector<float> angles(msg->ranges.size());
    for (std::size_t i = 0; i < angles.size(); ++i) {
      angles[i] = msg->angle_min + static_cast<float>(i) * msg->angle_increment;
    }

    const autodriver::SensorId id = ResolveSensorId(
      sensor_id_, msg->header.frame_id);
    Emit(std::make_unique<autodriver::LidarScan>(
        id,
        FromRosTime(msg->header.stamp),
        msg->ranges,
        std::move(angles),
        msg->range_min,
        msg->range_max));
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

class RosOdomDriver : public RosSensorDriverBase
{
public:
  RosOdomDriver(
    rclcpp::Node * node,
    const std::string & topic,
    autodriver::SensorId sensor_id)
  : RosSensorDriverBase(
      autodriver::SensorType::kWheelOdometry, std::move(sensor_id)),
    sub_(node->create_subscription<nav_msgs::msg::Odometry>(
        topic, rclcpp::QoS(10),
        std::bind(&RosOdomDriver::OnMessage, this, std::placeholders::_1)))
  {
  }

private:
  void OnMessage(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    const autodriver::SensorId id = ResolveSensorId(
      sensor_id_, msg->child_frame_id.empty() ? msg->header.frame_id :
      msg->child_frame_id);
    Emit(std::make_unique<autodriver::WheelOdometrySample>(
        id,
        FromRosTime(msg->header.stamp),
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.angular.z));
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

class RosGpsDriver : public RosSensorDriverBase
{
public:
  RosGpsDriver(
    rclcpp::Node * node,
    const std::string & topic,
    autodriver::SensorId sensor_id)
  : RosSensorDriverBase(autodriver::SensorType::kGps, std::move(sensor_id)),
    sub_(node->create_subscription<sensor_msgs::msg::NavSatFix>(
        topic, rclcpp::QoS(10),
        std::bind(&RosGpsDriver::OnMessage, this, std::placeholders::_1)))
  {
  }

private:
  void OnMessage(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    autodriver::GpsFixStatus fix = autodriver::GpsFixStatus::kNoFix;
    if (msg->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX) {
      fix = autodriver::GpsFixStatus::kRtkFixed;
    } else if (msg->status.status >=
      sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX)
    {
      fix = autodriver::GpsFixStatus::kRtkFloat;
    } else if (msg->status.status >=
      sensor_msgs::msg::NavSatStatus::STATUS_FIX)
    {
      fix = autodriver::GpsFixStatus::kFix3D;
    }

    const autodriver::SensorId id = ResolveSensorId(
      sensor_id_, msg->header.frame_id);
    Emit(std::make_unique<autodriver::GpsSample>(
        id,
        FromRosTime(msg->header.stamp),
        msg->latitude,
        msg->longitude,
        msg->altitude,
        fix));
  }

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_;
};

class RosRangeDriver : public RosSensorDriverBase
{
public:
  RosRangeDriver(
    rclcpp::Node * node,
    const std::string & topic,
    autodriver::SensorId sensor_id)
  : RosSensorDriverBase(
      autodriver::SensorType::kRangeFinder, std::move(sensor_id)),
    sub_(node->create_subscription<sensor_msgs::msg::Range>(
        topic, rclcpp::QoS(10),
        std::bind(&RosRangeDriver::OnMessage, this, std::placeholders::_1)))
  {
  }

private:
  void OnMessage(const sensor_msgs::msg::Range::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    const autodriver::SensorId id = ResolveSensorId(
      sensor_id_, msg->header.frame_id);
    Emit(std::make_unique<autodriver::RangeSample>(
        id,
        FromRosTime(msg->header.stamp),
        msg->range,
        msg->min_range,
        msg->max_range));
  }

  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_;
};

class RosCameraDriver : public RosSensorDriverBase
{
public:
  RosCameraDriver(
    rclcpp::Node * node,
    const std::string & topic,
    autodriver::SensorId sensor_id)
  : RosSensorDriverBase(autodriver::SensorType::kCamera, std::move(sensor_id)),
    sub_(node->create_subscription<sensor_msgs::msg::Image>(
        topic, rclcpp::SensorDataQoS(),
        std::bind(&RosCameraDriver::OnMessage, this, std::placeholders::_1)))
  {
  }

private:
  void OnMessage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    const autodriver::SensorId id = ResolveSensorId(
      sensor_id_, msg->header.frame_id);
    Emit(std::make_unique<autodriver::CameraFrame>(
        id,
        FromRosTime(msg->header.stamp),
        msg->width,
        msg->height,
        msg->encoding,
        msg->data));
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

}  // namespace

std::shared_ptr<autodriver::SensorDriver> CreateRosDriver(
  rclcpp::Node * node,
  const std::string & type,
  const std::string & topic,
  const autodriver::SensorId & sensor_id)
{
  if (!node) {
    return nullptr;
  }

  if (type == "imu") {
    return std::make_shared<RosImuDriver>(node, topic, sensor_id);
  }
  if (type == "laser" || type == "lidar") {
    return std::make_shared<RosLaserDriver>(node, topic, sensor_id);
  }
  if (type == "odom" || type == "wheel_odom") {
    return std::make_shared<RosOdomDriver>(node, topic, sensor_id);
  }
  if (type == "gps" || type == "navsat") {
    return std::make_shared<RosGpsDriver>(node, topic, sensor_id);
  }
  if (type == "range") {
    return std::make_shared<RosRangeDriver>(node, topic, sensor_id);
  }
  if (type == "camera" || type == "image") {
    return std::make_shared<RosCameraDriver>(node, topic, sensor_id);
  }

  throw std::runtime_error("Unknown ROS driver type: " + type);
}

}  // namespace autonomy_driver
