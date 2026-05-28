/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
 * email: quandy2020@126.com
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

#include "autonomy_ros/bridge.hpp"

#include <algorithm>
#include <chrono>
#include <functional>

#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/geometry_msgs/transform_stamped.h"
#include "autonomy_ros/constants.hpp"
#include "autonomy_ros/conversions/conversions.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace autonomy_ros
{

namespace
{

::geometry_msgs::TransformStamped ToInternalTransform(
  const ::autonomy::commsgs::geometry_msgs::TransformStamped & from)
{
  ::geometry_msgs::TransformStamped internal;
  internal.header.stamp =
    static_cast<uint64_t>(from.header.stamp.sec) * 1000000000ULL +
    static_cast<uint64_t>(from.header.stamp.nanosec);
  internal.header.frame_id = from.header.frame_id;
  internal.child_frame_id = from.child_frame_id;
  internal.transform.translation.x = from.transform.translation.x;
  internal.transform.translation.y = from.transform.translation.y;
  internal.transform.translation.z = from.transform.translation.z;
  internal.transform.rotation.x = from.transform.rotation.x;
  internal.transform.rotation.y = from.transform.rotation.y;
  internal.transform.rotation.z = from.transform.rotation.z;
  internal.transform.rotation.w = from.transform.rotation.w;
  return internal;
}

void InjectTransform(
  rclcpp::Node & node, const geometry_msgs::msg::TransformStamped & tf, bool is_static)
{
  const auto internal = ToInternalTransform(fromRos(tf));
  try {
    ::autonomy::transform::Buffer::Instance()->setTransform(
      internal, "autonomy_ros", is_static);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      node.get_logger(), *node.get_clock(), 5000,
      "[ros_bridge] setTransform failed: %s", e.what());
  }
}

}  // namespace

RosBridge::RosBridge(
  rclcpp::Node & node,
  ::autonomy::map::MapServer * map_server,
  ::autonomy::map::costmap_2d::Costmap2DWrapper * costmap_wrapper,
  const Options & options,
  OdomCallback on_odom,
  ScanCallback on_scan)
: node_(node)
, map_server_(map_server)
, costmap_wrapper_(costmap_wrapper)
, core_options_(options.core)
, ros_options_(options.ros)
, base_frame_(core_options_.base_frame)
, max_linear_vel_(core_options_.max_linear_vel)
, publish_map_(ros_options_.publish_map)
{
  ::autonomy::transform::Buffer::Instance()->Init();

  tf_sub_ = node_.create_subscription<tf2_msgs::msg::TFMessage>(
    ros_options_.tf_topic, rclcpp::QoS(100),
    [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) { OnTf(msg, false); });
  tf_static_sub_ = node_.create_subscription<tf2_msgs::msg::TFMessage>(
    ros_options_.tf_static_topic, rclcpp::QoS(1).transient_local(),
    [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) { OnTf(msg, true); });

  if (on_odom) {
    odom_sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
      ros_options_.odom_topic, 10,
      [cb = std::move(on_odom)](const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (msg) {
          cb(msg);
        }
      });
  }

  if (on_scan && ros_options_.scan_enabled) {
    scan_sub_ = node_.create_subscription<sensor_msgs::msg::LaserScan>(
      ros_options_.scan_topic, rclcpp::SensorDataQoS(),
      [cb = std::move(on_scan)](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (msg) {
          cb(msg);
        }
      });
  }

  if (publish_map_) {
    map_pub_ = node_.create_publisher<nav_msgs::msg::OccupancyGrid>(
      ros_options_.map_topic, rclcpp::QoS(1).transient_local());
  }

  if (map_server_) {
    reload_map_srv_ = node_.create_service<std_srvs::srv::Trigger>(
      kReloadMapService,
      std::bind(
        &RosBridge::OnReloadMap, this, std::placeholders::_1, std::placeholders::_2));
    if (const auto map = map_server_->GetStaticMapShared()) {
      PublishMap(map);
    }
  }

  if (costmap_wrapper_ && ros_options_.publish_costmaps) {
    global_costmap_pub_ = node_.create_publisher<nav_msgs::msg::OccupancyGrid>(
      kGlobalCostmapTopic, rclcpp::QoS(1).transient_local());
    local_costmap_pub_ = node_.create_publisher<nav_msgs::msg::OccupancyGrid>(
      kLocalCostmapTopic, rclcpp::QoS(1).transient_local());
    const int period_ms = static_cast<int>(
      1000.0 / std::max(ros_options_.costmap_publish_hz, 0.1));
    costmap_timer_ = node_.create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&RosBridge::OnCostmapTimer, this));
  }

  cmd_vel_pub_ = node_.create_publisher<geometry_msgs::msg::TwistStamped>(
    ros_options_.cmd_vel_topic, 10);

  RCLCPP_INFO(
    node_.get_logger(),
    "[ros_bridge] tf=%s/%s odom=%s scan=%s map=%s costmap=%s cmd_vel=%s",
    ros_options_.tf_topic.c_str(), ros_options_.tf_static_topic.c_str(),
    ros_options_.odom_topic.c_str(), ros_options_.scan_topic.c_str(),
    ros_options_.map_topic.c_str(),
    ros_options_.publish_costmaps ? "on" : "off",
    ros_options_.cmd_vel_topic.c_str());
}

void RosBridge::OnTf(const tf2_msgs::msg::TFMessage::SharedPtr msg, const bool is_static)
{
  if (!msg) {
    return;
  }
  for (const auto & tf : msg->transforms) {
    InjectTransform(node_, tf, is_static);
  }
}

void RosBridge::OnCostmapTimer()
{
  if (!costmap_wrapper_) {
    return;
  }
  costmap_wrapper_->updateMap();

  ::autonomy::commsgs::map_msgs::OccupancyGrid grid;
  if (!costmap_wrapper_->snapshotOccupancyGrid(grid)) {
    return;
  }
  const auto ros_grid = toRos(grid);
  if (global_costmap_pub_) {
    global_costmap_pub_->publish(ros_grid);
  }
  if (local_costmap_pub_) {
    local_costmap_pub_->publish(ros_grid);
  }
}

void RosBridge::PublishMap(
  const ::autonomy::commsgs::map_msgs::OccupancyGrid::SharedPtr & map)
{
  if (!publish_map_ || !map_pub_ || !map) {
    return;
  }
  const auto expected =
    static_cast<size_t>(map->info.width) * static_cast<size_t>(map->info.height);
  if (map->data.size() != expected) {
    RCLCPP_WARN(
      node_.get_logger(),
      "[ros_bridge] skip map publish: data size %zu != %u*%u",
      map->data.size(), map->info.width, map->info.height);
    return;
  }
  auto ros_map = toRos(*map);
  ros_map.header.stamp = node_.now();
  map_pub_->publish(ros_map);
}

void RosBridge::PublishCmdVel(const ::autonomy::commsgs::geometry_msgs::TwistStamped & cmd)
{
  if (!cmd_vel_pub_) {
    return;
  }
  auto ros_cmd = toRos(cmd);
  ros_cmd.header.frame_id = base_frame_;
  if (max_linear_vel_ > 0.0) {
    ros_cmd.twist.linear.x =
      std::clamp(ros_cmd.twist.linear.x, -max_linear_vel_, max_linear_vel_);
  }
  cmd_vel_pub_->publish(ros_cmd);
}

void RosBridge::PublishCmdVelZero()
{
  if (!cmd_vel_pub_) {
    return;
  }
  geometry_msgs::msg::TwistStamped stop;
  stop.header.stamp = node_.now();
  stop.header.frame_id = base_frame_;
  cmd_vel_pub_->publish(stop);
}

void RosBridge::OnReloadMap(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> & /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!map_server_) {
    response->success = false;
    response->message = "map_server not available";
    return;
  }
  if (!map_server_->ReloadMap()) {
    response->success = false;
    response->message = "ReloadMap failed";
    return;
  }
  map_server_->PublishMap();
  response->success = true;
  response->message = "map reloaded";
}

}  // namespace autonomy_ros
