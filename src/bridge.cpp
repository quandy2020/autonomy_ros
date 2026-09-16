/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy_ros/conversions/conversions.hpp"

namespace autonomy_ros
{

AutolinkBridge::AutolinkBridge(
  rclcpp::Node & node,
  std::shared_ptr<autolink::Node> al_node,
  const Options & options,
  PathCallback on_path,
  OdomCallback on_odom)
: node_(node)
, al_node_(std::move(al_node))
, options_(options)
, on_path_(std::move(on_path))
, on_odom_(std::move(on_odom))
{
  map_pub_ = node_.create_publisher<nav_msgs::msg::OccupancyGrid>(
    options_.ros.map_topic, rclcpp::QoS(1).transient_local());
  cmd_vel_pub_ = node_.create_publisher<geometry_msgs::msg::TwistStamped>(
    options_.ros.cmd_vel_topic, 10);
  odom_pub_ = node_.create_publisher<nav_msgs::msg::Odometry>(
    options_.ros.odom_topic, 10);

  map_reader_ = al_node_->CreateReader<automsgs::msgs::map_msgs::OccupancyGrid>(
    options_.autolink.map_channel,
    [this](const std::shared_ptr<automsgs::msgs::map_msgs::OccupancyGrid> & msg) {
      if (!msg || !map_pub_) {
        return;
      }
      auto ros_map = toRos(*msg);
      ros_map.header.stamp = node_.now();
      map_pub_->publish(ros_map);
    });

  plan_reader_ = al_node_->CreateReader<automsgs::msgs::nav_msgs::Path>(
    options_.autolink.plan_channel,
    [this](const std::shared_ptr<automsgs::msgs::nav_msgs::Path> & msg) {
      if (!msg || !on_path_) {
        return;
      }
      on_path_(toRos(*msg));
    });

  cmd_vel_reader_ =
    al_node_->CreateReader<automsgs::msgs::geometry_msgs::TwistStamped>(
      options_.autolink.cmd_vel_channel,
      [this](
        const std::shared_ptr<automsgs::msgs::geometry_msgs::TwistStamped> & msg) {
        if (!msg || !cmd_vel_pub_) {
          return;
        }
        cmd_vel_pub_->publish(toRos(*msg));
      });

  // autodriver → autonomy (/odom); mirror to ROS for RViz only
  odom_reader_ = al_node_->CreateReader<automsgs::msgs::nav_msgs::Odometry>(
    options_.autolink.odom_channel,
    [this](const std::shared_ptr<automsgs::msgs::nav_msgs::Odometry> & msg) {
      if (!msg) {
        return;
      }
      const auto ros_odom = toRos(*msg);
      if (odom_pub_) {
        odom_pub_->publish(ros_odom);
      }
      if (on_odom_) {
        on_odom_(ros_odom);
      }
    });

  RCLCPP_INFO(
    node_.get_logger(),
    "[autolink_bridge] egress map=%s plan=%s cmd_vel=%s odom=%s (no ROS→autonomy sensors)",
    options_.autolink.map_channel.c_str(),
    options_.autolink.plan_channel.c_str(),
    options_.autolink.cmd_vel_channel.c_str(),
    options_.autolink.odom_channel.c_str());
}

}  // namespace autonomy_ros
