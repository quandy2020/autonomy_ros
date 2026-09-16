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

#ifndef AUTONOMY_ROS__BRIDGE_HPP_
#define AUTONOMY_ROS__BRIDGE_HPP_

#include <functional>
#include <memory>

#include "autolink/node/node.hpp"
#include "autolink/node/reader.hpp"
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include "autonomy_ros/options.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy_ros
{

/**
 * @brief Process-isolated bridge: autonomy → ROS via autolink (no libautonomy).
 *
 * Egress only: /map, /plan, /cmd_vel, /odom (viz).
 * /odom /scan enter autonomy from autodriver, not from ROS.
 */
class AutolinkBridge
{
public:
  using PathCallback = std::function<void(const nav_msgs::msg::Path &)>;
  using OdomCallback = std::function<void(const nav_msgs::msg::Odometry &)>;

  AutolinkBridge(
    rclcpp::Node & node,
    std::shared_ptr<autolink::Node> al_node,
    const Options & options,
    PathCallback on_path = {},
    OdomCallback on_odom = {});

  std::shared_ptr<autolink::Node> AlNode() const { return al_node_; }

private:
  rclcpp::Node & node_;
  std::shared_ptr<autolink::Node> al_node_;
  Options options_;
  PathCallback on_path_;
  OdomCallback on_odom_;

  std::shared_ptr<autolink::Reader<automsgs::msgs::map_msgs::OccupancyGrid>> map_reader_;
  std::shared_ptr<autolink::Reader<automsgs::msgs::nav_msgs::Path>> plan_reader_;
  std::shared_ptr<autolink::Reader<automsgs::msgs::geometry_msgs::TwistStamped>>
    cmd_vel_reader_;
  std::shared_ptr<autolink::Reader<automsgs::msgs::nav_msgs::Odometry>> odom_reader_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

}  // namespace autonomy_ros

#endif  // AUTONOMY_ROS__BRIDGE_HPP_
