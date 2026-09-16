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

#include "autonomy_ros/options.hpp"

namespace autonomy_ros
{

namespace
{

template<typename T>
void DeclareIfNeeded(rclcpp::Node & node, const char * name, const T & default_value)
{
  if (!node.has_parameter(name)) {
    node.declare_parameter<T>(name, default_value);
  }
}

}  // namespace

Options CreateOptions(rclcpp::Node & node)
{
  Options options;

  auto & al = options.autolink;
  DeclareIfNeeded(node, "autolink.map_channel", al.map_channel);
  DeclareIfNeeded(node, "autolink.plan_channel", al.plan_channel);
  DeclareIfNeeded(node, "autolink.cmd_vel_channel", al.cmd_vel_channel);
  DeclareIfNeeded(node, "autolink.odom_channel", al.odom_channel);
  DeclareIfNeeded(node, "autolink.navigate_to_pose", al.navigate_to_pose);
  DeclareIfNeeded(
    node, "autolink.navigate_through_poses", al.navigate_through_poses);

  al.map_channel = node.get_parameter("autolink.map_channel").as_string();
  al.plan_channel = node.get_parameter("autolink.plan_channel").as_string();
  al.cmd_vel_channel = node.get_parameter("autolink.cmd_vel_channel").as_string();
  al.odom_channel = node.get_parameter("autolink.odom_channel").as_string();
  al.navigate_to_pose = node.get_parameter("autolink.navigate_to_pose").as_string();
  al.navigate_through_poses =
    node.get_parameter("autolink.navigate_through_poses").as_string();

  auto & ros = options.ros;
  DeclareIfNeeded(node, "ros.map_topic", ros.map_topic);
  DeclareIfNeeded(node, "ros.plan_topic", ros.plan_topic);
  DeclareIfNeeded(node, "ros.cmd_vel_topic", ros.cmd_vel_topic);
  DeclareIfNeeded(node, "ros.odom_topic", ros.odom_topic);

  ros.map_topic = node.get_parameter("ros.map_topic").as_string();
  ros.plan_topic = node.get_parameter("ros.plan_topic").as_string();
  ros.cmd_vel_topic = node.get_parameter("ros.cmd_vel_topic").as_string();
  ros.odom_topic = node.get_parameter("ros.odom_topic").as_string();

  auto & nav = options.navigation;
  DeclareIfNeeded(node, "navigation.waypoint_timeout_sec", nav.waypoint_timeout_sec);
  DeclareIfNeeded(node, "navigation.goal_pose_topic", nav.goal_pose_topic);
  DeclareIfNeeded(node, "navigation.waypoints_topic", nav.waypoints_topic);
  DeclareIfNeeded(node, "visualization.frame_id", nav.visualization_frame_id);

  nav.waypoint_timeout_sec =
    node.get_parameter("navigation.waypoint_timeout_sec").as_double();
  nav.goal_pose_topic = node.get_parameter("navigation.goal_pose_topic").as_string();
  nav.waypoints_topic = node.get_parameter("navigation.waypoints_topic").as_string();
  nav.visualization_frame_id =
    node.get_parameter("visualization.frame_id").as_string();

  return options;
}

}  // namespace autonomy_ros
