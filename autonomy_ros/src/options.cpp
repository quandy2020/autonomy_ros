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

  auto & core = options.core;
  DeclareIfNeeded(node, "autonomy.config_directory", core.config_directory);
  DeclareIfNeeded(node, "autonomy.config_file", core.config_file);
  DeclareIfNeeded(node, "autonomy.enable_bt_tasks", core.enable_bt_tasks);
  DeclareIfNeeded(node, "autonomy.use_bt_navigation", core.use_bt_navigation);
  DeclareIfNeeded(node, "autonomy.planner.planner_id", core.planner_id);
  DeclareIfNeeded(node, "autonomy.controller.controller_id", core.controller_id);
  DeclareIfNeeded(node, "autonomy.controller.goal_checker_id", core.goal_checker_id);
  DeclareIfNeeded(node, "autonomy.controller.progress_checker_id", core.progress_checker_id);
  DeclareIfNeeded(node, "autonomy.planner.global_frame", core.global_frame);
  DeclareIfNeeded(node, "autonomy.controller.goal_tolerance", core.goal_tolerance);
  DeclareIfNeeded(node, "autonomy.controller.base_frame", core.base_frame);
  DeclareIfNeeded(node, "autonomy.controller.max_linear_vel", core.max_linear_vel);

  core.config_directory = node.get_parameter("autonomy.config_directory").as_string();
  core.config_file = node.get_parameter("autonomy.config_file").as_string();
  core.enable_bt_tasks = node.get_parameter("autonomy.enable_bt_tasks").as_bool();
  core.use_bt_navigation = node.get_parameter("autonomy.use_bt_navigation").as_bool();
  core.planner_id = node.get_parameter("autonomy.planner.planner_id").as_string();
  core.controller_id = node.get_parameter("autonomy.controller.controller_id").as_string();
  core.goal_checker_id = node.get_parameter("autonomy.controller.goal_checker_id").as_string();
  core.progress_checker_id =
    node.get_parameter("autonomy.controller.progress_checker_id").as_string();
  core.global_frame = node.get_parameter("autonomy.planner.global_frame").as_string();
  core.goal_tolerance = node.get_parameter("autonomy.controller.goal_tolerance").as_double();
  core.base_frame = node.get_parameter("autonomy.controller.base_frame").as_string();
  core.max_linear_vel = node.get_parameter("autonomy.controller.max_linear_vel").as_double();

  auto & ros = options.ros;
  DeclareIfNeeded(node, "autonomy.map_topic", ros.map_topic);
  DeclareIfNeeded(node, "autonomy.publish_map", ros.publish_map);
  DeclareIfNeeded(node, "autonomy.tf_topic", ros.tf_topic);
  DeclareIfNeeded(node, "autonomy.tf_static_topic", ros.tf_static_topic);
  DeclareIfNeeded(node, "autonomy.odom_topic", ros.odom_topic);
  DeclareIfNeeded(node, "autonomy.cmd_vel_topic", ros.cmd_vel_topic);
  DeclareIfNeeded(node, "autonomy.enable_scan_bridge", ros.scan_enabled);
  DeclareIfNeeded(node, "autonomy.scan_topic", ros.scan_topic);
  DeclareIfNeeded(node, "autonomy.publish_costmaps", ros.publish_costmaps);
  DeclareIfNeeded(node, "autonomy.costmap_publish_hz", ros.costmap_publish_hz);
  DeclareIfNeeded(node, "autonomy.publish_diagnostics", ros.publish_diagnostics);

  ros.map_topic = node.get_parameter("autonomy.map_topic").as_string();
  ros.publish_map = node.get_parameter("autonomy.publish_map").as_bool();
  ros.tf_topic = node.get_parameter("autonomy.tf_topic").as_string();
  ros.tf_static_topic = node.get_parameter("autonomy.tf_static_topic").as_string();
  ros.odom_topic = node.get_parameter("autonomy.odom_topic").as_string();
  ros.cmd_vel_topic = node.get_parameter("autonomy.cmd_vel_topic").as_string();
  ros.scan_enabled = node.get_parameter("autonomy.enable_scan_bridge").as_bool();
  ros.scan_topic = node.get_parameter("autonomy.scan_topic").as_string();
  ros.publish_costmaps = node.get_parameter("autonomy.publish_costmaps").as_bool();
  ros.costmap_publish_hz = node.get_parameter("autonomy.costmap_publish_hz").as_double();
  ros.publish_diagnostics = node.get_parameter("autonomy.publish_diagnostics").as_bool();

  auto & nav = options.navigation;
  DeclareIfNeeded(node, "navigation.waypoint_timeout_sec", nav.waypoint_timeout_sec);
  DeclareIfNeeded(node, "navigation.init_pose_topic", nav.init_pose_topic);
  DeclareIfNeeded(node, "navigation.goal_pose_topic", nav.goal_pose_topic);
  DeclareIfNeeded(node, "visualization.frame_id", nav.visualization_frame_id);

  nav.waypoint_timeout_sec = node.get_parameter("navigation.waypoint_timeout_sec").as_double();
  nav.init_pose_topic = node.get_parameter("navigation.init_pose_topic").as_string();
  nav.goal_pose_topic = node.get_parameter("navigation.goal_pose_topic").as_string();
  nav.visualization_frame_id = node.get_parameter("visualization.frame_id").as_string();

  return options;
}

::autonomy::system::RuntimeOptions ToRuntimeOptions(const CoreOptions & core)
{
  ::autonomy::system::RuntimeOptions opts;
  opts.enable_bt_tasks = core.enable_bt_tasks;
  opts.use_bt_navigation = core.use_bt_navigation;
  opts.config_directory = core.config_directory;
  opts.planner_id = core.planner_id;
  opts.controller_id = core.controller_id;
  opts.goal_checker_id = core.goal_checker_id;
  opts.progress_checker_id = core.progress_checker_id;
  opts.global_frame = core.global_frame;
  opts.robot_base_frame = core.base_frame;
  opts.goal_tolerance = core.goal_tolerance;
  return opts;
}

}  // namespace autonomy_ros
