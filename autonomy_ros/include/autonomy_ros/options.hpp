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

#ifndef AUTONOMY_ROS__OPTIONS_HPP_
#define AUTONOMY_ROS__OPTIONS_HPP_

#include <string>

#include "autonomy/tasks/options.hpp"
#include "autonomy_ros/constants.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy_ros
{

/** @brief autonomy core configuration (Lua + planner/controller). */
struct CoreOptions
{
  std::string config_directory;
  std::string config_file{"autonomy.lua"};

  bool enable_bt_tasks{true};
  bool use_bt_navigation{true};

  std::string planner_id;
  std::string controller_id{"FollowPath"};
  std::string goal_checker_id{"goal_checker"};
  std::string progress_checker_id{"progress_checker"};
  std::string global_frame{"map"};
  double goal_tolerance{0.15};

  std::string base_frame{"base_footprint"};
  double max_linear_vel{0.22};
};

/** @brief ROS bridge topics and feature toggles. */
struct RosOptions
{
  std::string map_topic{kMapTopic};
  bool publish_map{true};
  std::string tf_topic{kTfTopic};
  std::string tf_static_topic{kTfStaticTopic};

  std::string odom_topic{kOdomTopic};
  std::string cmd_vel_topic{kCmdVelTopic};

  bool scan_enabled{true};
  std::string scan_topic{kScanTopic};

  bool publish_costmaps{true};
  double costmap_publish_hz{1.0};

  bool publish_diagnostics{true};
};

/** @brief Navigation server and visualization parameters. */
struct NavigationOptions
{
  double waypoint_timeout_sec{120.0};
  std::string init_pose_topic{kInitialPoseTopic};
  std::string goal_pose_topic{kGoalPoseTopic};
  std::string visualization_frame_id{kOdomTopic};
};

struct Options
{
  CoreOptions core;
  RosOptions ros;
  NavigationOptions navigation;
};

/** @brief Declare and read all autonomy_ros parameters from @p node. */
Options CreateOptions(rclcpp::Node & node);

/** @brief Map core options to autonomy task runtime options. */
::autonomy::tasks::RuntimeOptions ToRuntimeOptions(const CoreOptions & core);

}  // namespace autonomy_ros

#endif  // AUTONOMY_ROS__OPTIONS_HPP_
