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

#ifndef AUTONOMY_ROS__OPTIONS_HPP_
#define AUTONOMY_ROS__OPTIONS_HPP_

#include <string>

#include "autonomy_ros/constants.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy_ros
{

/** @brief Autolink channel names (must match autonomy / autodriver). */
struct AutolinkOptions
{
  std::string map_channel{kAutolinkMapChannel};
  std::string plan_channel{kAutolinkPlanChannel};
  std::string cmd_vel_channel{kAutolinkCmdVelChannel};
  std::string odom_channel{kAutolinkOdomChannel};
  std::string navigate_to_pose{kAutolinkNavigateToPose};
  std::string navigate_through_poses{kAutolinkNavigateThroughPoses};
};

/** @brief ROS topic names (egress / viz). */
struct RosOptions
{
  std::string map_topic{kMapTopic};
  std::string plan_topic{kPlanTopic};
  std::string cmd_vel_topic{kCmdVelTopic};
  std::string odom_topic{kOdomTopic};
};

/** @brief Navigation trigger / visualization. */
struct NavigationOptions
{
  double waypoint_timeout_sec{120.0};
  std::string goal_pose_topic{kGoalPoseTopic};
  std::string waypoints_topic{kWaypointsTopic};
  std::string visualization_frame_id{kOdomTopic};
};

struct Options
{
  AutolinkOptions autolink;
  RosOptions ros;
  NavigationOptions navigation;
};

Options CreateOptions(rclcpp::Node & node);

}  // namespace autonomy_ros

#endif  // AUTONOMY_ROS__OPTIONS_HPP_
