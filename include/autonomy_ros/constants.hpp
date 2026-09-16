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

#ifndef AUTONOMY_ROS__CONSTANTS_HPP_
#define AUTONOMY_ROS__CONSTANTS_HPP_

namespace autonomy_ros
{

// Must match autonomy / autolink channel names (process-isolated bridge).
inline constexpr const char kAutolinkMapChannel[] = "/map";
inline constexpr const char kAutolinkPlanChannel[] = "/plan";
inline constexpr const char kAutolinkCmdVelChannel[] = "/cmd_vel";
// /odom from autodriver → autonomy; autonomy_ros only mirrors for RViz
inline constexpr const char kAutolinkOdomChannel[] = "/odom";
inline constexpr const char kAutolinkNavigateToPose[] = "/navigate_to_pose";
inline constexpr const char kAutolinkNavigateThroughPoses[] =
  "/navigate_through_poses";

// ROS external API (Nav2-style names on the ROS side)
inline constexpr const char kNavigateToPoseAction[] = "navigate_to_pose";
inline constexpr const char kNavigateThroughPosesAction[] =
  "navigate_through_poses";
inline constexpr const char kSetInitialPoseService[] = "set_initial_pose";
inline constexpr const char kCancelTaskService[] = "cancel_task";

// ROS convenience / visualization topics
inline constexpr const char kGoalPoseTopic[] = "goal_pose";
inline constexpr const char kWaypointsTopic[] = "waypoints";
inline constexpr const char kPlanTopic[] = "plan";
inline constexpr const char kNavigationGoalTopic[] = "navigation_goal";
inline constexpr const char kRobotPoseTopic[] = "robot_pose";
inline constexpr const char kMapTopic[] = "map";
inline constexpr const char kOdomTopic[] = "odom";
inline constexpr const char kCmdVelTopic[] = "cmd_vel";
inline constexpr const char kTfTopic[] = "tf";
inline constexpr const char kTfStaticTopic[] = "tf_static";

}  // namespace autonomy_ros

#endif  // AUTONOMY_ROS__CONSTANTS_HPP_
