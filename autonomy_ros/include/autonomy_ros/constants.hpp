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

// Topics
inline constexpr const char kInitialPoseTopic[] = "initialpose";
inline constexpr const char kGoalPoseTopic[] = "goal_pose";
inline constexpr const char kPlanTopic[] = "plan";
inline constexpr const char kNavigationGoalTopic[] = "navigation_goal";
inline constexpr const char kRobotPoseTopic[] = "robot_pose";
inline constexpr const char kGlobalCostmapTopic[] = "global_costmap";
inline constexpr const char kLocalCostmapTopic[] = "local_costmap";
inline constexpr const char kDiagnosticsTopic[] = "diagnostics";
inline constexpr const char kScanTopic[] = "scan";
inline constexpr const char kTaskStatusTopic[] = "status";
inline constexpr const char kEventsTopic[] = "events";
inline constexpr const char kMapTopic[] = "map";
inline constexpr const char kOdomTopic[] = "odom";
inline constexpr const char kCmdVelTopic[] = "cmd_vel";
inline constexpr const char kTfTopic[] = "tf";
inline constexpr const char kTfStaticTopic[] = "tf_static";

// Services
inline constexpr const char kReloadMapService[] = "reload_map";
inline constexpr const char kCancelTaskService[] = "cancel_task";
inline constexpr const char kGetTaskStatusService[] = "get_task_status";
inline constexpr const char kPauseTaskService[] = "pause_task";
inline constexpr const char kResumeTaskService[] = "resume_task";
inline constexpr const char kTriggerEstopService[] = "trigger_estop";
inline constexpr const char kSetInitialPoseService[] = "set_initial_pose";

// Actions
inline constexpr const char kNavigatePoseAction[] = "navigate_pose";
inline constexpr const char kNavigateThroughAction[] = "navigate_through";

}  // namespace autonomy_ros

#endif  // AUTONOMY_ROS__CONSTANTS_HPP_
