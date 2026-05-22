// Copyright 2025 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTONOMY_ROS__CONSTANTS_HPP_
#define AUTONOMY_ROS__CONSTANTS_HPP_

#include <cstddef>

namespace autonomy_ros::constants
{

// ---------------------------------------------------------------------------
// ROS graph: topics, action_names, service_names
// ---------------------------------------------------------------------------

namespace topics
{
inline constexpr const char kInitialPose[] = "initialpose";
inline constexpr const char kInitPose[] = "init_pose";
inline constexpr const char kGoalPose[] = "goal_pose";
inline constexpr const char kFollowDetections[] = "/detections_3d";
inline constexpr const char kPlan[] = "plan";
inline constexpr const char kGlobalCostmap[] = "global_costmap";
inline constexpr const char kLocalCostmap[] = "local_costmap";
inline constexpr const char kDiagnostics[] = "/diagnostics";
inline constexpr const char kScan[] = "/scan";
inline constexpr const char kSpeedLimit[] = "autonomy/speed_limit";
inline constexpr const char kTaskStatus[] = "autonomy/status";
inline constexpr const char kBatteryStatus[] = "autonomy/battery";
inline constexpr const char kEvents[] = "autonomy/events";
inline constexpr const char kMap[] = "map";
inline constexpr const char kOdom[] = "odom";
inline constexpr const char kCmdVel[] = "cmd_vel";
inline constexpr const char kTf[] = "/tf";
inline constexpr const char kTfStatic[] = "/tf_static";
}  // namespace topics

namespace action_names
{
inline constexpr const char kNavigatePose[] = "autonomy/navigate_pose";
inline constexpr const char kNavigateThrough[] = "autonomy/navigate_through";
inline constexpr const char kFollow[] = "autonomy/follow";
inline constexpr const char kGuidedTour[] = "autonomy/guided_tour";
inline constexpr const char kDock[] = "autonomy/dock";
inline constexpr const char kTeleop[] = "autonomy/teleop";
}  // namespace action_names

namespace service_names
{
inline constexpr const char kCancelTask[] = "autonomy/cancel_task";
inline constexpr const char kGetTaskStatus[] = "autonomy/get_task_status";
inline constexpr const char kPauseTask[] = "autonomy/pause_task";
inline constexpr const char kResumeTask[] = "autonomy/resume_task";
inline constexpr const char kContinueTour[] = "autonomy/continue_tour";
inline constexpr const char kSkipToExhibit[] = "autonomy/skip_to_exhibit";
inline constexpr const char kTriggerEstop[] = "autonomy/trigger_estop";
inline constexpr const char kSetInitialPose[] = "autonomy/set_initial_pose";
inline constexpr const char kSetTeleopMode[] = "autonomy/set_teleop_mode";
inline constexpr const char kListDocks[] = "autonomy/list_docks";
}  // namespace service_names

// ---------------------------------------------------------------------------
// ROS declare_parameter / get_parameter keys
// ---------------------------------------------------------------------------

namespace params
{
// command.*
inline constexpr const char kCommandDefaultDockId[] = "command.default_dock_id";
inline constexpr const char kCommandWaypointTimeoutSec[] = "command.waypoint_timeout_sec";
inline constexpr const char kCommandInitPoseTopic[] = "command.init_pose_topic";
inline constexpr const char kCommandGoalPoseTopic[] = "command.goal_pose_topic";
inline constexpr const char kCommandEnableFollowDetections[] = "command.enable_follow_detections";
inline constexpr const char kCommandFollowDetectionsTopic[] = "command.follow_detections_topic";
inline constexpr const char kCommandDockX[] = "command.dock_x";
inline constexpr const char kCommandDockY[] = "command.dock_y";
inline constexpr const char kCommandDockW[] = "command.dock_w";

// autonomy.*
inline constexpr const char kAutonomyConfigDirectory[] = "autonomy.config_directory";
inline constexpr const char kAutonomyConfigFile[] = "autonomy.config_file";
inline constexpr const char kAutonomyEnableBtTasks[] = "autonomy.enable_bt_tasks";
inline constexpr const char kAutonomyUseBtNavigation[] = "autonomy.use_bt_navigation";
inline constexpr const char kAutonomyPlannerId[] = "autonomy.planner.planner_id";
inline constexpr const char kAutonomyControllerId[] = "autonomy.controller.controller_id";
inline constexpr const char kAutonomyGoalCheckerId[] = "autonomy.controller.goal_checker_id";
inline constexpr const char kAutonomyProgressCheckerId[] = "autonomy.controller.progress_checker_id";
inline constexpr const char kAutonomyGlobalFrame[] = "autonomy.planner.global_frame";
inline constexpr const char kAutonomyGoalTolerance[] = "autonomy.controller.goal_tolerance";
inline constexpr const char kAutonomyEnableScanBridge[] = "autonomy.enable_scan_bridge";
inline constexpr const char kAutonomyScanTopic[] = "autonomy.scan_topic";
inline constexpr const char kAutonomyPublishCostmaps[] = "autonomy.publish_costmaps";
inline constexpr const char kAutonomyCostmapPublishHz[] = "autonomy.costmap_publish_hz";
inline constexpr const char kAutonomyPublishDiagnostics[] = "autonomy.publish_diagnostics";
inline constexpr const char kAutonomyEnableSpeedLimitTopic[] = "autonomy.enable_speed_limit_topic";
inline constexpr const char kAutonomySpeedLimitTopic[] = "autonomy.speed_limit_topic";
inline constexpr const char kAutonomySpeedLimitPercentage[] = "autonomy.speed_limit_percentage";
}  // namespace params

// ---------------------------------------------------------------------------
// Default values, QoS depths, timers (non-ROS-name literals)
// ---------------------------------------------------------------------------

namespace defaults
{
// command
inline constexpr const char kCommandDockId[] = "dock_main";
inline constexpr double kCommandWaypointTimeoutSec = 120.0;
inline constexpr bool kCommandFollowDetectionsEnabled = true;
inline constexpr double kCommandDockX = 0.5;
inline constexpr double kCommandDockY = 0.0;
inline constexpr double kCommandDockW = 1.0;
inline constexpr const char kDockType[] = "charger";
inline constexpr const char kDockFrame[] = "odom";

// autonomy
inline constexpr const char kAutonomyLuaConfig[] = "autonomy.lua";
inline constexpr bool kAutonomyEnableBtTasks = true;
inline constexpr bool kAutonomyUseBtNavigation = true;
inline constexpr const char kAutonomyControllerId[] = "FollowPath";
inline constexpr const char kAutonomyGoalCheckerId[] = "goal_checker";
inline constexpr const char kAutonomyProgressCheckerId[] = "progress_checker";
inline constexpr const char kAutonomyGlobalFrame[] = "map";
inline constexpr double kAutonomyGoalTolerance = 0.15;
inline constexpr const char kAutonomyFallbackPlannerId[] = "navfn_planner";
inline constexpr bool kAutonomyScanEnabled = true;
inline constexpr bool kAutonomyCostmapsEnabled = true;
inline constexpr double kAutonomyCostmapPublishHz = 1.0;
inline constexpr bool kAutonomyDiagnosticsEnabled = true;
inline constexpr bool kAutonomySpeedLimitEnabled = true;
inline constexpr bool kAutonomySpeedLimitPercentage = false;
inline constexpr double kAutonomyClearedSpeedLimit = 0.0;
inline constexpr std::size_t kAutonomyMinPathPoses = 2;
inline constexpr const char kPkgAutonomy[] = "autonomy";
inline constexpr const char kPkgConfigSubpath[] = "/config";

// shared QoS / timing
inline constexpr std::size_t kQueueDepth = 10;
inline constexpr std::size_t kInitialPosePubDepth = 1;
inline constexpr std::size_t kCostmapPubDepth = 1;
inline constexpr int kSpinRateHz = 10;
inline constexpr int kControlTimerMs = 50;
inline constexpr int kBtWaitPollMs = 50;
inline constexpr int kWaypointWaitPollMs = 100;
inline constexpr int kTourFeedbackPollMs = 200;
inline constexpr int kDiagnosticsTimerSec = 1;
inline constexpr double kFollowDetectionStaleSec = 2.0;
inline constexpr double kDirectNavDefaultTimeoutSec = 300.0;
inline constexpr double kCostmapHzToMs = 1000.0;
inline constexpr double kCostmapMinHz = 0.1;
}  // namespace defaults

// ---------------------------------------------------------------------------
// Task ids, error text, control-flow tags, diagnostics labels
// ---------------------------------------------------------------------------

namespace msg
{
// task ids
inline constexpr const char kTaskTeleopSrv[] = "teleop_srv";
inline constexpr const char kTaskGoalPose[] = "goal_pose";
inline constexpr const char kTaskDockSuffix[] = "_dock";

// API / result strings
inline constexpr const char kEmpty[] = "";
inline constexpr const char kOk[] = "ok";
inline constexpr const char kCanceled[] = "canceled";
inline constexpr const char kPreempted[] = "preempted";
inline constexpr const char kBusy[] = "busy";
inline constexpr const char kCancelFailed[] = "cancel failed";
inline constexpr const char kUnknownTaskId[] = "unknown task_id";
inline constexpr const char kNoActiveTask[] = "no active task";
inline constexpr const char kCannotResume[] = "cannot resume";
inline constexpr const char kEstopOrBusy[] = "estop or busy";
inline constexpr const char kGoalPoseTimeout[] = "goal_pose";
inline constexpr const char kNavigatePoseTimeout[] = "navigate_pose";
inline constexpr const char kPerActionBtUnsupported[] =
  "per-action behavior_tree is not supported; set default BT in tasks lua";
inline constexpr const char kEmptyWaypoints[] = "empty waypoints";
inline constexpr const char kWaypointTimeoutOrPreempted[] = "timeout or preempted";
inline constexpr const char kWaypointFailed[] = "waypoint failed";
inline constexpr const char kNoTarget[] = "no target";
inline constexpr const char kTargetLostDetail[] =
  "detection stream missing or target id not found";
inline constexpr const char kNoExhibits[] = "no exhibits";
inline constexpr const char kLowBatteryDock[] = "low battery dock";
inline constexpr const char kStagingFailed[] = "staging";
inline constexpr const char kAlignFailed[] = "align";
inline constexpr const char kEventAutoDock[] = "auto dock";

// std::runtime_error tags (catch by e.what())
inline constexpr const char kSigCancel[] = "cancel";
inline constexpr const char kSigPreempt[] = "preempt";
inline constexpr const char kSigWpFail[] = "wp_fail";
inline constexpr const char kSigReached[] = "reached";
inline constexpr const char kSigTargetLost[] = "target_lost";
inline constexpr const char kSigTimeout[] = "timeout";
inline constexpr const char kSigLowBatteryDock[] = "low_battery_dock";

// /diagnostics status
inline constexpr const char kDiagCore[] = "autonomy_ros/core";
inline constexpr const char kDiagOdom[] = "autonomy_ros/odom";
inline constexpr const char kDiagMap[] = "autonomy_ros/map";
inline constexpr const char kDiagController[] = "autonomy_ros/controller";
inline constexpr const char kDiagCoreRunning[] = "core running";
inline constexpr const char kDiagCoreNotRunning[] = "core not running";
inline constexpr const char kDiagOdomAvailable[] = "odometry available";
inline constexpr const char kDiagOdomMissing[] = "no odometry";
inline constexpr const char kDiagMapLoaded[] = "static map loaded";
inline constexpr const char kDiagMapMissing[] = "static map missing";
inline constexpr const char kDiagControllerEnabled[] = "controller enabled";
inline constexpr const char kDiagControllerIdle[] = "controller idle";
}  // namespace msg

}  // namespace autonomy_ros::constants

#endif  // AUTONOMY_ROS__CONSTANTS_HPP_
