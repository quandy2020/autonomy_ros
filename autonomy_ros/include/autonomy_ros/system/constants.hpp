// Copyright 2025 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTONOMY_ROS__SYSTEM__CONSTANTS_HPP_
#define AUTONOMY_ROS__SYSTEM__CONSTANTS_HPP_

#include <cstddef>

namespace autonomy_ros::constants
{

namespace topics
{
inline constexpr const char kInitialPose[] = "initialpose";
inline constexpr const char kInitPose[] = "init_pose";
inline constexpr const char kGoalPose[] = "goal_pose";
inline constexpr const char kPlan[] = "plan";
inline constexpr const char kNavigationGoal[] = "navigation_goal";
inline constexpr const char kRobotPose[] = "robot_pose";
inline constexpr const char kGlobalCostmap[] = "global_costmap";
inline constexpr const char kLocalCostmap[] = "local_costmap";
inline constexpr const char kDiagnostics[] = "/diagnostics";
inline constexpr const char kScan[] = "/scan";
inline constexpr const char kTaskStatus[] = "autonomy/status";
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
}  // namespace action_names

namespace service_names
{
inline constexpr const char kCancelTask[] = "autonomy/cancel_task";
inline constexpr const char kGetTaskStatus[] = "autonomy/get_task_status";
inline constexpr const char kPauseTask[] = "autonomy/pause_task";
inline constexpr const char kResumeTask[] = "autonomy/resume_task";
inline constexpr const char kTriggerEstop[] = "autonomy/trigger_estop";
inline constexpr const char kSetInitialPose[] = "autonomy/set_initial_pose";
}  // namespace service_names

namespace params
{
inline constexpr const char kNavigationWaypointTimeoutSec[] = "navigation.waypoint_timeout_sec";
inline constexpr const char kNavigationInitPoseTopic[] = "navigation.init_pose_topic";
inline constexpr const char kNavigationGoalPoseTopic[] = "navigation.goal_pose_topic";

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
inline constexpr const char kAutonomyOdomTopic[] = "autonomy.odom_topic";
inline constexpr const char kAutonomyCmdVelTopic[] = "autonomy.cmd_vel_topic";
inline constexpr const char kAutonomyBaseFrame[] = "autonomy.controller.base_frame";
inline constexpr const char kAutonomyMaxLinearVel[] = "autonomy.controller.max_linear_vel";
inline constexpr const char kAutonomyEnableScanBridge[] = "autonomy.enable_scan_bridge";
inline constexpr const char kAutonomyScanTopic[] = "autonomy.scan_topic";
inline constexpr const char kAutonomyPublishCostmaps[] = "autonomy.publish_costmaps";
inline constexpr const char kAutonomyCostmapPublishHz[] = "autonomy.costmap_publish_hz";
inline constexpr const char kAutonomyPublishDiagnostics[] = "autonomy.publish_diagnostics";

inline constexpr const char kSensorBridgePointCloudEnabled[] =
  "autonomy.sensor_bridge.point_cloud_enabled";
inline constexpr const char kSensorBridgePointCloudTopic[] =
  "autonomy.sensor_bridge.point_cloud_topic";
inline constexpr const char kSensorBridgeRangeEnabled[] =
  "autonomy.sensor_bridge.range_enabled";
inline constexpr const char kSensorBridgeRangeTopic[] = "autonomy.sensor_bridge.range_topic";
}  // namespace params

namespace defaults
{
inline constexpr double kNavigationWaypointTimeoutSec = 120.0;

inline constexpr const char kAutonomyLuaConfig[] = "autonomy.lua";
inline constexpr bool kAutonomyEnableBtTasks = true;
inline constexpr bool kAutonomyUseBtNavigation = true;
inline constexpr const char kAutonomyControllerId[] = "FollowPath";
inline constexpr const char kAutonomyGoalCheckerId[] = "goal_checker";
inline constexpr const char kAutonomyProgressCheckerId[] = "progress_checker";
inline constexpr const char kAutonomyGlobalFrame[] = "map";
inline constexpr double kAutonomyGoalTolerance = 0.15;
inline constexpr const char kAutonomyBaseFrameDefault[] = "base_footprint";
inline constexpr double kAutonomyMaxLinearVel = 0.22;
inline constexpr const char kAutonomyFallbackPlannerId[] = "navfn_planner";
inline constexpr bool kAutonomyScanEnabled = true;
inline constexpr bool kAutonomyCostmapsEnabled = true;
inline constexpr double kAutonomyCostmapPublishHz = 1.0;
inline constexpr bool kAutonomyDiagnosticsEnabled = true;
inline constexpr const char kPkgAutonomy[] = "autonomy";
inline constexpr const char kPkgConfigSubpath[] = "/config";

inline constexpr std::size_t kQueueDepth = 10;
inline constexpr std::size_t kInitialPosePubDepth = 1;
inline constexpr std::size_t kCostmapPubDepth = 1;
inline constexpr int kControlTimerMs = 50;
inline constexpr int kWaypointWaitPollMs = 100;
inline constexpr int kDiagnosticsTimerSec = 1;
inline constexpr double kCostmapHzToMs = 1000.0;
inline constexpr double kCostmapMinHz = 0.1;
}  // namespace defaults

namespace msg
{
inline constexpr const char kTaskGoalPose[] = "goal_pose";
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

inline constexpr const char kSigCancel[] = "cancel";
inline constexpr const char kSigPreempt[] = "preempt";
inline constexpr const char kSigWpFail[] = "wp_fail";

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

#endif  // AUTONOMY_ROS__SYSTEM__CONSTANTS_HPP_
