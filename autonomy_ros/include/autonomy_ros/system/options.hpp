// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTONOMY_ROS__SYSTEM__OPTIONS_HPP_
#define AUTONOMY_ROS__SYSTEM__OPTIONS_HPP_

#include <string>
#include <tuple>

#include "autonomy/tasks/options.hpp"
#include "autonomy_ros/system/constants.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy_ros::system
{

/**
 * @brief Non-ROS configuration for the autonomy core (Lua + planner/controller IDs).
 *
 * Loaded from autonomy.config_* and autonomy.planner/controller.* parameters.
 * Passed to ::autonomy::system::CreateOptions() after config_directory is set.
 */
struct AutonomyCoreOptions
{
  std::string config_directory;
  std::string config_file{constants::defaults::kAutonomyLuaConfig};

  bool enable_bt_tasks{constants::defaults::kAutonomyEnableBtTasks};
  bool use_bt_navigation{constants::defaults::kAutonomyUseBtNavigation};

  std::string planner_id;
  std::string controller_id{constants::defaults::kAutonomyControllerId};
  std::string goal_checker_id{constants::defaults::kAutonomyGoalCheckerId};
  std::string progress_checker_id{constants::defaults::kAutonomyProgressCheckerId};
  std::string global_frame{constants::defaults::kAutonomyGlobalFrame};
  double goal_tolerance{constants::defaults::kAutonomyGoalTolerance};

  std::string base_frame{constants::defaults::kAutonomyBaseFrameDefault};
  double max_linear_vel{constants::defaults::kAutonomyMaxLinearVel};
};

/** @brief Optional ROS sensor ingress beyond odom / laser (see autonomy.sensor_bridge.*). */
struct SensorBridgeOptions
{
  bool point_cloud_enabled{false};
  std::string point_cloud_topic{"/points"};

  bool range_enabled{false};
  std::string range_topic{"/range"};
};

/**
 * @brief ROS graph and bridge configuration (topics, publishers, node toggles).
 *
 * Loaded from autonomy.* ROS parameters (see config/parameters.yaml).
 */
struct AutonomyRosOptions
{
  std::string map_topic{constants::topics::kMap};
  bool publish_map{true};
  std::string tf_topic{constants::topics::kTf};
  std::string tf_static_topic{constants::topics::kTfStatic};

  std::string odom_topic{constants::topics::kOdom};
  std::string cmd_vel_topic{constants::topics::kCmdVel};

  bool scan_enabled{constants::defaults::kAutonomyScanEnabled};
  std::string scan_topic{constants::topics::kScan};

  bool publish_costmaps{constants::defaults::kAutonomyCostmapsEnabled};
  double costmap_publish_hz{constants::defaults::kAutonomyCostmapPublishHz};

  bool publish_diagnostics{constants::defaults::kAutonomyDiagnosticsEnabled};

  SensorBridgeOptions sensor_bridge;
};

/**
 * @brief Autonomy options (core and ROS).
 */
struct AutonomyOptions
{
  /**
   * @brief Autonomy core options.
   */
  AutonomyCoreOptions core_options;

  /**
   * @brief Autonomy ROS options.
   */
  AutonomyRosOptions ros_options;
};

/** @brief Declare and read autonomy core parameters from @p node. */
AutonomyCoreOptions loadAutonomyCoreOptions(rclcpp::Node & node);

/** @brief Declare and read ROS bridge / I/O parameters from @p node. */
AutonomyRosOptions loadAutonomyRosOptions(rclcpp::Node & node);

/**
 * @brief Declare and read all autonomy_ros parameters from @p node (single entry point).
 * @return {AutonomyCoreOptions, AutonomyRosOptions}
 */
std::tuple<AutonomyCoreOptions, AutonomyRosOptions> createOptions(rclcpp::Node & node);

/** @brief Map ROS core parameters to autonomy task runtime options. */
::autonomy::tasks::RuntimeOptions toRuntimeOptions(const AutonomyCoreOptions & core);

}  // namespace autonomy_ros::system

#endif  // AUTONOMY_ROS__SYSTEM__OPTIONS_HPP_
