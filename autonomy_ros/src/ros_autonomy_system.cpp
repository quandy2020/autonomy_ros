// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autonomy_ros/ros_autonomy_system.hpp"

#include "autonomy_ros/bridge/actuation_bridge.hpp"
#include "autonomy_ros/bridge/costmap_bridge.hpp"
#include "autonomy_ros/bridge/map_bridge.hpp"
#include "autonomy_ros/bridge/sensor_bridge.hpp"
#include "autonomy_ros/bridge/tf_bridge.hpp"
#include "autonomy_ros/command/command_interface.hpp"
#include "autonomy_ros/constants.hpp"
#include "autonomy_ros/conversions/conversions.hpp"
#include "autonomy_ros/debug/diagnostics_publisher.hpp"
#include "autonomy_ros/options.hpp"
#include "autonomy_ros/visualization/visualizer.hpp"

#include "autonomy/sensor/internal/dispatchable.hpp"
#include "autonomy/system/options.hpp"

namespace autonomy_ros
{

RosAutonomySystem::RosAutonomySystem(
  rclcpp::Node & node,
  AutonomyCoreOptions core_options,
  AutonomyRosOptions ros_options)
: node_(node)
, options_{std::move(core_options), std::move(ros_options)}
{
  visualizer_ = std::make_unique<visualization::Visualizer>(node_);

  const auto & core_opts = options_.core_options;
  if (core_opts.config_directory.empty()) {
    RCLCPP_ERROR(
      node_.get_logger(),
      "autonomy.config_directory is empty; set it in autonomy_stack.launch.py "
      "(or pass autonomy_config_directory:=<path>).");
    return;
  }

  try {
    const auto sys_options = ::autonomy::system::CreateOptions(
      core_opts.config_directory, core_opts.config_file);
    core_ = ::autonomy::system::CreateAutonomy(sys_options);
    if (!core_) {
      RCLCPP_ERROR(node_.get_logger(), "[autonomy] core system failed to start");
      return;
    }
    core_->Start();
    core_->Configure(toRuntimeOptions(core_opts));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_.get_logger(), "[autonomy] failed to load or start core: %s", e.what());
    core_.reset();
    return;
  }

  if (core_opts.enable_bt_tasks && !core_->IsReady()) {
    RCLCPP_WARN(node_.get_logger(), "[autonomy] BT navigators attach failed");
  } else if (core_->IsReady()) {
    RCLCPP_INFO(node_.get_logger(), "[autonomy] BT navigators attached to core");
  }

  running_ = true;
  startBridges();

  command_interface_ = std::make_unique<command::CommandInterface>(
    node_, *core_, options_.core_options,
    [this]() {
      if (actuation_bridge_) {
        actuation_bridge_->publishZero();
      }
    },
    options_.ros_options.cmd_vel_teleop_topic,
    visualizer_.get());

  if (options_.ros_options.publish_diagnostics) {
    diagnostics_ = std::make_unique<debug::DiagnosticsPublisher>(
      node_,
      [this]() {
        debug::DiagnosticsSnapshot snap;
        snap.core_running = running_ && core_ != nullptr;
        if (command_interface_) {
          snap.odometry_received = command_interface_->hasOdometry();
          snap.navigation_active = command_interface_->hasActiveNavigationTask();
          snap.controller_enabled = command_interface_->isControllerEnabled();
        }
        snap.static_map_loaded = core_ && core_->GetMapServer() != nullptr;
        return snap;
      });
  }

  RCLCPP_INFO(
    node_.get_logger(),
    "[autonomy] core ready (config=%s/%s planner=%s controller=%s frame=%s bt=%s)",
    core_opts.config_directory.c_str(), core_opts.config_file.c_str(),
    core_opts.planner_id.c_str(), core_opts.controller_id.c_str(),
    core_opts.global_frame.c_str(),
    (core_->UseBehaviorTreeNavigation()) ? "on" : "off");
}

RosAutonomySystem::~RosAutonomySystem()
{
  command_interface_.reset();
  diagnostics_.reset();
  if (!running_) {
    return;
  }
  running_ = false;
  if (core_) {
    core_->RequestCancelNavigation();
    core_->SetControllerEnabled(false);
  }
  sensor_bridge_.reset();
  costmap_bridge_.reset();
  actuation_bridge_.reset();
  map_bridge_.reset();
  tf_bridge_.reset();
  if (core_) {
    core_->Shutdown();
    core_.reset();
  }
}

void RosAutonomySystem::startBridges()
{
  const auto & ros_opts = options_.ros_options;

  tf_bridge_ = std::make_unique<bridge::TfBridge>(node_, ros_opts);

  costmap_bridge_ = std::make_unique<bridge::CostmapBridge>(
    node_, core_->GetGlobalCostmap(), ros_opts);

  if (core_->GetMapServer()) {
    map_bridge_ = std::make_unique<bridge::MapBridge>(
      node_, core_->GetMapServer(), ros_opts);
    core_->AddMapPublishListener(
      [this](const ::autonomy::commsgs::map_msgs::OccupancyGrid::SharedPtr & map) {
        if (map_bridge_) {
          map_bridge_->publishFromCore(map);
        }
      });
    core_->GetMapServer()->PublishMap();
  }

  core_->AddPathListener(
    [this](const ::autonomy::commsgs::planning_msgs::Path & path) {
      if (visualizer_) {
        visualizer_->onGlobalPath(conversions::toRos(path));
      }
    });

  bridge::SensorBridgeHandlers sensor_handlers;
  sensor_handlers.on_odom = [this](const nav_msgs::msg::Odometry::SharedPtr & msg) {
    if (!msg || !core_) {
      return;
    }
    core_->GetSensorCollator().AddSensorData(
      ::autonomy::sensor::MakeOdometryData(conversions::fromRos(*msg)));
    if (command_interface_) {
      command_interface_->updateOdom(*msg);
    }
    if (visualizer_) {
      visualizer_->onRobotPose(*msg);
    }
  };
  sensor_handlers.on_laser_scan = [this](
      const ::autonomy::commsgs::sensor_msgs::LaserScan & scan) {
    core_->GetSensorCollator().AddSensorData(
      ::autonomy::sensor::MakeLaserScanData(scan, "global_costmap"));
  };
  const auto & sensor_bridge_options = ros_opts.sensor_bridge;
  if (sensor_bridge_options.point_cloud_enabled) {
    sensor_handlers.on_point_cloud = [this](
        const ::autonomy::commsgs::sensor_msgs::PointCloud2 & cloud) {
      core_->GetSensorCollator().AddSensorData(
        ::autonomy::sensor::MakePointCloud2Data(cloud));
    };
  }
  if (sensor_bridge_options.range_enabled) {
    sensor_handlers.on_range = [this](const ::autonomy::commsgs::sensor_msgs::Range & range) {
      core_->GetSensorCollator().AddSensorData(
        ::autonomy::sensor::MakeRangeData(range));
    };
  }
  sensor_bridge_ = std::make_unique<bridge::SensorBridge>(
    node_, ros_opts, std::move(sensor_handlers));

  actuation_bridge_ = std::make_unique<bridge::ActuationBridge>(
    node_, ros_opts, options_.core_options);
  actuation_bridge_->startControlLoop(
    constants::defaults::kControlTimerMs, [this]() { runControl(); });
}

void RosAutonomySystem::runControl()
{
  if (!running_ || !core_ || !actuation_bridge_) {
    return;
  }
  if (command_interface_) {
    if (auto teleop = command_interface_->teleopCommand()) {
      actuation_bridge_->publish(*teleop);
      return;
    }
  }
  actuation_bridge_->publish(core_->TickControl());
}

}  // namespace autonomy_ros
