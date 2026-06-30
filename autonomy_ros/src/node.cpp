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

#include "autonomy_ros/node.hpp"

#include <chrono>

#include "autonomy_ros/bridge.hpp"
#include "autonomy_ros/server.hpp"
#include "autonomy_ros/conversions/conversions.hpp"
#include "autonomy_ros/diagnostic.hpp"
#include "autonomy_ros/visualizer.hpp"

#include "autonomy/sensor/dispatchable.hpp"
#include "autonomy/system/options.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace autonomy_ros
{

RosAutonomySystem::RosAutonomySystem(rclcpp::Node & node, Options options)
: node_(node)
, options_(std::move(options))
{
  visualizer_ = std::make_unique<Visualizer>(
    node_, options_.navigation.visualization_frame_id);

  const auto & core_opts = options_.core;
  if (core_opts.config_directory.empty()) {
    RCLCPP_ERROR(
      node_.get_logger(),
      "autonomy.config_directory is empty; set it in navigation_stack.launch.py "
      "(or pass core_config_directory:=<path>).");
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
    core_->Configure(ToRuntimeOptions(core_opts));
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

  navigation_service_ = std::make_unique<NavigationService>(
    node_, *core_, options_.core, options_.navigation,
    [this]() {
      if (bridge_) {
        bridge_->PublishCmdVelZero();
      }
    },
    visualizer_.get());

  StartBridges();

  if (options_.ros.publish_diagnostics) {
    diagnostic_publisher_ = std::make_unique<DiagnosticPublisher>(
      node_,
      [this]() {
        SystemHealthSnapshot snapshot;
        snapshot.core_running = running_ && core_ != nullptr;
        snapshot.static_map_loaded = core_ != nullptr && core_->GetMapServer() != nullptr;
        if (navigation_service_) {
          snapshot.odometry_received = navigation_service_->HasOdometry();
          snapshot.controller_enabled = navigation_service_->IsControllerEnabled();
        }
        return snapshot;
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
  control_timer_.reset();
  navigation_service_.reset();
  diagnostic_publisher_.reset();
  bridge_.reset();
  if (!running_) {
    return;
  }
  running_ = false;
  if (core_) {
    core_->RequestCancelNavigation();
    core_->SetControllerEnabled(false);
  }
  if (core_) {
    core_->Shutdown();
    core_.reset();
  }
}

void RosAutonomySystem::StartBridges()
{
  RosBridge::OdomCallback on_odom;
  if (core_) {
    on_odom = [this](const nav_msgs::msg::Odometry::SharedPtr & msg) {
      core_->GetSensorCollator().AddSensorData(
        ::autonomy::sensor::MakeOdometryData(fromRos(*msg)));
      if (navigation_service_) {
        navigation_service_->UpdateOdom(*msg);
      }
      if (visualizer_) {
        visualizer_->OnRobotPose(*msg);
      }
    };
  }

  RosBridge::ScanCallback on_scan;
  if (core_ && options_.ros.scan_enabled) {
    on_scan = [this](const sensor_msgs::msg::LaserScan::SharedPtr & msg) {
      core_->GetSensorCollator().AddSensorData(
        ::autonomy::sensor::MakeLaserScanData(
          fromRos(*msg), "global_costmap"));
    };
  }

  bridge_ = std::make_unique<RosBridge>(
    node_,
    core_->GetMapServer(),
    core_->GetGlobalCostmap().get(),
    options_,
    std::move(on_odom),
    std::move(on_scan));

  if (core_->GetMapServer()) {
    core_->AddMapPublishListener(
      [this](const ::autonomy::commsgs::map_msgs::OccupancyGrid::SharedPtr & map) {
        if (bridge_) {
          bridge_->PublishMap(map);
        }
      });
    core_->GetMapServer()->PublishMap();
  }

  core_->AddPathListener(
    [this](const ::autonomy::commsgs::planning_msgs::Path & path) {
      if (visualizer_) {
        visualizer_->OnGlobalPath(toRos(path));
      }
    });

  control_timer_ = node_.create_wall_timer(
    std::chrono::milliseconds(50), [this]() { RunControl(); });
}

void RosAutonomySystem::RunControl()
{
  if (!running_ || !core_ || !bridge_) {
    return;
  }
  bridge_->PublishCmdVel(core_->TickControl());
}

}  // namespace autonomy_ros
