// Copyright 2026 autonomy_ros contributors
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

#include "autonomy_ros/autonomy.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <chrono>
#include <future>
#include <thread>

#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/planning/common/planner_exceptions.hpp"
#include "autonomy/system/options.hpp"
#include "autonomy/tasks/navigator/proto/action.pb.h"
#include "autonomy/control/controller_server.hpp"
#include "autonomy_ros/conversions/conversions.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace autonomy_ros
{

namespace
{

using CoreNode = ::autonomy::system::AutonomyNode;

geometry_msgs::msg::PoseStamped odomToPoseStamped(
  const nav_msgs::msg::Odometry & odom, const std::string & frame)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = odom.header;
  if (!frame.empty()) {
    pose.header.frame_id = frame;
  }
  pose.pose = odom.pose.pose;
  return pose;
}

}  // namespace

Autonomy::Autonomy(rclcpp::Node & node)
: node_(node)
{
  loadParameters();
}

void Autonomy::loadParameters()
{
  node_.declare_parameter<std::string>("autonomy.config_directory", config_directory_);
  node_.declare_parameter<std::string>("autonomy.config_file", config_file_);
  node_.declare_parameter<bool>("autonomy.enable_bt_tasks", enable_bt_tasks_);
  node_.declare_parameter<bool>("autonomy.use_bt_navigation", use_bt_navigation_);
  node_.declare_parameter<std::string>("autonomy.planner_id", planner_id_);
  node_.declare_parameter<std::string>("autonomy.controller_id", controller_id_);
  node_.declare_parameter<std::string>("autonomy.goal_checker_id", goal_checker_id_);
  node_.declare_parameter<std::string>("autonomy.progress_checker_id", progress_checker_id_);
  node_.declare_parameter<std::string>("planner.global_frame", global_frame_);
  node_.declare_parameter<double>("controller.goal_tolerance", goal_tolerance_);

  config_directory_ = node_.get_parameter("autonomy.config_directory").as_string();
  config_file_ = node_.get_parameter("autonomy.config_file").as_string();
  enable_bt_tasks_ = node_.get_parameter("autonomy.enable_bt_tasks").as_bool();
  use_bt_navigation_ = node_.get_parameter("autonomy.use_bt_navigation").as_bool();
  planner_id_ = node_.get_parameter("autonomy.planner_id").as_string();
  controller_id_ = node_.get_parameter("autonomy.controller_id").as_string();
  goal_checker_id_ = node_.get_parameter("autonomy.goal_checker_id").as_string();
  progress_checker_id_ =
    node_.get_parameter("autonomy.progress_checker_id").as_string();
  global_frame_ = node_.get_parameter("planner.global_frame").as_string();
  goal_tolerance_ = node_.get_parameter("controller.goal_tolerance").as_double();

  node_.declare_parameter<bool>("autonomy.enable_scan_bridge", scan_enabled_);
  node_.declare_parameter<std::string>("autonomy.scan_topic", scan_topic_);
  node_.declare_parameter<bool>("autonomy.publish_costmaps", costmaps_enabled_);
  node_.declare_parameter<double>("autonomy.costmap_publish_hz", costmap_hz_);
  node_.declare_parameter<bool>("autonomy.publish_diagnostics", diagnostics_enabled_);
  node_.declare_parameter<bool>("autonomy.enable_speed_limit_topic", speed_limit_enabled_);
  node_.declare_parameter<std::string>("autonomy.speed_limit_topic", speed_limit_topic_);
  node_.declare_parameter<bool>("autonomy.speed_limit_percentage", speed_limit_percentage_);
  scan_enabled_ = node_.get_parameter("autonomy.enable_scan_bridge").as_bool();
  scan_topic_ = node_.get_parameter("autonomy.scan_topic").as_string();
  costmaps_enabled_ = node_.get_parameter("autonomy.publish_costmaps").as_bool();
  costmap_hz_ = node_.get_parameter("autonomy.costmap_publish_hz").as_double();
  diagnostics_enabled_ = node_.get_parameter("autonomy.publish_diagnostics").as_bool();
  speed_limit_enabled_ = node_.get_parameter("autonomy.enable_speed_limit_topic").as_bool();
  speed_limit_topic_ = node_.get_parameter("autonomy.speed_limit_topic").as_string();
  speed_limit_percentage_ = node_.get_parameter("autonomy.speed_limit_percentage").as_bool();
}

std::string Autonomy::resolveConfigDirectory() const
{
  if (!config_directory_.empty()) {
    return config_directory_;
  }
  try {
    return ament_index_cpp::get_package_share_directory("autonomy") + "/config";
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_.get_logger(),
      "autonomy.config_directory is empty and package 'autonomy' not found: %s",
      e.what());
    return {};
  }
}

void Autonomy::start()
{
  if (running_) {
    return;
  }
  startCore();
  if (!core_) {
    RCLCPP_ERROR(node_.get_logger(), "[autonomy] core system failed to start");
    return;
  }
  startTaskScheduler();
  startRosBridges();
  running_ = true;
  RCLCPP_INFO(
    node_.get_logger(),
    "[autonomy] core ready (config=%s/%s planner=%s controller=%s frame=%s bt=%s)",
    resolveConfigDirectory().c_str(), config_file_.c_str(),
    planner_id_.c_str(), controller_id_.c_str(), global_frame_.c_str(),
    (task_scheduler_ && use_bt_navigation_) ? "on" : "off");
}

void Autonomy::startCore()
{
  const std::string config_dir = resolveConfigDirectory();
  if (config_dir.empty()) {
    return;
  }

  try {
    const auto options =
      ::autonomy::system::CreateOptions(config_dir, config_file_);
    core_ = ::autonomy::system::CreateAutonomy(options);
    if (!core_) {
      return;
    }

    odom_smoother_ = std::make_shared<::autonomy::control::utils::OdomSmoother>();
    if (auto * controller = core_->controller_server()) {
      controller->SetOdomSmoother(odom_smoother_);
    }

    if (planner_id_.empty() && core_->planner_server()) {
      planner_id_ = core_->planner_server()->GetDefaultPlannerId();
    }
    if (planner_id_.empty()) {
      planner_id_ = "navfn_planner";
    }

    core_->Start();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_.get_logger(), "[autonomy] failed to load or start core: %s", e.what());
    core_.reset();
  }
}

void Autonomy::startTaskScheduler()
{
  if (!enable_bt_tasks_ || !core_) {
    return;
  }
  const std::string config_dir = resolveConfigDirectory();
  if (config_dir.empty()) {
    return;
  }

  auto planner = core_->planner_server()
    ? std::shared_ptr<::autonomy::planning::PlannerServer>(
      core_->planner_server(), [](::autonomy::planning::PlannerServer *) {})
    : nullptr;
  auto smoother = core_->smoother_server()
    ? std::shared_ptr<::autonomy::planning::SmootherServer>(
      core_->smoother_server(), [](::autonomy::planning::SmootherServer *) {})
    : nullptr;
  auto controller = core_->controller_server()
    ? std::shared_ptr<::autonomy::control::ControllerServer>(
      core_->controller_server(), [](::autonomy::control::ControllerServer *) {})
    : nullptr;
  if (!core_->task_context()) {
    RCLCPP_WARN(node_.get_logger(), "[autonomy] skip TaskScheduler: task_context missing");
    return;
  }
  auto task_ctx = std::shared_ptr<::autonomy::tasks::common::TaskContext>(
    core_->task_context(), [](::autonomy::tasks::common::TaskContext *) {});

  if (!planner || !controller) {
    RCLCPP_WARN(node_.get_logger(), "[autonomy] skip TaskScheduler: missing core servers");
    return;
  }

  task_scheduler_ = std::make_unique<::autonomy::tasks::scheduler::TaskScheduler>();
  ::autonomy::tasks::scheduler::SharedSystem attachment;
  attachment.planner = planner;
  attachment.smoother = smoother;
  attachment.controller = controller;
  attachment.task_context = task_ctx;
  attachment.odom_smoother = odom_smoother_;
  task_scheduler_->InitializeAttached(config_dir, attachment);

  if (!task_scheduler_->IsInitialized()) {
    RCLCPP_WARN(node_.get_logger(), "[autonomy] TaskScheduler attach failed");
    task_scheduler_.reset();
    return;
  }

  if (auto * ctx = taskContext()) {
    if (!global_frame_.empty()) {
      ctx->global_frame = global_frame_;
    }
    if (!controller_id_.empty()) {
      ctx->selected_controller_id = controller_id_;
    }
    if (!goal_checker_id_.empty()) {
      ctx->selected_goal_checker_id = goal_checker_id_;
    }
  }
  RCLCPP_INFO(node_.get_logger(), "[autonomy] TaskScheduler attached to shared core");
}

::autonomy::map::costmap_2d::Costmap2DWrapper * Autonomy::plannerCostmap()
{
  if (!core_ || !core_->planner_server()) {
    return nullptr;
  }
  return core_->planner_server()->GetCostmapWrapper().get();
}

void Autonomy::applyMapToCostmap(
  const ::autonomy::commsgs::map_msgs::OccupancyGrid::SharedPtr & map)
{
  if (!map) {
    return;
  }
  if (auto * wrapper = plannerCostmap()) {
    wrapper->applyOccupancyGrid(*map);
  }
}

void Autonomy::startRosBridges()
{
  tf_bridge_ = std::make_unique<bridge::TfBridge>(node_);
  tf_bridge_->start();

  map_bridge_ = std::make_unique<bridge::MapBridge>(node_);
  if (core_ && core_->map_server()) {
    map_bridge_->start(core_->map_server());
    core_->map_server()->SetMapPublishCallback(
      [this](const ::autonomy::commsgs::map_msgs::OccupancyGrid::SharedPtr & map) {
        applyMapToCostmap(map);
        if (map_bridge_) {
          map_bridge_->publishFromCore(map);
        }
      });
    core_->map_server()->PublishMap();
  }

  platform_bridge_ = std::make_unique<bridge::PlatformBridge>(node_);
  platform_bridge_->start(
    [this](const nav_msgs::msg::Odometry::SharedPtr & msg) { dispatchOdom(msg); });

  startOutboundIo();

  plan_pub_ = node_.create_publisher<nav_msgs::msg::Path>("plan", 10);
  control_timer_ = node_.create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&Autonomy::controlStep, this));
}

void Autonomy::shutdown()
{
  if (!running_) {
    return;
  }
  running_ = false;
  requestCancelNavigation();
  setControllerEnabled(false);
  control_timer_.reset();
  plan_pub_.reset();
  if (platform_bridge_) {
    platform_bridge_->stop();
    platform_bridge_.reset();
  }
  if (map_bridge_) {
    map_bridge_->stop();
    map_bridge_.reset();
  }
  stopOutboundIo();
  if (tf_bridge_) {
    tf_bridge_->stop();
    tf_bridge_.reset();
  }
  if (task_scheduler_) {
    task_scheduler_->Shutdown();
    task_scheduler_.reset();
  }
  if (core_) {
    core_->Shutdown();
    core_.reset();
  }
  odom_smoother_.reset();
}

CoreNode & Autonomy::core()
{
  return *core_;
}

const CoreNode & Autonomy::core() const
{
  return *core_;
}

::autonomy::tasks::common::TaskContext * Autonomy::taskContext()
{
  return core_ ? core_->task_context() : nullptr;
}

const ::autonomy::tasks::common::TaskContext * Autonomy::taskContext() const
{
  return core_ ? core_->task_context() : nullptr;
}

::autonomy::tasks::scheduler::TaskScheduler * Autonomy::taskScheduler()
{
  return task_scheduler_.get();
}

const ::autonomy::tasks::scheduler::TaskScheduler * Autonomy::taskScheduler() const
{
  return task_scheduler_.get();
}

bool Autonomy::useBehaviorTreeNavigation() const
{
  return use_bt_navigation_ && task_scheduler_ != nullptr &&
         task_scheduler_->IsInitialized();
}

bool Autonomy::hasTaskScheduler() const
{
  return task_scheduler_ != nullptr && task_scheduler_->IsInitialized();
}

std::function<bool()> Autonomy::navigationCancelChecker()
{
  return [this]() {
    return cancel_navigation_.load();
  };
}

bool Autonomy::planToGoal(const geometry_msgs::msg::PoseStamped & goal)
{
  cancel_navigation_.store(false);
  auto * planner = core_ ? core_->planner_server() : nullptr;
  if (!planner || !odom_smoother_ || !odom_smoother_->HasOdometry()) {
    RCLCPP_WARN(node_.get_logger(), "[autonomy] cannot plan: planner or odom missing");
    return false;
  }

  ::autonomy::commsgs::planning_msgs::Odometry odom;
  if (!odom_smoother_->GetLatestOdometry(odom)) {
    RCLCPP_WARN(node_.get_logger(), "[autonomy] cannot plan: no odometry");
    return false;
  }

  nav_msgs::msg::Odometry ros_odom;
  ros_odom.header.stamp = node_.now();
  ros_odom.header.frame_id = odom.header.frame_id;
  ros_odom.child_frame_id = odom.child_frame_id;
  ros_odom.pose.pose.position.x = odom.pose.pose.position.x;
  ros_odom.pose.pose.position.y = odom.pose.pose.position.y;
  ros_odom.pose.pose.position.z = odom.pose.pose.position.z;
  ros_odom.pose.pose.orientation.x = odom.pose.pose.orientation.x;
  ros_odom.pose.pose.orientation.y = odom.pose.pose.orientation.y;
  ros_odom.pose.pose.orientation.z = odom.pose.pose.orientation.z;
  ros_odom.pose.pose.orientation.w = odom.pose.pose.orientation.w;

  const auto start = conversions::fromRos(odomToPoseStamped(ros_odom, global_frame_));
  auto goal_com = conversions::fromRos(goal);
  if (goal_com.header.frame_id.empty()) {
    goal_com.header.frame_id = global_frame_;
  }

  try {
    const auto path = planner->ComputePathToPose(
      start, goal_com, planner_id_, navigationCancelChecker());
    if (path.poses.size() < 2) {
      RCLCPP_WARN(node_.get_logger(), "[autonomy] planner returned empty path");
      return false;
    }

    const auto ros_path = conversions::toRos(path);
    {
      std::lock_guard<std::mutex> lock(path_mutex_);
      last_path_ = ros_path;
    }
    notifyPlan(ros_path);

    auto * controller = core_->controller_server();
    if (!controller) {
      return false;
    }
    if (!controller->BeginFollowPath(
        path, controller_id_, goal_checker_id_, progress_checker_id_))
    {
      RCLCPP_WARN(node_.get_logger(), "[autonomy] BeginFollowPath failed");
      return false;
    }
    following_path_.store(true);
    RCLCPP_INFO(
      node_.get_logger(), "[autonomy] plan ready (%zu poses)", path.poses.size());
    return true;
  } catch (const ::autonomy::planning::common::PlannerException & e) {
    RCLCPP_WARN(node_.get_logger(), "[autonomy] planning failed: %s", e.what());
  } catch (const std::exception & e) {
    RCLCPP_WARN(node_.get_logger(), "[autonomy] planning error: %s", e.what());
  }
  return false;
}

void Autonomy::setNavigationGoal(const geometry_msgs::msg::PoseStamped & goal)
{
  if (!running_ || !core_) {
    RCLCPP_WARN(node_.get_logger(), "[autonomy] setNavigationGoal ignored: not running");
    return;
  }
  if (!planToGoal(goal)) {
    return;
  }
  setControllerEnabled(true);
}

std::optional<nav_msgs::msg::Path> Autonomy::lastPath() const
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  return last_path_;
}

void Autonomy::setControllerEnabled(bool enabled)
{
  controller_enabled_.store(enabled);
  if (!enabled) {
    requestCancelNavigation();
    if (auto * controller = core_ ? core_->controller_server() : nullptr) {
      if (following_path_.load()) {
        controller->EndFollowPath();
        following_path_.store(false);
      }
    }
    if (platform_bridge_) {
      platform_bridge_->publishZeroCmdVel();
    }
  }
}

bool Autonomy::controllerEnabled() const
{
  return controller_enabled_.load();
}

double Autonomy::goalTolerance() const
{
  return goal_tolerance_;
}

void Autonomy::applyControllerSpeedLimit(
  const ::autonomy::commsgs::planning_msgs::SpeedLimit & limit)
{
  if (!core_ || !core_->controller_server()) {
    return;
  }
  core_->controller_server()->ApplySpeedLimit(limit);
}

void Autonomy::clearControllerSpeedLimit()
{
  ::autonomy::commsgs::planning_msgs::SpeedLimit cleared;
  cleared.header.stamp = ::autonomy::commsgs::builtin_interfaces::Time::Now();
  cleared.percentage = false;
  cleared.speed_limit = 0.0;
  applyControllerSpeedLimit(cleared);
}

void Autonomy::startOutboundIo()
{
  if (scan_enabled_) {
    scan_sub_ = node_.create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Autonomy::onScan, this, std::placeholders::_1));
    RCLCPP_INFO(node_.get_logger(), "[outbound] scan %s -> costmap", scan_topic_.c_str());
  }

  if (costmaps_enabled_) {
    global_costmap_pub_ = node_.create_publisher<nav_msgs::msg::OccupancyGrid>(
      "global_costmap", rclcpp::QoS(1).transient_local());
    local_costmap_pub_ = node_.create_publisher<nav_msgs::msg::OccupancyGrid>(
      "local_costmap", rclcpp::QoS(1).transient_local());
    const auto period_ms = static_cast<int>(1000.0 / std::max(costmap_hz_, 0.1));
    costmap_timer_ = node_.create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&Autonomy::publishCostmaps, this));
    RCLCPP_INFO(
      node_.get_logger(), "[outbound] costmaps at %.1f Hz", costmap_hz_);
  }

  if (diagnostics_enabled_) {
    diagnostics_pub_ = node_.create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);
    diagnostics_timer_ = node_.create_wall_timer(
      std::chrono::seconds(1), std::bind(&Autonomy::publishDiagnostics, this));
    RCLCPP_INFO(node_.get_logger(), "[outbound] /diagnostics");
  }

  if (speed_limit_enabled_) {
    speed_limit_sub_ = node_.create_subscription<std_msgs::msg::Float32>(
      speed_limit_topic_, 10,
      std::bind(&Autonomy::onSpeedLimitTopic, this, std::placeholders::_1));
    RCLCPP_INFO(
      node_.get_logger(), "[outbound] speed limit %s", speed_limit_topic_.c_str());
  }
}

void Autonomy::stopOutboundIo()
{
  scan_sub_.reset();
  speed_limit_sub_.reset();
  costmap_timer_.reset();
  diagnostics_timer_.reset();
  global_costmap_pub_.reset();
  local_costmap_pub_.reset();
  diagnostics_pub_.reset();
}

void Autonomy::onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (msg) {
    feedScanToCostmap(conversions::fromRos(*msg));
  }
}

void Autonomy::onSpeedLimitTopic(const std_msgs::msg::Float32::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  ::autonomy::commsgs::planning_msgs::SpeedLimit limit;
  limit.header.stamp = ::autonomy::commsgs::builtin_interfaces::Time::Now();
  limit.percentage = speed_limit_percentage_;
  limit.speed_limit = msg->data;
  applyControllerSpeedLimit(limit);
}

void Autonomy::publishCostmaps()
{
  if (global_costmap_pub_) {
    ::autonomy::commsgs::map_msgs::OccupancyGrid grid;
    if (snapshotCostmap(grid)) {
      global_costmap_pub_->publish(conversions::toRos(grid));
    }
  }
  if (local_costmap_pub_) {
    ::autonomy::commsgs::map_msgs::OccupancyGrid grid;
    if (snapshotCostmap(grid)) {
      local_costmap_pub_->publish(conversions::toRos(grid));
    }
  }
}

void Autonomy::publishDiagnostics()
{
  if (diagnostics_pub_) {
    diagnostics_pub_->publish(buildDiagnostics());
  }
}

bool Autonomy::navigateToPose(
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker,
  const double timeout_sec)
{
  if (useBehaviorTreeNavigation()) {
    cancel_navigation_.store(false);
    task_scheduler_->RequestCancel();

    auto goal_com = conversions::fromRos(goal);
    if (goal_com.header.frame_id.empty()) {
      goal_com.header.frame_id = global_frame_;
    }
    auto proto_goal = std::make_shared<
      ::autonomy::tasks::behavior_tree::proto::NavigateToPoseAction::Goal>();
    *proto_goal->mutable_pose() =
      ::autonomy::commsgs::geometry_msgs::ToProto(goal_com);

    bt_navigation_active_.store(true);
    setControllerEnabled(true);

    std::packaged_task<::autonomy::tasks::behavior_tree::BtStatus()> bt_task(
      [this, proto_goal]() { return task_scheduler_->NavigateToPose(proto_goal); });
    auto bt_future = bt_task.get_future();
    std::thread bt_thread(std::move(bt_task));

    while (bt_future.wait_for(std::chrono::milliseconds(50)) !=
      std::future_status::ready)
    {
      if (cancel_checker && cancel_checker()) {
        task_scheduler_->RequestCancel();
      }
      if (!rclcpp::ok()) {
        task_scheduler_->RequestCancel();
        break;
      }
    }

    const auto status = bt_future.get();
    bt_thread.join();
    bt_navigation_active_.store(false);

    if (cancel_checker && cancel_checker()) {
      return false;
    }
    return status == ::autonomy::tasks::behavior_tree::BtStatus::SUCCEEDED;
  }

  setNavigationGoal(goal);
  return waitForDirectNavigation(goal, goal_tolerance_, cancel_checker, timeout_sec);
}

void Autonomy::requestCancelNavigation()
{
  cancel_navigation_.store(true);
  if (task_scheduler_) {
    task_scheduler_->RequestCancel();
  }
}

void Autonomy::addOdomListener(
  std::function<void(const nav_msgs::msg::Odometry::SharedPtr &)> listener)
{
  if (!listener) {
    return;
  }
  std::lock_guard<std::mutex> lock(odom_mutex_);
  odom_listeners_.push_back(std::move(listener));
}

void Autonomy::dispatchOdom(const nav_msgs::msg::Odometry::SharedPtr & msg)
{
  if (msg && odom_smoother_) {
    odom_smoother_->UpdateOdometry(conversions::fromRos(*msg));
  }
  std::lock_guard<std::mutex> lock(odom_mutex_);
  for (const auto & listener : odom_listeners_) {
    if (listener) {
      listener(msg);
    }
  }
}

bool Autonomy::hasOdom() const
{
  return odom_smoother_ && odom_smoother_->HasOdometry();
}

bool Autonomy::lastFollowPathSucceeded() const
{
  using Tick = ::autonomy::control::ControllerServer::FollowPathTickResult;
  return static_cast<Tick>(last_follow_result_.load()) == Tick::Succeeded;
}

bool Autonomy::lastFollowPathFailed() const
{
  using Tick = ::autonomy::control::ControllerServer::FollowPathTickResult;
  return static_cast<Tick>(last_follow_result_.load()) == Tick::Failed;
}

void Autonomy::feedScanToCostmap(
  const ::autonomy::commsgs::sensor_msgs::LaserScan & scan)
{
  if (auto * wrapper = plannerCostmap()) {
    wrapper->feedLaserScan(scan);
  }
}

bool Autonomy::snapshotCostmap(::autonomy::commsgs::map_msgs::OccupancyGrid & grid)
{
  if (auto * wrapper = plannerCostmap()) {
    wrapper->updateMap();
    return wrapper->snapshotOccupancyGrid(grid);
  }
  return false;
}

void Autonomy::addPlanListener(std::function<void(const nav_msgs::msg::Path &)> listener)
{
  if (!listener) {
    return;
  }
  std::lock_guard<std::mutex> lock(odom_mutex_);
  plan_listeners_.push_back(std::move(listener));
}

void Autonomy::notifyPlan(const nav_msgs::msg::Path & path)
{
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    last_path_ = path;
  }
  if (plan_pub_) {
    plan_pub_->publish(path);
  }
  std::lock_guard<std::mutex> lock(odom_mutex_);
  for (const auto & listener : plan_listeners_) {
    if (listener) {
      listener(path);
    }
  }
}

bool Autonomy::waitForDirectNavigation(
  const geometry_msgs::msg::PoseStamped & goal, double tolerance,
  std::function<bool()> cancel_checker, const double timeout_sec)
{
  if (!following_path_.load()) {
    return false;
  }
  const double limit_sec = timeout_sec > 0.0 ? timeout_sec : 300.0;
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(limit_sec);
  rclcpp::Rate rate(10);
  while (rclcpp::ok() && following_path_.load()) {
    if (cancel_checker && cancel_checker()) {
      return false;
    }
    if (cancel_navigation_.load()) {
      return false;
    }
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    if (lastFollowPathSucceeded()) {
      return true;
    }
    if (lastFollowPathFailed()) {
      return false;
    }
    if (odom_smoother_ && odom_smoother_->HasOdometry()) {
      ::autonomy::commsgs::planning_msgs::Odometry odom;
      if (odom_smoother_->GetLatestOdometry(odom)) {
        const double dx = goal.pose.position.x - odom.pose.pose.position.x;
        const double dy = goal.pose.position.y - odom.pose.pose.position.y;
        if (std::hypot(dx, dy) < tolerance) {
          return true;
        }
      }
    }
    rate.sleep();
  }
  return lastFollowPathSucceeded();
}

diagnostic_msgs::msg::DiagnosticArray Autonomy::buildDiagnostics() const
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = node_.now();

  auto make_status = [&](const std::string & name, bool ok, const std::string & msg) {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = name;
    status.level = ok ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = msg;
    return status;
  };

  array.status.push_back(make_status(
    "autonomy_ros/core", running_ && core_ != nullptr,
    running_ ? "core running" : "core not running"));
  array.status.push_back(make_status(
    "autonomy_ros/odom", hasOdom(), hasOdom() ? "odometry available" : "no odometry"));
  array.status.push_back(make_status(
    "autonomy_ros/map",
    core_ && core_->map_server() && core_->map_server()->HasStaticMap(),
    (core_ && core_->map_server() && core_->map_server()->HasStaticMap()) ?
    "static map loaded" : "static map missing"));
  array.status.push_back(make_status(
    "autonomy_ros/controller",
    controller_enabled_.load(),
    controller_enabled_.load() ? "controller enabled" : "controller idle"));

  return array;
}

void Autonomy::controlStep()
{
  if (!running_ || !core_) {
    return;
  }
  auto * controller = core_->controller_server();
  if (!controller) {
    return;
  }

  if (!controller_enabled_.load()) {
    return;
  }

  if (bt_navigation_active_.load()) {
    if (platform_bridge_) {
      platform_bridge_->publishCmdVel(controller->GetLastCmdVel());
    }
    return;
  }

  if (!following_path_.load()) {
    return;
  }

  const auto tick = controller->TickFollowPath(navigationCancelChecker());
  if (platform_bridge_) {
    platform_bridge_->publishCmdVel(controller->GetLastCmdVel());
  }
  last_follow_result_.store(static_cast<int>(tick));

  if (tick != ::autonomy::control::ControllerServer::FollowPathTickResult::Running) {
    controller->EndFollowPath();
    following_path_.store(false);
    if (tick == ::autonomy::control::ControllerServer::FollowPathTickResult::Succeeded) {
      RCLCPP_INFO(node_.get_logger(), "[autonomy] controller reached goal");
    } else if (tick ==
      ::autonomy::control::ControllerServer::FollowPathTickResult::Failed)
    {
      RCLCPP_WARN(node_.get_logger(), "[autonomy] controller follow failed");
    }
  }
}

}  // namespace autonomy_ros
