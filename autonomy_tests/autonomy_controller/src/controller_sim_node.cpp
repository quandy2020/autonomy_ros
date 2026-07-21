/*
 * Copyright 2026 autonomy_ros contributors
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

/**
 * @file
 * @brief Implements ControllerSimNode.
 */

#include "autonomy_controller/controller_sim_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <utility>

#include "autonomy/common/config.hpp"
#include "autonomy/control/controller/graceful_controller/controller.hpp"
#include "autonomy/control/controller/mppi_controller/controller.hpp"
#include "autonomy/control/controller/pure_pursuit_controller/controller.hpp"
#include "autonomy_controller/controller_sim_constants.hpp"
#include "autonomy_controller/controller_sim_utils.hpp"
#include "autonomy_ros/conversions/geometry_msgs.hpp"
#include "autonomy_ros/conversions/map_msgs.hpp"
#include "autonomy_ros/conversions/planning_msgs.hpp"
#include "autonomy_ros/conversions/sensor_msgs.hpp"

namespace autonomy_controller
{

ControllerSimNode::ControllerSimNode()
: Node("controller_sim_node")
{
  DeclareParameters();
  LoadParameters();
  InitTransform();
  SetupCostmap();
  SetupController();
  SetupPath();
  SetupRosInterfaces();

  goal_checker_.Initialize("sim_goal_checker", costmap_);
  goal_checker_.SetTolerances(
    get_parameter("xy_goal_tolerance").as_double(),
    get_parameter("yaw_goal_tolerance").as_double(),
    true);

  PublishReferencePath();
  if (snap_robot_to_path_start_ && !reference_path_.poses.empty()) {
    PublishSetPose(reference_path_.poses.front());
  }

  following_ = true;
  const auto period = std::chrono::duration<double>(1.0 / controller_frequency_);
  tick_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&ControllerSimNode::OnTick, this));

  RCLCPP_INFO(
    get_logger(),
    "controller_sim ready: controller=%s path=%s freq=%.1fHz cloud=%s goal=%s",
    controller_id_.c_str(), get_parameter("path_shape").as_string().c_str(),
    controller_frequency_,
    get_parameter("obstacle_cloud_topic").as_string().c_str(),
    get_parameter("goal_pose_topic").as_string().c_str());
}

ControllerSimNode::~ControllerSimNode()
{
  following_ = false;
  PublishZeroCmd();
  if (tick_timer_) {
    tick_timer_->cancel();
  }
  if (controller_) {
    mppi_ = nullptr;
    controller_.reset();
  }
  if (costmap_) {
    costmap_->Stop();
    costmap_.reset();
  }
}

void ControllerSimNode::DeclareParameters()
{
  declare_parameter<std::string>("configuration_directory", "");
  declare_parameter<std::string>("frame_id", "odom");
  declare_parameter<std::string>("base_frame", "base_footprint");
  declare_parameter<std::string>("controller_id", kDefaultControllerId);
  declare_parameter<std::string>("odom_topic", "odom");
  declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
  declare_parameter<std::string>("obstacle_cloud_topic", "controller_sim/obstacle_cloud");
  declare_parameter<std::string>("path_shape", "circle");
  declare_parameter<double>("path_center_x", 0.0);
  declare_parameter<double>("path_center_y", 0.0);
  declare_parameter<double>("path_radius", 2.0);
  declare_parameter<double>("path_width", 4.0);
  declare_parameter<double>("path_height", 3.0);
  declare_parameter<double>("figure_eight_scale", 2.0);
  declare_parameter<double>("path_pose_spacing", 0.05);
  declare_parameter<bool>("repeat_path", true);
  declare_parameter<bool>("snap_robot_to_path_start", true);
  declare_parameter<double>("xy_goal_tolerance", 0.35);
  declare_parameter<double>("yaw_goal_tolerance", 0.6);
  declare_parameter<std::string>("goal_pose_topic", kDefaultGoalPoseTopic);
  declare_parameter<int>("mppi_viz_max_candidates", 40);
  declare_parameter<double>("mppi_viz_line_width", 0.008);

  declare_parameter<double>("costmap_resolution", 0.05);
  declare_parameter<double>("costmap_width", 12.0);
  declare_parameter<double>("costmap_height", 12.0);
  declare_parameter<double>("costmap_update_frequency", 20.0);
  declare_parameter<double>("robot_radius", 0.22);
  declare_parameter<double>("inflation_radius", 0.75);
  declare_parameter<double>("inflation_cost_scaling_factor", 3.0);
  declare_parameter<double>("obstacle_min_height", -1.0);
  declare_parameter<double>("obstacle_max_height", 2.0);
  declare_parameter<double>("raytrace_max_range", 25.0);
}

void ControllerSimNode::LoadParameters()
{
  configuration_directory_ = get_parameter("configuration_directory").as_string();
  if (configuration_directory_.empty()) {
    configuration_directory_ = ::autonomy::common::kConfigurationFilesDirectory;
  }
  frame_id_ = get_parameter("frame_id").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  controller_id_ = get_parameter("controller_id").as_string();
  repeat_path_ = get_parameter("repeat_path").as_bool();
  snap_robot_to_path_start_ = get_parameter("snap_robot_to_path_start").as_bool();

  path_params_.frame_id = frame_id_;
  path_params_.center_x = get_parameter("path_center_x").as_double();
  path_params_.center_y = get_parameter("path_center_y").as_double();
  path_params_.radius = get_parameter("path_radius").as_double();
  path_params_.width = get_parameter("path_width").as_double();
  path_params_.height = get_parameter("path_height").as_double();
  path_params_.figure_eight_scale = get_parameter("figure_eight_scale").as_double();
  path_params_.pose_spacing = get_parameter("path_pose_spacing").as_double();
  path_shape_ = autonomy_ros::ParsePathShape(
    get_parameter("path_shape").as_string());

  controller_options_ = LoadControllerOptionsBundle(configuration_directory_);
  controller_frequency_ = controller_options_.controller_frequency() > 0.0 ?
    controller_options_.controller_frequency() : 20.0;
}

void ControllerSimNode::InitTransform()
{
  autonomy::transform::Buffer::Instance()->Init();
  tf_buffer_ = std::shared_ptr<autonomy::transform::Buffer>(
    autonomy::transform::Buffer::Instance(),
    [](autonomy::transform::Buffer *) {});

  geometry_msgs::TransformStamped identity_tf;
  identity_tf.header.frame_id = frame_id_;
  identity_tf.child_frame_id = base_frame_;
  identity_tf.transform.rotation.w = 1.0;
  autonomy::transform::Buffer::Instance()->setTransform(
    identity_tf, "controller_sim", false);
}

void ControllerSimNode::SetupRosInterfaces()
{
  const auto qos = rclcpp::QoS(10);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    get_parameter("odom_topic").as_string(), qos,
    std::bind(&ControllerSimNode::OnOdom, this, std::placeholders::_1));
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    get_parameter("obstacle_cloud_topic").as_string(), qos,
    std::bind(&ControllerSimNode::OnCloud, this, std::placeholders::_1));
  goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    get_parameter("goal_pose_topic").as_string(), qos,
    std::bind(&ControllerSimNode::OnGoalPose, this, std::placeholders::_1));

  cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
    get_parameter("cmd_vel_topic").as_string(), qos);
  set_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "fake_robot/set_pose", qos);

  const auto latched = rclcpp::QoS(1).transient_local().reliable();
  reference_path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "controller_sim/reference_path", latched);
  executed_path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "controller_sim/executed_path", qos);
  costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "controller_sim/local_costmap", qos);
  cloud_debug_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "controller_sim/obstacle_cloud_viz", qos);

  mppi_viz_.Configure(
    this, frame_id_,
    get_parameter("mppi_viz_max_candidates").as_int(),
    get_parameter("mppi_viz_line_width").as_double());
}

void ControllerSimNode::SetupCostmap()
{
  auto opts = MakeSimCostmapOptions(
    frame_id_,
    get_parameter("obstacle_cloud_topic").as_string(),
    get_parameter("costmap_resolution").as_double(),
    get_parameter("costmap_width").as_double(),
    get_parameter("costmap_height").as_double(),
    get_parameter("costmap_update_frequency").as_double(),
    get_parameter("robot_radius").as_double(),
    get_parameter("inflation_radius").as_double(),
    get_parameter("inflation_cost_scaling_factor").as_double(),
    get_parameter("obstacle_min_height").as_double(),
    get_parameter("obstacle_max_height").as_double(),
    get_parameter("raytrace_max_range").as_double());

  costmap_ = std::make_shared<autonomy::map::costmap_2d::Costmap2DWrapper>(
    opts, "controller_sim_costmap");
  costmap_->setGlobalFrameID(frame_id_);
  costmap_->setRobotBaseFrameID(base_frame_);
  costmap_->Start();
  costmap_->Pause();
}

void ControllerSimNode::SetupController()
{
  mppi_ = nullptr;
  const std::string & id = controller_id_;
  if (id == "mppi" || id == "mppi_controller") {
    if (!controller_options_.has_mppi_controller_options()) {
      throw std::runtime_error("mppi_controller options missing in controller.lua");
    }
    auto ctrl = std::make_unique<
      autonomy::control::controller::mppi_controller::MPPIController>();
    ctrl->Configure(controller_options_, "mppi_controller", tf_buffer_, costmap_);
    ctrl->Activate();
    mppi_ = ctrl.get();
    controller_ = std::move(ctrl);
  } else if (id == "rpp" || id == "pure_pursuit" || id == "regulated_pure_pursuit") {
    auto ctrl = std::make_unique<
      autonomy::control::controller::pure_pursuit_controller::
      RegulatedPurePursuitController>();
    ctrl->Configure(
      controller_options_, "regulated_pure_pursuit", tf_buffer_, costmap_);
    ctrl->Activate();
    controller_ = std::move(ctrl);
  } else if (id == "graceful" || id == "graceful_controller") {
    auto ctrl = std::make_unique<
      autonomy::control::controller::GracefulController>();
    ctrl->Configure(
      controller_options_, "graceful_controller", tf_buffer_, costmap_);
    ctrl->Activate();
    controller_ = std::move(ctrl);
  } else {
    throw std::runtime_error(
            "Unknown controller_id='" + id + "' (use mppi | rpp | graceful)");
  }
}

void ControllerSimNode::SetupPath()
{
  reference_path_ = autonomy_ros::GeneratePath(path_shape_, path_params_);
  if (reference_path_.poses.empty()) {
    throw std::runtime_error("generated reference path is empty");
  }
  controller_->SetPlan(reference_path_);
  executed_path_.header.frame_id = frame_id_;
  executed_path_.poses.clear();
}

bool ControllerSimNode::ClosedLoopTrackingMode() const
{
  return repeat_path_ && !goal_pose_mode_;
}

bool ControllerSimNode::ShouldCheckGoalReached(
  const autonomy::commsgs::geometry_msgs::PoseStamped & pose,
  double xy_tolerance)
{
  if (ClosedLoopTrackingMode()) {
    return false;
  }

  const auto & goal_pose = reference_path_.poses.back().pose;
  const double dist_to_goal = std::hypot(
    pose.pose.position.x - goal_pose.position.x,
    pose.pose.position.y - goal_pose.position.y);
  const double arm_radius = std::max(1.0, 3.0 * xy_tolerance);

  if (goal_pose_mode_) {
    if (!lap_.left_goal_region) {
      if (lap_.goal_initial_dist <= xy_tolerance) {
        lap_.left_goal_region = true;
      } else {
        const auto & start_pose = reference_path_.poses.front().pose;
        const double dist_from_start = std::hypot(
          pose.pose.position.x - start_pose.position.x,
          pose.pose.position.y - start_pose.position.y);
        const double progress_toward_goal =
          lap_.goal_initial_dist - dist_to_goal;
        const double progress_required = std::clamp(
          lap_.goal_initial_dist * 0.25, xy_tolerance * 0.25, arm_radius);
        if (dist_from_start > progress_required ||
          progress_toward_goal > progress_required)
        {
          lap_.left_goal_region = true;
        }
      }
    }
    return lap_.left_goal_region;
  }

  const auto & start_pose = reference_path_.poses.front().pose;
  const double dist_to_start = std::hypot(
    pose.pose.position.x - start_pose.position.x,
    pose.pose.position.y - start_pose.position.y);
  if (!lap_.armed_from_start) {
    if (dist_to_start < std::max(0.5, 2.0 * xy_tolerance)) {
      lap_.armed_from_start = true;
    }
    return false;
  }
  if (!lap_.left_goal_region) {
    if (dist_to_goal > arm_radius) {
      lap_.left_goal_region = true;
    }
    return false;
  }
  return true;
}

void ControllerSimNode::ApplyReferencePlan(const bool reset_executed_path)
{
  if (!controller_ || reference_path_.poses.empty()) {
    return;
  }
  goal_checker_.Reset();
  controller_->Reset();
  controller_->SetPlan(reference_path_);
  following_ = true;
  lap_.armed_from_start = true;
  lap_.left_goal_region = false;
  PublishReferencePath();
  if (reset_executed_path) {
    executed_path_.poses.clear();
    executed_path_.header.frame_id = frame_id_;
  }
}

void ControllerSimNode::OnGoalPose(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!msg || !controller_) {
    return;
  }

  autonomy::commsgs::geometry_msgs::PoseStamped goal =
    autonomy_ros::fromRos(*msg);
  try {
    if (!goal.header.frame_id.empty() && goal.header.frame_id != frame_id_) {
      goal = tf_buffer_->transform(goal, frame_id_, 0.2f);
    }
  } catch (const std::exception & ex) {
    RCLCPP_WARN(
      get_logger(), "goal_pose TF %s -> %s failed: %s",
      msg->header.frame_id.c_str(), frame_id_.c_str(), ex.what());
    return;
  }
  goal.header.frame_id = frame_id_;

  nav_msgs::msg::Odometry odom;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!have_odom_) {
      RCLCPP_WARN(get_logger(), "goal_pose ignored: no odom yet");
      return;
    }
    odom = latest_odom_;
  }

  autonomy::commsgs::geometry_msgs::PoseStamped start;
  start.header.frame_id = frame_id_;
  start.pose = autonomy_ros::fromRos(odom.pose.pose);

  try {
    reference_path_ = autonomy_ros::GenerateLinePath(
      start, goal, path_params_.pose_spacing);
  } catch (const std::exception & ex) {
    RCLCPP_WARN(get_logger(), "goal_pose path generation failed: %s", ex.what());
    return;
  }

  goal_pose_mode_ = true;
  repeat_path_ = false;

  const auto & goal_pose = reference_path_.poses.back().pose;
  lap_.goal_initial_dist = std::hypot(
    start.pose.position.x - goal_pose.position.x,
    start.pose.position.y - goal_pose.position.y);

  ApplyReferencePlan(true);

  RCLCPP_INFO(
    get_logger(),
    "New goal (%.2f, %.2f) frame=%s path_poses=%zu",
    goal_pose.position.x, goal_pose.position.y, frame_id_.c_str(),
    reference_path_.poses.size());
}

void ControllerSimNode::PublishReferencePath()
{
  if (!reference_path_pub_ || reference_path_.poses.empty()) {
    return;
  }
  auto msg = autonomy_ros::toRos(reference_path_);
  msg.header.stamp = now();
  msg.header.frame_id = frame_id_;
  for (auto & pose : msg.poses) {
    pose.header.stamp = msg.header.stamp;
    if (pose.header.frame_id.empty()) {
      pose.header.frame_id = frame_id_;
    }
  }
  reference_path_pub_->publish(msg);
}

void ControllerSimNode::PublishSetPose(
  const autonomy::commsgs::geometry_msgs::PoseStamped & pose)
{
  auto msg = autonomy_ros::toRos(pose);
  msg.header.stamp = now();
  msg.header.frame_id = frame_id_;
  set_pose_pub_->publish(msg);
}

void ControllerSimNode::PublishZeroCmd()
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = now();
  cmd.header.frame_id = base_frame_;
  cmd_pub_->publish(cmd);
}

void ControllerSimNode::OnOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  latest_odom_ = *msg;
  have_odom_ = true;
  PublishOdomTf(*msg, frame_id_, base_frame_);
}

void ControllerSimNode::OnCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    latest_cloud_ = autonomy_ros::fromRos(*msg);
    have_cloud_ = true;
  }
  cloud_debug_pub_->publish(*msg);
}

void ControllerSimNode::UpdateAndPublishCostmap()
{
  if (!costmap_) {
    return;
  }

  autonomy::commsgs::sensor_msgs::PointCloud2 cloud;
  bool have_cloud = false;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (have_cloud_) {
      cloud = latest_cloud_;
      have_cloud = true;
    }
  }

  std::lock_guard<std::mutex> costmap_lock(costmap_mutex_);
  if (have_cloud) {
    costmap_->feedPointCloud2(cloud);
  }
  costmap_->updateMap();

  autonomy::commsgs::map_msgs::OccupancyGrid grid;
  if (costmap_->snapshotOccupancyGrid(grid)) {
    costmap_pub_->publish(autonomy_ros::toRos(grid));
  }
}

void ControllerSimNode::AppendExecutedPose(const nav_msgs::msg::Odometry & odom)
{
  geometry_msgs::msg::PoseStamped executed_pose;
  executed_pose.header.stamp = now();
  executed_pose.header.frame_id = frame_id_;
  executed_pose.pose = odom.pose.pose;
  executed_path_.header.stamp = executed_pose.header.stamp;
  executed_path_.poses.push_back(executed_pose);

  if (executed_path_.poses.size() > kExecutedPathMaxPoses) {
    executed_path_.poses.erase(
      executed_path_.poses.begin(),
      executed_path_.poses.begin() +
      static_cast<std::ptrdiff_t>(kExecutedPathTrimBatch));
  }
  executed_path_pub_->publish(executed_path_);
}

void ControllerSimNode::OnTick()
{
  if (!following_ || !controller_) {
    return;
  }

  nav_msgs::msg::Odometry odom;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!have_odom_) {
      return;
    }
    odom = latest_odom_;
  }

  UpdateAndPublishCostmap();

  geometry_msgs::msg::PoseStamped ros_pose;
  ros_pose.header = odom.header;
  ros_pose.pose = odom.pose.pose;
  auto pose = autonomy_ros::fromRos(ros_pose);
  pose.header.frame_id =
    pose.header.frame_id.empty() ? frame_id_ : pose.header.frame_id;

  autonomy::commsgs::geometry_msgs::TwistStamped velocity;
  velocity.header.frame_id = base_frame_;
  velocity.twist = autonomy_ros::fromRos(odom.twist.twist);

  const double xy_tol = get_parameter("xy_goal_tolerance").as_double();
  if (ShouldCheckGoalReached(pose, xy_tol)) {
    const auto & goal_pose = reference_path_.poses.back().pose;
    if (goal_checker_.IsGoalReached(pose.pose, goal_pose, velocity.twist)) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000, "Goal reached");
      PublishZeroCmd();
      following_ = false;
      return;
    }
  }

  autonomy::commsgs::geometry_msgs::TwistStamped cmd;
  std::string message;
  uint32_t code = 0;
  try {
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      code = controller_->ComputeVelocityCommands(
        pose, velocity, cmd, &goal_checker_, message);
    }
    if (code != 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ComputeVelocityCommands code=%u msg=%s", code, message.c_str());
      PublishZeroCmd();
      UpdateAndPublishCostmap();
      return;
    }
  } catch (const std::exception & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "ComputeVelocityCommands exception: %s", ex.what());
    PublishZeroCmd();
    UpdateAndPublishCostmap();
    return;
  }

  mppi_viz_.Publish(mppi_, now());

  auto ros_cmd = autonomy_ros::toRos(cmd);
  ros_cmd.header.stamp = now();
  if (ros_cmd.header.frame_id.empty()) {
    ros_cmd.header.frame_id = base_frame_;
  }
  cmd_pub_->publish(ros_cmd);
  AppendExecutedPose(odom);
}

}  // namespace autonomy_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<autonomy_controller::ControllerSimNode>();
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    fprintf(stderr, "controller_sim_node failed: %s\n", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
