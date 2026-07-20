/*
 * Copyright 2026 autonomy_ros contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include "autonomy_controller/controller_sim_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "autonomy/common/config.hpp"
#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/control/control_options.hpp"
#include "autonomy/control/controller/graceful_controller/controller.hpp"
#include "autonomy/control/controller/mppi_controller/controller.hpp"
#include "autonomy/control/controller/pure_pursuit_controller/controller.hpp"
#include "autonomy/control/controller/pure_pursuit_controller/parameter_options.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/transform/geometry_msgs/transform_stamped.h"
#include "autonomy/transform/tf2/utils.h"
#include "autonomy_ros/conversions/geometry_msgs.hpp"
#include "autonomy_ros/conversions/map_msgs.hpp"
#include "autonomy_ros/conversions/planning_msgs.hpp"
#include "autonomy_ros/conversions/sensor_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace autonomy_controller
{
namespace
{

constexpr char kDefaultController[] = "mppi";

autonomy::control::proto::ControllerOptions LoadControllerOptionsBundle(
  const std::string & configuration_directory)
{
  const auto dirs =
    ::autonomy::common::ConfigurationSearchDirectories(configuration_directory);
  auto resolver =
    std::make_unique<::autonomy::common::ConfigurationFileResolver>(dirs);
  const std::string code = ::autonomy::common::GetLuaScriptWithCommonOrDie(
    *resolver, "control/controller.lua");
  auto file_resolver =
    std::make_unique<::autonomy::common::ConfigurationFileResolver>(dirs);
  ::autonomy::common::LuaParameterDictionary lua(code, std::move(file_resolver));
  auto root = lua.GetDictionary("AUTONOMY_CONTROLLER");
  auto options = autonomy::control::LoadOptions(root.get());

  // RPP defaults when controller.lua omits pure_pursuit_controller.
  if (!options.has_pure_pursuit_controller_options()) {
    auto empty_resolver =
      std::make_unique<::autonomy::common::ConfigurationFileResolver>(dirs);
    ::autonomy::common::LuaParameterDictionary empty(
      "return {}", std::move(empty_resolver));
    *options.mutable_pure_pursuit_controller_options() =
      autonomy::control::controller::pure_pursuit_controller::LoadOptions(&empty);
  }
  return options;
}

autonomy::map::proto::Costmap2DOptions MakeSimCostmapOptions(
  const std::string & frame_id,
  const std::string & cloud_topic,
  double resolution,
  double width,
  double height,
  double update_frequency,
  double robot_radius,
  double inflation_radius,
  double inflation_cost_scaling_factor,
  double obstacle_min_height,
  double obstacle_max_height,
  double raytrace_max_range)
{
  autonomy::map::proto::Costmap2DOptions options;
  options.set_enabled(true);
  options.set_frame_id(frame_id);
  options.set_name("controller_sim_costmap");
  options.set_resolution(resolution);
  options.set_width(width);
  options.set_height(height);
  options.set_update_frequency(update_frequency);
  options.set_rolling_window(true);
  options.set_robot_radius(robot_radius);
  options.add_plugins("obstacle_layer");
  options.add_plugins("inflation_layer");

  auto * obstacle = options.mutable_obstacle_layer();
  obstacle->set_enabled(true);
  obstacle->set_footprint_clearing_enabled(true);
  auto & sources = *obstacle->mutable_sensor_sources();
  auto & src = sources["sim_cloud"];
  src.set_topic(cloud_topic);
  src.set_data_type("PointCloud2");
  src.set_marking(true);
  src.set_clearing(false);
  src.set_min_obstacle_height(obstacle_min_height);
  src.set_max_obstacle_height(obstacle_max_height);
  src.set_raytrace_max_range(raytrace_max_range);
  src.set_raytrace_min_range(0.0);

  auto * inflation = options.mutable_inflation_layer();
  inflation->set_enabled(true);
  inflation->set_cost_scaling_factor(inflation_cost_scaling_factor);
  inflation->set_inflation_radius(inflation_radius);
  return options;
}

void PublishOdomTf(
  const nav_msgs::msg::Odometry & odom,
  const std::string & default_parent,
  const std::string & default_child)
{
  geometry_msgs::TransformStamped tf;
  const auto & stamp = odom.header.stamp;
  tf.header.stamp =
    static_cast<uint64_t>(stamp.sec) * 1'000'000'000ULL +
    static_cast<uint64_t>(stamp.nanosec);
  tf.header.frame_id =
    odom.header.frame_id.empty() ? default_parent : odom.header.frame_id;
  tf.child_frame_id =
    odom.child_frame_id.empty() ? default_child : odom.child_frame_id;
  tf.transform.translation.x = odom.pose.pose.position.x;
  tf.transform.translation.y = odom.pose.pose.position.y;
  tf.transform.translation.z = odom.pose.pose.position.z;
  tf.transform.rotation.x = odom.pose.pose.orientation.x;
  tf.transform.rotation.y = odom.pose.pose.orientation.y;
  tf.transform.rotation.z = odom.pose.pose.orientation.z;
  tf.transform.rotation.w = odom.pose.pose.orientation.w;
  autonomy::transform::Buffer::Instance()->setTransform(
    tf, "controller_sim", false);
}

}  // namespace

ControllerSimNode::ControllerSimNode()
: Node("controller_sim_node")
{
  declare_parameter<std::string>("configuration_directory", "");
  declare_parameter<std::string>("frame_id", "odom");
  declare_parameter<std::string>("base_frame", "base_footprint");
  declare_parameter<std::string>("controller_id", kDefaultController);
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
  declare_parameter<std::string>("goal_pose_topic", "goal_pose");
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

  configuration_directory_ = get_parameter("configuration_directory").as_string();
  if (configuration_directory_.empty()) {
    configuration_directory_ = ::autonomy::common::kConfigurationFilesDirectory;
  }
  frame_id_ = get_parameter("frame_id").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  controller_id_ = get_parameter("controller_id").as_string();
  repeat_path_ = get_parameter("repeat_path").as_bool();
  snap_robot_to_path_start_ = get_parameter("snap_robot_to_path_start").as_bool();
  mppi_viz_max_candidates_ = get_parameter("mppi_viz_max_candidates").as_int();
  mppi_viz_line_width_ = get_parameter("mppi_viz_line_width").as_double();
  if (mppi_viz_max_candidates_ < 1) {
    mppi_viz_max_candidates_ = 1;
  }

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

  const std::string cloud_topic =
    get_parameter("obstacle_cloud_topic").as_string();

  controller_options_ = LoadControllerOptionsBundle(configuration_directory_);
  controller_frequency_ = controller_options_.controller_frequency() > 0.0
    ? controller_options_.controller_frequency() : 20.0;

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

  SetupCostmap();
  SetupController();
  SetupPath();

  goal_checker_.Initialize("sim_goal_checker", costmap_);
  goal_checker_.SetTolerances(
    get_parameter("xy_goal_tolerance").as_double(),
    get_parameter("yaw_goal_tolerance").as_double(),
    true);

  auto qos = rclcpp::QoS(10);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    get_parameter("odom_topic").as_string(), qos,
    std::bind(&ControllerSimNode::OnOdom, this, std::placeholders::_1));
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic, qos,
    std::bind(&ControllerSimNode::OnCloud, this, std::placeholders::_1));
  goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    get_parameter("goal_pose_topic").as_string(), qos,
    std::bind(&ControllerSimNode::OnGoalPose, this, std::placeholders::_1));

  cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
    get_parameter("cmd_vel_topic").as_string(), qos);
  set_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "fake_robot/set_pose", qos);
  auto latched = rclcpp::QoS(1).transient_local().reliable();
  reference_path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "controller_sim/reference_path", latched);
  executed_path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "controller_sim/executed_path", qos);
  costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "controller_sim/local_costmap", qos);
  cloud_debug_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "controller_sim/obstacle_cloud_viz", qos);
  mppi_candidates_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "controller_sim/mppi_candidates", qos);
  mppi_optimal_path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "controller_sim/mppi_optimal_path", qos);

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
    controller_frequency_, cloud_topic.c_str(),
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

void ControllerSimNode::SetupCostmap()
{
  const auto cloud_topic = get_parameter("obstacle_cloud_topic").as_string();
  auto opts = MakeSimCostmapOptions(
    frame_id_,
    cloud_topic,
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
  // Drive updates only from OnTick / OnCloud to avoid racing the MPPI lock.
  costmap_->Pause();
}

void ControllerSimNode::SetupController()
{
  mppi_ = nullptr;
  const std::string id = controller_id_;
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
      "Unknown controller_id='" + id +
      "' (use mppi | rpp | graceful)");
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

void ControllerSimNode::ApplyReferencePlan(const bool reset_executed_path)
{
  if (!controller_ || reference_path_.poses.empty()) {
    return;
  }
  goal_checker_.Reset();
  controller_->Reset();
  controller_->SetPlan(reference_path_);
  following_ = true;
  armed_from_start_ = true;
  left_goal_region_ = false;
  PublishReferencePath();
  if (reset_executed_path) {
    executed_path_.poses.clear();
    executed_path_.header.frame_id = frame_id_;
  }
}

void ControllerSimNode::OnGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
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
  goal_initial_dist_ = std::hypot(
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
  // Costmap refresh runs on the control tick to avoid doubling work here.
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
  // Rolling window follows robot via TF published in OnOdom.
  costmap_->updateMap();

  autonomy::commsgs::map_msgs::OccupancyGrid grid;
  if (costmap_->snapshotOccupancyGrid(grid)) {
    costmap_pub_->publish(autonomy_ros::toRos(grid));
  }
}

void ControllerSimNode::PublishMppiTrajectories(const double robot_x, const double robot_y)
{
  (void)robot_x;
  (void)robot_y;
  if (!mppi_ || !mppi_candidates_pub_ || !mppi_optimal_path_pub_) {
    return;
  }
  if (mppi_candidates_pub_->get_subscription_count() == 0 &&
    mppi_optimal_path_pub_->get_subscription_count() == 0)
  {
    return;
  }

  const auto & opts = mppi_->GetMppiOptions();
  size_t time_step = 4;
  if (opts.has_trajectory_visualizer()) {
    if (opts.trajectory_visualizer().time_step() > 0) {
      time_step = static_cast<size_t>(opts.trajectory_visualizer().time_step());
    }
  }

  const auto & candidates = mppi_->GetGeneratedTrajectories();
  const auto stamp = now();

  visualization_msgs::msg::MarkerArray markers;
  const size_t n_rows = static_cast<size_t>(candidates.x.rows());
  const size_t n_cols = static_cast<size_t>(candidates.x.cols());
  if (n_rows == 0 || n_cols == 0) {
    return;
  }

  // One thin LINE_STRIP per rollout (kinematic integration of sampled controls).
  const size_t max_lines = static_cast<size_t>(mppi_viz_max_candidates_);
  const size_t row_step = std::max(size_t(1), (n_rows + max_lines - 1) / max_lines);
  markers.markers.reserve(max_lines + 2);

  int marker_id = 0;
  for (size_t i = 0; i < n_rows && marker_id < static_cast<int>(max_lines); i += row_step) {
    visualization_msgs::msg::Marker line;
    line.header.frame_id = frame_id_;
    line.header.stamp = stamp;
    line.ns = "mppi_candidates";
    line.id = marker_id++;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = mppi_viz_line_width_;
    const float t = n_rows > 1
      ? static_cast<float>(i) / static_cast<float>(n_rows - 1) : 0.0f;
    line.color.r = 0.1f;
    line.color.g = 0.55f + 0.35f * t;
    line.color.b = 0.9f - 0.4f * t;
    line.color.a = 0.55f;

    line.points.reserve((n_cols + time_step - 1) / time_step);
    for (size_t j = 0; j < n_cols; j += time_step) {
      const float x =
        candidates.x(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j));
      const float y =
        candidates.y(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j));
      if (!std::isfinite(x) || !std::isfinite(y)) {
        continue;
      }
      geometry_msgs::msg::Point p;
      p.x = static_cast<double>(x);
      p.y = static_cast<double>(y);
      p.z = 0.04;
      line.points.push_back(p);
    }
    if (line.points.size() >= 2) {
      markers.markers.push_back(std::move(line));
    } else {
      --marker_id;
    }
  }

  for (int id = marker_id; id < mppi_viz_published_candidates_; ++id) {
    visualization_msgs::msg::Marker del;
    del.header.frame_id = frame_id_;
    del.header.stamp = stamp;
    del.ns = "mppi_candidates";
    del.id = id;
    del.action = visualization_msgs::msg::Marker::DELETE;
    markers.markers.push_back(std::move(del));
  }
  mppi_viz_published_candidates_ = marker_id;

  if (mppi_optimal_path_pub_->get_subscription_count() > 0) {
    const auto optimal = mppi_->GetOptimizedTrajectory();
    nav_msgs::msg::Path optimal_path;
    optimal_path.header.frame_id = frame_id_;
    optimal_path.header.stamp = stamp;

    visualization_msgs::msg::Marker opt_line;
    opt_line.header = optimal_path.header;
    opt_line.ns = "mppi_optimal";
    opt_line.id = 0;
    opt_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    opt_line.action = visualization_msgs::msg::Marker::ADD;
    opt_line.pose.orientation.w = 1.0;
    opt_line.scale.x = std::max(0.02, mppi_viz_line_width_ * 2.5);
    opt_line.color.r = 1.0f;
    opt_line.color.g = 0.45f;
    opt_line.color.b = 0.05f;
    opt_line.color.a = 0.95f;

    const size_t opt_rows = static_cast<size_t>(optimal.rows());
    opt_line.points.reserve(opt_rows);
    optimal_path.poses.reserve(opt_rows);
    for (size_t i = 0; i < opt_rows; ++i) {
      const float x = optimal(static_cast<Eigen::Index>(i), 0);
      const float y = optimal(static_cast<Eigen::Index>(i), 1);
      if (!std::isfinite(x) || !std::isfinite(y)) {
        continue;
      }
      geometry_msgs::msg::Point p;
      p.x = static_cast<double>(x);
      p.y = static_cast<double>(y);
      p.z = 0.06;
      opt_line.points.push_back(p);

      geometry_msgs::msg::PoseStamped ps;
      ps.header = optimal_path.header;
      ps.pose.position = p;
      const double yaw = static_cast<double>(
        optimal(static_cast<Eigen::Index>(i), 2));
      ps.pose.orientation.z = std::sin(yaw * 0.5);
      ps.pose.orientation.w = std::cos(yaw * 0.5);
      optimal_path.poses.push_back(ps);
    }
    if (opt_line.points.size() >= 2) {
      markers.markers.push_back(std::move(opt_line));
    }
    mppi_optimal_path_pub_->publish(optimal_path);
  }

  if (!markers.markers.empty()) {
    mppi_candidates_pub_->publish(markers);
  }
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

  // Keep /controller_sim/local_costmap synced with cloud + robot pose each cycle.
  UpdateAndPublishCostmap();

  auto pose = autonomy_ros::fromRos(
    [&]() {
      geometry_msgs::msg::PoseStamped p;
      p.header = odom.header;
      p.pose = odom.pose.pose;
      return p;
    }());
  pose.header.frame_id =
    pose.header.frame_id.empty() ? frame_id_ : pose.header.frame_id;

  autonomy::commsgs::geometry_msgs::TwistStamped velocity;
  velocity.header.frame_id = base_frame_;
  velocity.twist = autonomy_ros::fromRos(odom.twist.twist);

  const double xy_tol = get_parameter("xy_goal_tolerance").as_double();

  if (!ClosedLoopTrackingMode()) {
  // Goal = last pose on reference path (open-loop / goal_pose navigation).
  const auto & goal_pose = reference_path_.poses.back().pose;
  const double dist_to_goal = std::hypot(
    pose.pose.position.x - goal_pose.position.x,
    pose.pose.position.y - goal_pose.position.y);
  const double arm_radius = std::max(1.0, 3.0 * xy_tol);

  bool check_goal_reached = false;
  if (goal_pose_mode_) {
    if (!left_goal_region_) {
      if (goal_initial_dist_ <= xy_tol) {
        // Already within XY tolerance when goal was issued; yaw-only finish.
        left_goal_region_ = true;
      } else {
        const auto & start_pose = reference_path_.poses.front().pose;
        const double dist_from_start = std::hypot(
          pose.pose.position.x - start_pose.position.x,
          pose.pose.position.y - start_pose.position.y);
        const double progress_toward_goal = goal_initial_dist_ - dist_to_goal;
        const double progress_required = std::clamp(
          goal_initial_dist_ * 0.25, xy_tol * 0.25, arm_radius);
        if (dist_from_start > progress_required ||
          progress_toward_goal > progress_required)
        {
          left_goal_region_ = true;
        }
      }
    }
    if (left_goal_region_) {
      check_goal_reached = true;
    }
  } else {
    // Closed loops start ≈ goal; wait until near path start, then require
    // leaving the goal neighborhood before accepting IsGoalReached.
    const auto & start_pose = reference_path_.poses.front().pose;
    const double dist_to_start = std::hypot(
      pose.pose.position.x - start_pose.position.x,
      pose.pose.position.y - start_pose.position.y);
    if (!armed_from_start_) {
      if (dist_to_start < std::max(0.5, 2.0 * xy_tol)) {
        armed_from_start_ = true;
      }
    } else if (!left_goal_region_) {
      if (dist_to_goal > arm_radius) {
        left_goal_region_ = true;
      }
    } else {
      check_goal_reached = true;
    }
  }

  if (check_goal_reached &&
    goal_checker_.IsGoalReached(pose.pose, goal_pose, velocity.twist))
  {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000, "Goal reached");
    if (goal_pose_mode_) {
      PublishZeroCmd();
      following_ = false;
      return;
    } else {
      PublishZeroCmd();
      following_ = false;
      return;
    }
  }
  }

  autonomy::commsgs::geometry_msgs::TwistStamped cmd;
  std::string message;
  try {
    decltype(controller_->ComputeVelocityCommands(
      pose, velocity, cmd, &goal_checker_, message)) code = 0;
    {
      // Keep costmap stable while MPPI scores trajectories.
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

  PublishMppiTrajectories(pose.pose.position.x, pose.pose.position.y);

  auto ros_cmd = autonomy_ros::toRos(cmd);
  ros_cmd.header.stamp = now();
  if (ros_cmd.header.frame_id.empty()) {
    ros_cmd.header.frame_id = base_frame_;
  }
  cmd_pub_->publish(ros_cmd);

  geometry_msgs::msg::PoseStamped executed_pose;
  executed_pose.header.stamp = now();
  executed_pose.header.frame_id = frame_id_;
  executed_pose.pose = odom.pose.pose;
  executed_path_.header.stamp = executed_pose.header.stamp;
  executed_path_.poses.push_back(executed_pose);
  if (executed_path_.poses.size() > 5000) {
    executed_path_.poses.erase(
      executed_path_.poses.begin(),
      executed_path_.poses.begin() + 1000);
  }
  executed_path_pub_->publish(executed_path_);
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
