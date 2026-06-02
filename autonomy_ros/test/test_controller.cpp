/*
 * Copyright 2026 autonomy_ros contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include "autolink/autolink.hpp"
#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/control/control_options.hpp"
#include "autonomy/control/controller/mppi_controller/mppi_controller.hpp"
#include "autonomy/control/controller_server.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/map/proto/map_2d_option.pb.h"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/geometry_msgs/transform_stamped.h"
#include "autonomy/transform/tf2/utils.h"
#include "autonomy_ros/conversions/geometry_msgs.hpp"
#include "autonomy_ros/conversions/planning_msgs.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "autonomy/control/controller/mppi_controller/mppi_visualization.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "path_generator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace autonomy_ros
{
namespace
{

constexpr char kDefaultFrame[] = "odom";
// Match fake_robot_waffle.yaml base_frame for odom / MPPI TF.
constexpr char kRobotFrame[] = "base_footprint";
constexpr char kDefaultControllerId[] = "mppi_controller";

autonomy::control::proto::ControllerOptions LoadControllerOptions(
  const std::string & configuration_directory)
{
  const auto dirs = ::autonomy::common::ConfigurationSearchDirectories(
    configuration_directory);
  auto file_resolver =
    std::make_unique<::autonomy::common::ConfigurationFileResolver>(dirs);
  const std::string code = ::autonomy::common::GetLuaScriptWithCommonOrDie(
    *file_resolver, "control/controller.lua");
  ::autonomy::common::LuaParameterDictionary lua_dictionary(
    code, std::move(file_resolver));
  return autonomy::control::LoadOptions(
    lua_dictionary.GetDictionary("AUTONOMY_CONTROLLER").get());
}

struct CostmapBounds
{
  double origin_x{0.0};
  double origin_y{0.0};
  double width{20.0};
  double height{20.0};
};

CostmapBounds ComputeCostmapBounds(
  PathShape shape,
  const PathGeneratorParams & params,
  double margin)
{
  CostmapBounds bounds;
  switch (shape) {
    case PathShape::Circle:
      bounds.origin_x = params.center_x - params.radius - margin;
      bounds.origin_y = params.center_y - params.radius - margin;
      bounds.width = 2.0 * (params.radius + margin);
      bounds.height = bounds.width;
      break;
    case PathShape::Rectangle:
      bounds.origin_x = params.center_x - params.width * 0.5 - margin;
      bounds.origin_y = params.center_y - params.height * 0.5 - margin;
      bounds.width = params.width + 2.0 * margin;
      bounds.height = params.height + 2.0 * margin;
      break;
    case PathShape::FigureEight: {
        const double r = params.figure_eight_scale * 0.5;
        bounds.origin_x = params.center_x - r - margin;
        bounds.origin_y = params.center_y - 2.0 * r - margin;
        bounds.width = 2.0 * (r + margin);
        bounds.height = 4.0 * r + 2.0 * margin;
        break;
      }
  }
  // Extra room for MPPI rollout (~vx_max * time_steps * model_dt).
  constexpr double kMppiRolloutMargin = 2.5;
  bounds.width += 2.0 * kMppiRolloutMargin;
  bounds.height += 2.0 * kMppiRolloutMargin;
  bounds.origin_x -= kMppiRolloutMargin;
  bounds.origin_y -= kMppiRolloutMargin;
  bounds.width = std::max(bounds.width, 10.0);
  bounds.height = std::max(bounds.height, 10.0);
  return bounds;
}

autonomy::map::costmap_2d::Costmap2DWrapper::SharedPtr CreateSyntheticCostmap(
  const std::string & frame_id,
  PathShape shape,
  const PathGeneratorParams & params,
  double margin,
  const autonomy::map::proto::Costmap2DOptions & template_opts)
{
  const auto bounds = ComputeCostmapBounds(shape, params, margin);
  constexpr double kResolution = 0.05;

  autonomy::map::proto::Costmap2DOptions costmap_opts = template_opts;
  costmap_opts.set_enabled(true);
  costmap_opts.set_frame_id(frame_id);
  costmap_opts.set_name("controller_test_costmap");
  costmap_opts.set_resolution(kResolution);
  costmap_opts.set_width(static_cast<int32_t>(std::ceil(bounds.width)));
  costmap_opts.set_height(static_cast<int32_t>(std::ceil(bounds.height)));
  costmap_opts.set_rolling_window(false);
  costmap_opts.clear_plugins();
  costmap_opts.add_plugins("none");

  auto wrapper = std::make_shared<autonomy::map::costmap_2d::Costmap2DWrapper>(
    costmap_opts, "controller_test_costmap");
  if (auto * grid = wrapper->getCostmap()) {
    const unsigned int size_x = static_cast<unsigned int>(
      bounds.width / kResolution);
    const unsigned int size_y = static_cast<unsigned int>(
      bounds.height / kResolution);
    grid->resizeMap(
      std::max(1u, size_x), std::max(1u, size_y), kResolution,
      bounds.origin_x, bounds.origin_y);
    grid->resetMapToValue(
      0, 0, grid->getSizeInCellsX(), grid->getSizeInCellsY(),
      autonomy::map::costmap_2d::FREE_SPACE);
  }
  return wrapper;
}

geometry_msgs::msg::Point MakePoint(double x, double y, double z = 0.0)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

nav_msgs::msg::Path BuildOptimalPathMsg(
  const autonomy::control::controller::mppi::MppiVisualizationSnapshot & snap,
  const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path path;
  path.header.stamp = stamp;
  path.header.frame_id = snap.frame_id;
  path.poses.reserve(snap.optimal_x.size());
  for (size_t i = 0; i < snap.optimal_x.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = snap.optimal_x[i];
    pose.pose.position.y = snap.optimal_y[i];
    pose.pose.position.z = 0.0;
    const double yaw = static_cast<double>(snap.optimal_yaw[i]);
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    pose.pose.orientation.w = std::cos(yaw * 0.5);
    path.poses.push_back(pose);
  }
  return path;
}

visualization_msgs::msg::MarkerArray BuildMppiMarkerArray(
  const autonomy::control::controller::mppi::MppiVisualizationSnapshot & snap,
  const rclcpp::Time & stamp,
  int trajectory_step_override,
  int time_step_override)
{
  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker clear;
  clear.header.stamp = stamp;
  clear.header.frame_id = snap.frame_id;
  clear.ns = "mppi";
  clear.id = 0;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(clear);

  const int traj_step = std::max(
    1, trajectory_step_override > 0 ? trajectory_step_override
                                    : snap.trajectory_step);
  const int time_step = std::max(
    1, time_step_override > 0 ? time_step_override : snap.time_step);

  const auto & cand = snap.candidates;
  const int n_rows = static_cast<int>(cand.x.rows());
  const int n_cols = static_cast<int>(cand.x.cols());
  if (n_rows <= 0 || n_cols <= 0) {
    return array;
  }

  int marker_id = 1;
  const int sampled_rows = std::max(1, (n_rows + traj_step - 1) / traj_step);

  for (int i = 0; i < n_rows; i += traj_step) {
    visualization_msgs::msg::Marker line;
    line.header.stamp = stamp;
    line.header.frame_id = snap.frame_id;
    line.ns = "candidate_trajectories";
    line.id = marker_id++;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.015;
    line.color.a = 0.35f;
    line.color.r = 0.0f;
    const float row_hue =
      static_cast<float>(i / traj_step) / static_cast<float>(sampled_rows);
    line.color.g = row_hue;
    line.color.b = 1.0f - row_hue;

    for (int j = 0; j < n_cols; j += time_step) {
      line.points.push_back(
        MakePoint(cand.x(i, j), cand.y(i, j), 0.03));
    }
    if (line.points.size() >= 2) {
      array.markers.push_back(line);
    }
  }

  if (!snap.optimal_x.empty()) {
    visualization_msgs::msg::Marker optimal;
    optimal.header.stamp = stamp;
    optimal.header.frame_id = snap.frame_id;
    optimal.ns = "optimal_trajectory";
    optimal.id = marker_id++;
    optimal.type = visualization_msgs::msg::Marker::LINE_STRIP;
    optimal.action = visualization_msgs::msg::Marker::ADD;
    optimal.pose.orientation.w = 1.0;
    optimal.scale.x = 0.05;
    optimal.color.r = 1.0f;
    optimal.color.g = 0.25f;
    optimal.color.b = 0.0f;
    optimal.color.a = 1.0f;
    for (size_t k = 0; k < snap.optimal_x.size(); ++k) {
      optimal.points.push_back(
        MakePoint(snap.optimal_x[k], snap.optimal_y[k], 0.06));
    }
    array.markers.push_back(optimal);
  }

  return array;
}

void PublishOdomToTf(const nav_msgs::msg::Odometry & odom, const std::string & default_parent)
{
  const std::string parent =
    odom.header.frame_id.empty() ? default_parent : odom.header.frame_id;
  const std::string child =
    odom.child_frame_id.empty() ? kRobotFrame : odom.child_frame_id;

  geometry_msgs::TransformStamped tf;
  const auto & stamp = odom.header.stamp;
  tf.header.stamp =
    static_cast<uint64_t>(stamp.sec) * 1'000'000'000ULL +
    static_cast<uint64_t>(stamp.nanosec);
  tf.header.frame_id = parent;
  tf.child_frame_id = child;
  tf.transform.translation.x = odom.pose.pose.position.x;
  tf.transform.translation.y = odom.pose.pose.position.y;
  tf.transform.translation.z = odom.pose.pose.position.z;
  tf.transform.rotation.x = odom.pose.pose.orientation.x;
  tf.transform.rotation.y = odom.pose.pose.orientation.y;
  tf.transform.rotation.z = odom.pose.pose.orientation.z;
  tf.transform.rotation.w = odom.pose.pose.orientation.w;

  autonomy::transform::Buffer::Instance()->setTransform(
    tf, "test_controller", false);
}

}  // namespace

class ControllerRosTester : public rclcpp::Node
{
public:
  ControllerRosTester()
  : Node("controller_ros_tester")
  {
    declare_parameter<std::string>("configuration_directory", "");
    declare_parameter<std::string>("frame_id", kDefaultFrame);
    declare_parameter<std::string>("controller_id", kDefaultControllerId);
    declare_parameter<std::string>("goal_checker_id", "goal_checker");
    declare_parameter<std::string>("progress_checker_id", "progress_checker");
    declare_parameter<std::string>("path_shape", "circle");
    declare_parameter<double>("path_center_x", 0.0);
    declare_parameter<double>("path_center_y", 0.0);
    declare_parameter<double>("path_radius", 2.0);
    declare_parameter<double>("path_width", 4.0);
    declare_parameter<double>("path_height", 3.0);
    declare_parameter<double>("figure_eight_scale", 2.0);
    declare_parameter<double>("path_pose_spacing", 0.05);
    declare_parameter<double>("costmap_margin", 3.0);
    declare_parameter<bool>("auto_start", true);
    declare_parameter<bool>("snap_robot_to_path_start", true);
    declare_parameter<bool>("repeat_path", true);
    declare_parameter<bool>("publish_mppi_trajectories", true);
    declare_parameter<int>("mppi_trajectory_step", 5);
    declare_parameter<int>("mppi_time_step", 3);

    const std::string config_dir = get_parameter("configuration_directory").as_string();
    if (config_dir.empty()) {
      throw std::runtime_error("parameter 'configuration_directory' is required");
    }

    frame_id_ = get_parameter("frame_id").as_string();
    controller_id_ = get_parameter("controller_id").as_string();
    goal_checker_id_ = get_parameter("goal_checker_id").as_string();
    progress_checker_id_ = get_parameter("progress_checker_id").as_string();
    path_shape_name_ = get_parameter("path_shape").as_string();
    path_shape_ = ParsePathShape(path_shape_name_);
    auto_start_ = get_parameter("auto_start").as_bool();
    snap_robot_to_path_start_ = get_parameter("snap_robot_to_path_start").as_bool();
    repeat_path_ = get_parameter("repeat_path").as_bool();

    path_params_.frame_id = frame_id_;
    path_params_.center_x = get_parameter("path_center_x").as_double();
    path_params_.center_y = get_parameter("path_center_y").as_double();
    path_params_.radius = get_parameter("path_radius").as_double();
    path_params_.width = get_parameter("path_width").as_double();
    path_params_.height = get_parameter("path_height").as_double();
    path_params_.figure_eight_scale = get_parameter("figure_eight_scale").as_double();
    path_params_.pose_spacing = get_parameter("path_pose_spacing").as_double();
    costmap_margin_ = get_parameter("costmap_margin").as_double();
    publish_mppi_trajectories_ = get_parameter("publish_mppi_trajectories").as_bool();
    mppi_trajectory_step_ = get_parameter("mppi_trajectory_step").as_int();
    mppi_time_step_ = get_parameter("mppi_time_step").as_int();

    auto controller_options = LoadControllerOptions(config_dir);
    if (controller_id_ == "mppi_controller") {
      controller_options.set_failure_tolerance(
        std::max(controller_options.failure_tolerance(), 120.0));
      if (controller_options.has_checker_options()) {
        auto * gc =
          controller_options.mutable_checker_options()->mutable_goal_checker();
        if (gc->xy_goal_tolerance() < 0.35) {
          gc->set_xy_goal_tolerance(0.35);
        }
        if (gc->yaw_goal_tolerance() < 0.6) {
          gc->set_yaw_goal_tolerance(0.6);
        }
      }
    }
    if (controller_id_ == "mppi_controller" &&
      !controller_options.has_mppi_controller_options())
    {
      throw std::runtime_error(
        "controller_id=mppi_controller but mppi_controller block missing in "
        "control/controller.lua");
    }

    controller_frequency_ = controller_options.controller_frequency() > 0.0
      ? controller_options.controller_frequency() : 20.0;

    autonomy::transform::Buffer::Instance()->Init();
    geometry_msgs::TransformStamped identity_tf;
    identity_tf.header.frame_id = frame_id_;
    identity_tf.child_frame_id = kRobotFrame;
    identity_tf.transform.rotation.w = 1.0;
    autonomy::transform::Buffer::Instance()->setTransform(
      identity_tf, "test_controller", false);

    costmap_wrapper_ = CreateSyntheticCostmap(
      frame_id_, path_shape_, path_params_, costmap_margin_,
      controller_options.costmap_2d_options());
    costmap_wrapper_->setGlobalFrameID(frame_id_);
    costmap_wrapper_->setRobotBaseFrameID(kRobotFrame);

    controller_ = std::make_unique<autonomy::control::ControllerServer>(controller_options);

    auto tf = std::shared_ptr<autonomy::transform::Buffer>(
      autonomy::transform::Buffer::Instance(),
      [](autonomy::transform::Buffer *) {});
    controller_->SetNavigationContext(tf, frame_id_, kRobotFrame);
    controller_->SetSharedCostmap(costmap_wrapper_);
    controller_->Start();

    auto reference_qos = rclcpp::QoS(1).reliable().transient_local();
    reference_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "controller_test/reference_path", reference_qos);
    executed_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "controller_test/executed_path", 10);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", rclcpp::QoS(rclcpp::KeepLast(10)));
    center_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "controller_test/center", 10);
    if (publish_mppi_trajectories_ && controller_id_ == "mppi_controller") {
      mppi_candidate_markers_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(
        "controller_test/mppi/candidate_trajectories", 10);
      mppi_optimal_path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "controller_test/mppi/optimal_trajectory", 10);
    }

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&ControllerRosTester::OnOdom, this, std::placeholders::_1));
    fake_robot_set_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "fake_robot/set_pose", rclcpp::QoS(1).reliable());

    const auto cmd_vel_period_ms = static_cast<int64_t>(
      std::max(1.0, std::ceil(1000.0 / controller_frequency_)));
    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(cmd_vel_period_ms),
      std::bind(&ControllerRosTester::OnControlTimer, this));
    reference_path_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&ControllerRosTester::OnReferencePathTimer, this));

    if (!EnsureReferencePath()) {
      throw std::runtime_error("Failed to generate reference path at startup");
    }
    PublishReferencePath();

    const auto costmap_bounds =
      ComputeCostmapBounds(path_shape_, path_params_, costmap_margin_);
    RCLCPP_INFO(
      get_logger(),
      "Controller tester started. shape=%s controller=%s frame=%s "
      "costmap origin=(%.2f, %.2f) size=%.1fx%.1f m reference_path_topic=%s",
      path_shape_name_.c_str(), controller_id_.c_str(), frame_id_.c_str(),
      costmap_bounds.origin_x, costmap_bounds.origin_y,
      costmap_bounds.width, costmap_bounds.height,
      "controller_test/reference_path");
    if (publish_mppi_trajectories_ && controller_id_ == "mppi_controller") {
      RCLCPP_INFO(
        get_logger(),
        "MPPI viz: candidate_markers=controller_test/mppi/candidate_trajectories "
        "optimal_path=controller_test/mppi/optimal_trajectory "
        "(trajectory_step=%d time_step=%d)",
        mppi_trajectory_step_, mppi_time_step_);
    }

    if (auto_start_) {
      pending_auto_start_ = true;
    }
  }

  ~ControllerRosTester() override
  {
    follow_active_.store(false);
    if (follow_thread_.joinable()) {
      follow_thread_.join();
    }
    if (controller_) {
      controller_->Shutdown();
    }
  }

private:
  bool EnsureReferencePath()
  {
    if (reference_path_.poses.size() >= 2) {
      return true;
    }
    try {
      reference_path_ = GeneratePath(path_shape_, path_params_);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Path generation failed: %s", ex.what());
      return false;
    }
    if (reference_path_.poses.size() < 2) {
      RCLCPP_ERROR(get_logger(), "Generated path is too short");
      return false;
    }
    return true;
  }

  void SyncControllerOdometryAndTf(
    const autonomy::commsgs::geometry_msgs::PoseStamped & pose)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = kRobotFrame;
    odom.pose.pose = autonomy_ros::toRos(pose).pose;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.angular.z = 0.0;
    PublishOdomToTf(odom, frame_id_);
    controller_->UpdateOdometry(autonomy_ros::fromRos(odom));
  }

  void SnapFakeRobotToPathStart()
  {
    if (!snap_robot_to_path_start_ || reference_path_.poses.empty()) {
      return;
    }
    const auto & start = reference_path_.poses.front();
    auto pose = autonomy_ros::toRos(start);
    pose.header.stamp = now();
    if (pose.header.frame_id.empty()) {
      pose.header.frame_id = frame_id_;
    }
    fake_robot_set_pose_pub_->publish(pose);
    SyncControllerOdometryAndTf(start);
    const double yaw = autonomy::transform::tf2::getYaw(start.pose.orientation);
    RCLCPP_INFO(
      get_logger(),
      "Snapped fake_robot to path start (%.2f, %.2f, yaw=%.2f rad)",
      pose.pose.position.x, pose.pose.position.y, yaw);
  }

  void StartReferencePath()
  {
    if (follow_thread_.joinable()) {
      follow_active_.store(false);
      follow_thread_.join();
    }

    if (!EnsureReferencePath()) {
      return;
    }

    SnapFakeRobotToPathStart();
    PublishReferencePath();
    PublishCenter();

    follow_thread_ = std::thread(
      &ControllerRosTester::FollowGeneratedPath, this, reference_path_);
  }

  void OnOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!msg || !controller_) {
      return;
    }

    if (pending_auto_start_) {
      pending_auto_start_ = false;
      RCLCPP_INFO(get_logger(), "First odom received, starting path follow");
      StartReferencePath();
      return;
    }

    try {
      PublishOdomToTf(*msg, frame_id_);
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to publish odom to TF buffer: %s", ex.what());
      return;
    }

    controller_->UpdateOdometry(autonomy_ros::fromRos(*msg));

    if (!follow_active_.load()) {
      return;
    }

    autonomy::commsgs::geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id_;
    pose.pose = autonomy_ros::fromRos(*msg).pose.pose;

    std::lock_guard<std::mutex> lock(executed_mutex_);
    if (executed_path_.poses.empty() ||
      std::hypot(
        executed_path_.poses.back().pose.position.x - pose.pose.position.x,
        executed_path_.poses.back().pose.position.y - pose.pose.position.y) > 0.02)
    {
      executed_path_.poses.push_back(pose);
    }
    PublishExecutedPathLocked();
  }

  void OnControlTimer()
  {
    if (!follow_active_.load() || !controller_) {
      return;
    }
    auto cmd = autonomy_ros::toRos(last_cmd_vel_);
    cmd.header.stamp = now();
    if (cmd.header.frame_id.empty()) {
      cmd.header.frame_id = kRobotFrame;
    }
    cmd_vel_pub_->publish(cmd);
  }

  void FollowGeneratedPath(const autonomy::commsgs::planning_msgs::Path & path)
  {
    follow_active_.store(true);

    using TickResult = autonomy::control::ControllerServer::FollowPathTickResult;
    const auto tick_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / controller_frequency_));

    int lap_count = 0;
    TickResult final_result = TickResult::Running;

    while (follow_active_.load() && autolink::OK()) {
      ++lap_count;
      if (lap_count == 1) {
        RCLCPP_INFO(
          get_logger(),
          "Following path_generator path (%zu poses) controller=%s repeat=%s",
          path.poses.size(), controller_id_.c_str(),
          repeat_path_ ? "true" : "false");
      } else {
        RCLCPP_INFO(get_logger(), "Restarting path follow (lap %d)", lap_count);
        if (snap_robot_to_path_start_) {
          SnapFakeRobotToPathStart();
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
      }

      {
        std::lock_guard<std::mutex> lock(executed_mutex_);
        executed_path_.poses.clear();
        executed_path_.header = path.header;
      }

      if (!controller_->BeginFollowPath(
          path, controller_id_, goal_checker_id_, progress_checker_id_))
      {
        RCLCPP_ERROR(get_logger(), "BeginFollowPath failed for generated path");
        break;
      }

      auto next_tick = std::chrono::steady_clock::now();
      final_result = TickResult::Running;

      while (follow_active_.load() && autolink::OK()) {
        const auto now_tick = std::chrono::steady_clock::now();
        if (now_tick < next_tick) {
          std::this_thread::sleep_for(next_tick - now_tick);
        }
        next_tick += tick_period;

        final_result = controller_->TickFollowPath(
          []() { return false; }, &last_cmd_vel_);
        if (publish_mppi_trajectories_ && controller_id_ == "mppi_controller") {
          PublishMppiTrajectories();
        }
        if (final_result == TickResult::Succeeded) {
          RCLCPP_INFO(
            get_logger(), "Controller reached path goal (lap %d)", lap_count);
          break;
        }
        if (final_result == TickResult::Failed) {
          RCLCPP_ERROR(get_logger(), "Controller tick failed (lap %d)", lap_count);
          break;
        }
        if (final_result == TickResult::Cancelled) {
          RCLCPP_WARN(get_logger(), "Controller follow cancelled (lap %d)", lap_count);
          break;
        }
      }

      controller_->EndFollowPath();

      geometry_msgs::msg::TwistStamped stop;
      stop.header.stamp = now();
      cmd_vel_pub_->publish(stop);

      if (final_result == TickResult::Succeeded && repeat_path_) {
        continue;
      }
      break;
    }

    follow_active_.store(false);

    if (final_result == TickResult::Succeeded) {
      RCLCPP_INFO(
        get_logger(), "Finished path follow after %d lap(s)", lap_count);
    }
  }

  void OnReferencePathTimer()
  {
    if (reference_path_.poses.size() < 2) {
      return;
    }
    PublishReferencePath();
  }

  void PublishReferencePath()
  {
    auto ros_path = autonomy_ros::toRos(reference_path_);
    ros_path.header.stamp = now();
    if (ros_path.header.frame_id.empty()) {
      ros_path.header.frame_id = frame_id_;
    }
    reference_path_pub_->publish(ros_path);
  }

  void PublishCenter()
  {
    autonomy::commsgs::geometry_msgs::PoseStamped center;
    center.header.frame_id = frame_id_;
    center.pose.position.x = path_params_.center_x;
    center.pose.position.y = path_params_.center_y;
    center.pose.orientation.w = 1.0;
    auto ros_center = autonomy_ros::toRos(center);
    ros_center.header.stamp = now();
    center_pub_->publish(ros_center);
  }

  void PublishExecutedPathLocked()
  {
    auto ros_path = autonomy_ros::toRos(executed_path_);
    ros_path.header.stamp = now();
    if (ros_path.header.frame_id.empty()) {
      ros_path.header.frame_id = frame_id_;
    }
    executed_path_pub_->publish(ros_path);
  }

  void PublishMppiTrajectories()
  {
    if (!controller_ || !mppi_candidate_markers_pub_ || !mppi_optimal_path_pub_) {
      return;
    }

    auto * plugin = controller_->GetController(controller_id_);
    auto * mppi =
      dynamic_cast<autonomy::control::controller::MppiController *>(plugin);
    if (!mppi) {
      return;
    }
    autonomy::control::controller::mppi::MppiVisualizationSnapshot snap;
    if (!mppi->GetVisualizationSnapshot(snap) || !snap.valid) {
      return;
    }

    const rclcpp::Time stamp = now();
    mppi_candidate_markers_pub_->publish(
      BuildMppiMarkerArray(
        snap, stamp, mppi_trajectory_step_, mppi_time_step_));
    mppi_optimal_path_pub_->publish(BuildOptimalPathMsg(snap, stamp));
  }

  std::unique_ptr<autonomy::control::ControllerServer> controller_;
  double controller_frequency_{20.0};
  autonomy::map::costmap_2d::Costmap2DWrapper::SharedPtr costmap_wrapper_;

  std::string frame_id_;
  std::string controller_id_;
  std::string goal_checker_id_;
  std::string progress_checker_id_;
  std::string path_shape_name_;
  PathShape path_shape_{PathShape::Circle};
  PathGeneratorParams path_params_;
  double costmap_margin_{3.0};
  bool auto_start_{true};
  bool pending_auto_start_{false};
  bool snap_robot_to_path_start_{true};
  bool repeat_path_{true};
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    fake_robot_set_pose_pub_;
  bool publish_mppi_trajectories_{true};
  int mppi_trajectory_step_{5};
  int mppi_time_step_{3};

  autonomy::commsgs::planning_msgs::Path reference_path_;
  autonomy::commsgs::planning_msgs::Path executed_path_;
  std::mutex executed_mutex_;

  std::atomic<bool> follow_active_{false};
  std::thread follow_thread_;
  autonomy::commsgs::geometry_msgs::TwistStamped last_cmd_vel_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr executed_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr center_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
  mppi_candidate_markers_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mppi_optimal_path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr reference_path_timer_;
};

}  // namespace autonomy_ros

int main(int argc, char ** argv)
{
  if (!autolink::Init(argv[0])) {
    return 1;
  }

  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    auto node = std::make_shared<autonomy_ros::ControllerRosTester>();
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    std::cerr << "controller_ros_tester error: " << ex.what() << std::endl;
    rc = 1;
  }

  rclcpp::shutdown();
  autolink::Clear();
  return rc;
}
