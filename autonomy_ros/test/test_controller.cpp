/*
 * Copyright 2026 autonomy_ros contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

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
#include "autonomy/control/controller_server.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/planning/planner_options.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy_ros/conversions/geometry_msgs.hpp"
#include "autonomy_ros/conversions/planning_msgs.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "path_generator.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy_ros
{
namespace
{

constexpr char kDefaultFrame[] = "odom";
constexpr char kRobotFrame[] = "base_link";

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

autonomy::map::costmap_2d::Costmap2DWrapper::SharedPtr CreateSyntheticCostmap(
  const std::string & frame_id)
{
  auto planner_options = autonomy::planning::proto::PlannerOptions{};
  auto * costmap = planner_options.mutable_costmap();
  costmap->set_enabled(true);
  costmap->set_frame_id(frame_id);
  costmap->set_resolution(0.05);
  costmap->set_width(30.0);
  costmap->set_height(30.0);
  costmap->clear_plugins();
  costmap->add_plugins("none");

  auto wrapper = std::make_shared<autonomy::map::costmap_2d::Costmap2DWrapper>(
    planner_options.costmap(), "controller_test_costmap");
  if (auto * grid = wrapper->getCostmap()) {
    grid->resetMapToValue(
      0, 0, grid->getSizeInCellsX(), grid->getSizeInCellsY(),
      autonomy::map::costmap_2d::FREE_SPACE);
  }
  wrapper->updateMap();
  wrapper->Stop();
  return wrapper;
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
    declare_parameter<std::string>("controller_id", "graceful_controller");
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
    declare_parameter<bool>("auto_start", true);

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

    path_params_.frame_id = frame_id_;
    path_params_.center_x = get_parameter("path_center_x").as_double();
    path_params_.center_y = get_parameter("path_center_y").as_double();
    path_params_.radius = get_parameter("path_radius").as_double();
    path_params_.width = get_parameter("path_width").as_double();
    path_params_.height = get_parameter("path_height").as_double();
    path_params_.figure_eight_scale = get_parameter("figure_eight_scale").as_double();
    path_params_.pose_spacing = get_parameter("path_pose_spacing").as_double();

    auto controller_options = LoadControllerOptions(config_dir);
    controller_frequency_ = controller_options.controller_frequency() > 0.0
      ? controller_options.controller_frequency() : 20.0;
    controller_ = std::make_unique<autonomy::control::ControllerServer>(controller_options);

    costmap_wrapper_ = CreateSyntheticCostmap(frame_id_);
    auto tf = std::shared_ptr<autonomy::transform::Buffer>(
      autonomy::transform::Buffer::Instance(),
      [](autonomy::transform::Buffer *) {});
    controller_->SetNavigationContext(tf, frame_id_, kRobotFrame);
    controller_->SetSharedCostmap(costmap_wrapper_);
    controller_->Start();

    plan_path_pub_ = create_publisher<nav_msgs::msg::Path>("controller_test/plan", 10);
    executed_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "controller_test/executed_path", 10);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
    center_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "controller_test/center", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&ControllerRosTester::OnOdom, this, std::placeholders::_1));

    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&ControllerRosTester::OnControlTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "Controller tester started. shape=%s controller=%s frame=%s "
      "(path_generator only, no map)",
      path_shape_name_.c_str(), controller_id_.c_str(), frame_id_.c_str());

    if (auto_start_) {
      StartReferencePath();
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
  void StartReferencePath()
  {
    if (follow_thread_.joinable()) {
      follow_active_.store(false);
      follow_thread_.join();
    }

    try {
      reference_path_ = GeneratePath(path_shape_, path_params_);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Path generation failed: %s", ex.what());
      return;
    }

    if (reference_path_.poses.size() < 2) {
      RCLCPP_ERROR(get_logger(), "Generated path is too short");
      return;
    }

    PublishPlanPath(reference_path_);
    PublishCenter();

    follow_thread_ = std::thread(
      &ControllerRosTester::FollowGeneratedPath, this, reference_path_);
  }

  void OnOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!msg || !controller_) {
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
    auto cmd = autonomy_ros::toRos(controller_->GetLastCmdVel());
    cmd.header.stamp = now();
    if (cmd.header.frame_id.empty()) {
      cmd.header.frame_id = kRobotFrame;
    }
    cmd_vel_pub_->publish(cmd);
  }

  void FollowGeneratedPath(const autonomy::commsgs::planning_msgs::Path & path)
  {
    follow_active_.store(true);
    {
      std::lock_guard<std::mutex> lock(executed_mutex_);
      executed_path_.poses.clear();
      executed_path_.header = path.header;
    }

    if (!controller_->BeginFollowPath(
        path, controller_id_, goal_checker_id_, progress_checker_id_))
    {
      RCLCPP_ERROR(get_logger(), "BeginFollowPath failed for generated path");
      follow_active_.store(false);
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Following path_generator path (%zu poses) controller=%s",
      path.poses.size(), controller_id_.c_str());

    const auto tick_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / controller_frequency_));
    auto next_tick = std::chrono::steady_clock::now();

    using TickResult = autonomy::control::ControllerServer::FollowPathTickResult;
    TickResult final_result = TickResult::Running;

    while (follow_active_.load() && autolink::OK()) {
      const auto now_tick = std::chrono::steady_clock::now();
      if (now_tick < next_tick) {
        std::this_thread::sleep_for(next_tick - now_tick);
      }
      next_tick += tick_period;

      final_result = controller_->TickFollowPath([]() { return false; });
      if (final_result == TickResult::Succeeded) {
        RCLCPP_INFO(get_logger(), "Controller reached path goal");
        break;
      }
      if (final_result == TickResult::Failed) {
        RCLCPP_ERROR(get_logger(), "Controller tick failed");
        break;
      }
      if (final_result == TickResult::Cancelled) {
        RCLCPP_WARN(get_logger(), "Controller follow cancelled");
        break;
      }
    }

    controller_->EndFollowPath();
    follow_active_.store(false);

    geometry_msgs::msg::TwistStamped stop;
    stop.header.stamp = now();
    cmd_vel_pub_->publish(stop);

    if (final_result == TickResult::Succeeded) {
      RCLCPP_INFO(get_logger(), "Finished following generated path");
    }
  }

  void PublishPlanPath(const autonomy::commsgs::planning_msgs::Path & path)
  {
    auto ros_path = autonomy_ros::toRos(path);
    ros_path.header.stamp = now();
    if (ros_path.header.frame_id.empty()) {
      ros_path.header.frame_id = frame_id_;
    }
    plan_path_pub_->publish(ros_path);
    RCLCPP_INFO(get_logger(), "Published reference path (%zu poses)", path.poses.size());
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
  bool auto_start_{true};

  autonomy::commsgs::planning_msgs::Path reference_path_;
  autonomy::commsgs::planning_msgs::Path executed_path_;
  std::mutex executed_mutex_;

  std::atomic<bool> follow_active_{false};
  std::thread follow_thread_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr executed_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr center_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
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
