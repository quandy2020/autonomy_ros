/*
 * Copyright 2026 autonomy_ros contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include "autonomy_planner/planner_sim_node.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/planning/planner/dijkstra/dijkstra_planner.hpp"
#include "autonomy/planning/planner/navfn/navfn_planner.hpp"
#include "autonomy/planning/planner/theta_star/theta_star_planner.hpp"
#include "autonomy/planning/planner_options.hpp"
#include "autonomy/transform/geometry_msgs/transform_stamped.h"
#include "autonomy_ros/conversions/geometry_msgs.hpp"
#include "autonomy_ros/conversions/map_msgs.hpp"
#include "autonomy_ros/conversions/planning_msgs.hpp"
#include "autonomy_ros/conversions/sensor_msgs.hpp"

namespace autonomy_planner
{
namespace
{

constexpr char kDefaultPlanner[] = "navfn_planner";

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
  options.set_name("planner_sim_costmap");
  options.set_resolution(resolution);
  options.set_width(width);
  options.set_height(height);
  options.set_update_frequency(update_frequency);
  options.set_rolling_window(false);
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
    tf, "planner_sim", false);
}

}  // namespace

PlannerSimNode::PlannerSimNode()
: Node("planner_sim_node")
{
  declare_parameter<std::string>("configuration_directory", "");
  declare_parameter<std::string>("frame_id", "odom");
  declare_parameter<std::string>("base_frame", "base_footprint");
  declare_parameter<std::string>("odom_topic", "odom");
  declare_parameter<std::string>("obstacle_cloud_topic", "planner_sim/obstacle_cloud");
  declare_parameter<std::string>("goal_pose_topic", "goal_pose");
  declare_parameter<std::string>("planner_id", kDefaultPlanner);

  declare_parameter<double>("costmap_resolution", 0.05);
  declare_parameter<double>("costmap_width", 20.0);
  declare_parameter<double>("costmap_height", 20.0);
  declare_parameter<double>("costmap_update_frequency", 5.0);
  declare_parameter<double>("costmap_publish_hz", 5.0);
  declare_parameter<double>("robot_radius", 0.22);
  declare_parameter<double>("inflation_radius", 0.55);
  declare_parameter<double>("inflation_cost_scaling_factor", 3.0);
  declare_parameter<double>("obstacle_min_height", -1.0);
  declare_parameter<double>("obstacle_max_height", 2.0);
  declare_parameter<double>("raytrace_max_range", 25.0);

  configuration_directory_ = get_parameter("configuration_directory").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  planner_id_ = get_parameter("planner_id").as_string();
  costmap_publish_hz_ = get_parameter("costmap_publish_hz").as_double();

  planner_options_ = autonomy::planning::CreateOptions(configuration_directory_);

  autonomy::transform::Buffer::Instance()->Init();
  tf_buffer_ = std::shared_ptr<autonomy::transform::Buffer>(
    autonomy::transform::Buffer::Instance(),
    [](autonomy::transform::Buffer *) {});

  geometry_msgs::TransformStamped identity_tf;
  identity_tf.header.frame_id = frame_id_;
  identity_tf.child_frame_id = base_frame_;
  identity_tf.transform.rotation.w = 1.0;
  autonomy::transform::Buffer::Instance()->setTransform(
    identity_tf, "planner_sim", false);

  SetupCostmap();
  SetupPlanners();

  const auto cloud_topic = get_parameter("obstacle_cloud_topic").as_string();
  auto qos = rclcpp::QoS(10);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    get_parameter("odom_topic").as_string(), qos,
    std::bind(&PlannerSimNode::OnOdom, this, std::placeholders::_1));
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic, qos,
    std::bind(&PlannerSimNode::OnCloud, this, std::placeholders::_1));
  goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    get_parameter("goal_pose_topic").as_string(), qos,
    std::bind(&PlannerSimNode::OnGoalPose, this, std::placeholders::_1));

  auto latched = rclcpp::QoS(1).transient_local().reliable();
  plan_pub_ = create_publisher<nav_msgs::msg::Path>("planner_sim/plan", latched);
  costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "planner_sim/global_costmap", qos);
  cloud_debug_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "planner_sim/obstacle_cloud_viz", qos);

  const double hz = costmap_publish_hz_ > 0.0 ? costmap_publish_hz_ : 5.0;
  const auto period = std::chrono::duration<double>(1.0 / hz);
  tick_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&PlannerSimNode::OnTick, this));

  RCLCPP_INFO(
    get_logger(),
    "planner_sim ready: planner=%s cloud=%s goal=%s",
    planner_id_.c_str(), cloud_topic.c_str(),
    get_parameter("goal_pose_topic").as_string().c_str());
}

PlannerSimNode::~PlannerSimNode()
{
  if (tick_timer_) {
    tick_timer_->cancel();
  }
  planners_.clear();
  if (costmap_) {
    costmap_->Stop();
    costmap_.reset();
  }
}

void PlannerSimNode::SetupCostmap()
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
    opts, "planner_sim_costmap");
  costmap_->setGlobalFrameID(frame_id_);
  costmap_->setRobotBaseFrameID(base_frame_);
  costmap_->Start();
  costmap_->Pause();

  // Seed free space so first plan works before obstacles arrive.
  if (auto * grid = costmap_->getCostmap()) {
    grid->resetMapToValue(
      0, 0, grid->getSizeInCellsX(), grid->getSizeInCellsY(),
      autonomy::map::costmap_2d::FREE_SPACE);
  }
}

void PlannerSimNode::SetupPlanners()
{
  planners_.clear();
  planners_["navfn_planner"] =
    std::make_shared<autonomy::planning::planner::navfn::NavfnPlanner>(
    planner_options_, "navfn_planner", costmap_);
  planners_["dijkstra_planner"] =
    std::make_shared<autonomy::planning::planner::dijkstra::DijkstraPlanner>(
    planner_options_, "dijkstra_planner", costmap_);
  planners_["theta_star_planner"] =
    std::make_shared<autonomy::planning::planner::theta_star::ThetaStarPlanner>(
    planner_options_, "theta_star_planner", costmap_);

  if (!GetActivePlanner()) {
    throw std::runtime_error(
            "Unknown planner_id='" + planner_id_ +
            "' (use navfn_planner | dijkstra_planner | theta_star_planner)");
  }
}

autonomy::planning::common::GlobalPlanner::SharedPtr
PlannerSimNode::GetActivePlanner() const
{
  const auto it = planners_.find(planner_id_);
  if (it == planners_.end()) {
    return nullptr;
  }
  return it->second;
}

void PlannerSimNode::OnOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  latest_odom_ = *msg;
  have_odom_ = true;
  PublishOdomTf(*msg, frame_id_, base_frame_);
}

void PlannerSimNode::OnCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    latest_cloud_ = autonomy_ros::fromRos(*msg);
    have_cloud_ = true;
  }
  cloud_debug_pub_->publish(*msg);
}

void PlannerSimNode::UpdateAndPublishCostmap()
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

bool PlannerSimNode::PlanToGoal(
  const autonomy::commsgs::geometry_msgs::PoseStamped & goal)
{
  auto planner = GetActivePlanner();
  if (!planner) {
    return false;
  }

  nav_msgs::msg::Odometry odom;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!have_odom_) {
      RCLCPP_WARN(get_logger(), "goal_pose ignored: no odom yet");
      return false;
    }
    odom = latest_odom_;
  }

  autonomy::commsgs::geometry_msgs::PoseStamped start;
  start.header.frame_id = frame_id_;
  start.pose = autonomy_ros::fromRos(odom.pose.pose);

  UpdateAndPublishCostmap();

  autonomy::commsgs::planning_msgs::Path path;
  uint32_t code = 0;
  {
    std::lock_guard<std::mutex> costmap_lock(costmap_mutex_);
    code = planner->CreatePlan(start, goal, path, []() {return false;});
  }

  if (code != static_cast<uint32_t>(
      autonomy::planning::proto::PlannerResultCode::PLANNER_SUCCESS) ||
    path.poses.empty())
  {
    RCLCPP_WARN(
      get_logger(), "CreatePlan failed: planner=%s code=%u",
      planner_id_.c_str(), code);
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    latest_plan_ = path;
  }

  auto msg = autonomy_ros::toRos(path);
  msg.header.stamp = now();
  msg.header.frame_id = frame_id_;
  for (auto & pose : msg.poses) {
    pose.header.stamp = msg.header.stamp;
    if (pose.header.frame_id.empty()) {
      pose.header.frame_id = frame_id_;
    }
  }
  plan_pub_->publish(msg);

  RCLCPP_INFO(
    get_logger(),
    "Plan ok: planner=%s poses=%zu goal=(%.2f, %.2f)",
    planner_id_.c_str(), path.poses.size(),
    goal.pose.position.x, goal.pose.position.y);
  return true;
}

void PlannerSimNode::OnGoalPose(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!msg) {
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
  PlanToGoal(goal);
}

void PlannerSimNode::OnTick()
{
  UpdateAndPublishCostmap();
}

}  // namespace autonomy_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<autonomy_planner::PlannerSimNode>();
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    fprintf(stderr, "planner_sim_node fatal: %s\n", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
