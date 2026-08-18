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
 * @brief Implements PlannerSimNode.
 */

#include "autonomy_planner/planner_sim_node.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "autonomy/common/config.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/layered_costmap.hpp"
#include "autonomy/map/costmap_2d/map_io.hpp"
#include "autonomy/planning/planner/dijkstra/dijkstra_planner.hpp"
#include "autonomy/planning/planner/navfn/navfn_planner.hpp"
#include "autonomy/planning/planner/theta_star/theta_star_planner.hpp"
#include "autonomy/planning/planner_options.hpp"
#include "autonomy/planning/proto/planning_options.pb.h"
#include "autonomy_planner/planner_sim_constants.hpp"
#include "autonomy_planner/planner_sim_utils.hpp"
#include "autonomy_ros/conversions/geometry_msgs.hpp"
#include "autonomy_ros/conversions/map_msgs.hpp"
#include "autonomy_ros/conversions/planning_msgs.hpp"
#include "autonomy_ros/conversions/sensor_msgs.hpp"

namespace autonomy_planner {

PlannerSimNode::PlannerSimNode()
: Node("planner_sim_node")
{
  DeclareParameters();
  LoadParameters();
  InitTransform();
  SetupCostmap();
  SetupPlanners();
  SetupRosInterfaces();

  PublishStaticMap();

  const double hz = costmap_publish_hz_ > 0.0 ? costmap_publish_hz_ : 5.0;
  const auto period = std::chrono::duration<double>(1.0 / hz);
  tick_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&PlannerSimNode::OnTick, this));

  RCLCPP_INFO(
    get_logger(),
    "planner_sim ready: planner=%s map=/%s initial=/%s goal=/%s",
    planner_id_.c_str(), map_topic_.c_str(),
    get_parameter("initial_pose_topic").as_string().c_str(),
    get_parameter("goal_pose_topic").as_string().c_str());
  RCLCPP_INFO(
    get_logger(),
    "Set RViz 2D Pose Estimate then 2D Nav Goal to plan (either order)");
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

void PlannerSimNode::DeclareParameters()
{
  declare_parameter<std::string>("configuration_directory", "");
  declare_parameter<std::string>("frame_id", "odom");
  declare_parameter<std::string>("base_frame", "base_footprint");
  declare_parameter<std::string>("map_frame", "map");
  declare_parameter<std::string>("odom_topic", "odom");
  declare_parameter<std::string>("obstacle_cloud_topic", "planner_sim/obstacle_cloud");
  declare_parameter<std::string>("initial_pose_topic", kDefaultInitialPoseTopic);
  declare_parameter<std::string>("goal_pose_topic", kDefaultGoalPoseTopic);
  declare_parameter<std::string>("planner_id", kDefaultPlannerId);
  declare_parameter<std::string>("map_file", kDefaultMapFile);
  declare_parameter<std::string>("map_topic", kDefaultMapTopic);

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
}

void PlannerSimNode::LoadParameters()
{
  configuration_directory_ = get_parameter("configuration_directory").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  map_frame_ = get_parameter("map_frame").as_string();
  planner_id_ = get_parameter("planner_id").as_string();
  map_topic_ = get_parameter("map_topic").as_string();
  costmap_publish_hz_ = get_parameter("costmap_publish_hz").as_double();
  planner_options_ = autonomy::planning::CreateOptions(configuration_directory_);
}

void PlannerSimNode::InitTransform()
{
  autonomy::transform::Buffer::Instance()->Init();
  tf_buffer_ = std::shared_ptr<autonomy::transform::Buffer>(
    autonomy::transform::Buffer::Instance(),
    [](autonomy::transform::Buffer *) {});
  static_tf_broadcaster_ =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  geometry_msgs::TransformStamped identity_tf;
  identity_tf.header.frame_id = frame_id_;
  identity_tf.child_frame_id = base_frame_;
  identity_tf.transform.rotation.w = 1.0;
  autonomy::transform::Buffer::Instance()->setTransform(
    identity_tf, "planner_sim", false);

  PublishMapToOdomTf();
}

void PlannerSimNode::SetupRosInterfaces()
{
  const auto qos = rclcpp::QoS(10);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    get_parameter("odom_topic").as_string(), qos,
    std::bind(&PlannerSimNode::OnOdom, this, std::placeholders::_1));
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    get_parameter("obstacle_cloud_topic").as_string(), qos,
    std::bind(&PlannerSimNode::OnCloud, this, std::placeholders::_1));
  initial_pose_sub_ =
    create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    get_parameter("initial_pose_topic").as_string(), qos,
    std::bind(&PlannerSimNode::OnInitialPose, this, std::placeholders::_1));
  goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    get_parameter("goal_pose_topic").as_string(), qos,
    std::bind(&PlannerSimNode::OnGoalPose, this, std::placeholders::_1));

  const auto latched = rclcpp::QoS(1).transient_local().reliable();
  plan_pub_ = create_publisher<nav_msgs::msg::Path>("planner_sim/plan", latched);
  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, latched);
  costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "planner_sim/global_costmap", qos);
  cloud_debug_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "planner_sim/obstacle_cloud_viz", qos);
  set_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "fake_robot/set_pose", rclcpp::QoS(1).reliable());
}

bool PlannerSimNode::TransformPoseToFrame(
  automsgs::msgs::geometry_msgs::PoseStamped * pose,
  const std::string & target_frame) const
{
  if (!pose || target_frame.empty()) {
    return false;
  }
  if (pose->header().frame_id().empty() ||
    pose->header().frame_id() == target_frame)
  {
    pose->mutable_header()->set_frame_id(target_frame);
    return true;
  }

  try {
    *pose = tf_buffer_->transform(*pose, target_frame, 0.2f);
    pose->mutable_header()->set_frame_id(target_frame);
    return true;
  } catch (const std::exception & ex) {
    RCLCPP_WARN(
      get_logger(), "TF %s -> %s failed: %s",
      pose->header().frame_id().c_str(), target_frame.c_str(), ex.what());
    return false;
  }
}

void PlannerSimNode::ApplyMapGridMetadata(
  nav_msgs::msg::OccupancyGrid * msg) const
{
  if (!msg) {
    return;
  }
  msg->header.frame_id = map_frame_;
  if (!have_static_map_) {
    return;
  }
  const auto ref_info = autonomy_ros::toRos(static_map_.info());
  msg->info.origin = ref_info.origin;
}

bool PlannerSimNode::LoadStaticMap(const std::string & map_file)
{
  const std::string resolved = ResolveMapYamlPath(map_file);
  if (resolved.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "map_file='%s' not found under %s/data or %s/map",
      map_file.c_str(),
      autonomy::common::kConfigurationFilesDirectory,
      autonomy::common::kConfigurationFilesDirectory);
    return false;
  }

  automsgs::msgs::map_msgs::OccupancyGrid grid;
  const auto status = autonomy::map::costmap_2d::loadMapFromYaml(resolved, grid);
  if (status != autonomy::map::costmap_2d::LOAD_MAP_STATUS::LOAD_MAP_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(), "Failed to load map yaml: %s (status=%d)",
      resolved.c_str(), static_cast<int>(status));
    return false;
  }

  grid.mutable_header()->set_frame_id(map_frame_);

  if (!costmap_->applyOccupancyGrid(grid)) {
    RCLCPP_ERROR(
      get_logger(), "applyOccupancyGrid failed for map: %s", resolved.c_str());
    return false;
  }

  static_map_ = grid;
  if (auto * costmap_grid = costmap_->getCostmap()) {
    static_map_.mutable_info()->mutable_origin()->mutable_position()->set_x(
      costmap_grid->getOriginX());
    static_map_.mutable_info()->mutable_origin()->mutable_position()->set_y(
      costmap_grid->getOriginY());
  }
  have_static_map_ = true;
  RCLCPP_INFO(
    get_logger(),
    "Loaded static map: %s (%ux%u @ %.3fm, origin=(%.2f, %.2f), frame=%s)",
    resolved.c_str(),
    static_map_.info().width(), static_map_.info().height(),
    static_map_.info().resolution(),
    static_map_.info().origin().position().x(),
    static_map_.info().origin().position().y(),
    static_map_.header().frame_id().c_str());
  return true;
}

void PlannerSimNode::PublishMapToOdomTf()
{
  if (map_frame_.empty() || map_frame_ == frame_id_) {
    return;
  }

  geometry_msgs::msg::TransformStamped ros_tf;
  ros_tf.header.stamp = now();
  ros_tf.header.frame_id = map_frame_;
  ros_tf.child_frame_id = frame_id_;
  ros_tf.transform.rotation.w = 1.0;
  static_tf_broadcaster_->sendTransform(ros_tf);

  geometry_msgs::TransformStamped autonomy_tf;
  autonomy_tf.header.frame_id = map_frame_;
  autonomy_tf.child_frame_id = frame_id_;
  autonomy_tf.transform.rotation.w = 1.0;
  autonomy::transform::Buffer::Instance()->setTransform(
    autonomy_tf, "planner_sim", true);

  RCLCPP_INFO(
    get_logger(), "Published static TF %s -> %s (identity)",
    map_frame_.c_str(), frame_id_.c_str());
}

void PlannerSimNode::PublishStaticMap()
{
  if (!have_static_map_ || !map_pub_) {
    return;
  }
  auto msg = autonomy_ros::toRos(static_map_);
  msg.header.stamp = now();
  ApplyMapGridMetadata(&msg);
  map_pub_->publish(msg);
}

void PlannerSimNode::SyncRobotState(double x, double y, double yaw)
{
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = frame_id_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.orientation.z = std::sin(yaw * 0.5);
  odom.pose.pose.orientation.w = std::cos(yaw * 0.5);

  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_odom_ = odom;
    have_odom_ = true;
  }
  PublishOdomTf(odom, frame_id_, base_frame_);
}

void PlannerSimNode::PublishRobotPose(double x, double y, double yaw)
{
  if (!set_pose_pub_) {
    return;
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = now();
  pose.header.frame_id = frame_id_;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.z = std::sin(yaw * 0.5);
  pose.pose.orientation.w = std::cos(yaw * 0.5);
  set_pose_pub_->publish(pose);
  SyncRobotState(x, y, yaw);

  robot_pose_.x = x;
  robot_pose_.y = y;
  robot_pose_.yaw = yaw;
  robot_pose_.pending = true;
}

void PlannerSimNode::SetupCostmap()
{
  auto opts = MakeSimCostmapOptions(
    map_frame_,
    get_parameter("obstacle_cloud_topic").as_string(),
    map_topic_,
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
  costmap_->setGlobalFrameID(map_frame_);
  costmap_->setRobotBaseFrameID(base_frame_);
  costmap_->Start();
  costmap_->Pause();

  const auto map_file = get_parameter("map_file").as_string();
  if (!LoadStaticMap(map_file)) {
    const double resolution = get_parameter("costmap_resolution").as_double();
    const double width = get_parameter("costmap_width").as_double();
    const double height = get_parameter("costmap_height").as_double();
    if (auto * layered = costmap_->getLayeredCostmap()) {
      const auto size_x =
        static_cast<unsigned int>(std::lround(width / resolution));
      const auto size_y =
        static_cast<unsigned int>(std::lround(height / resolution));
      layered->resizeMap(
        size_x, size_y, resolution, -0.5 * width, -0.5 * height);
    }
    if (auto * grid = costmap_->getCostmap()) {
      grid->resetMapToValue(
        0, 0, grid->getSizeInCellsX(), grid->getSizeInCellsY(),
        autonomy::map::costmap_2d::FREE_SPACE);
    }
  }

  if (auto * grid = costmap_->getCostmap()) {
    RCLCPP_INFO(
      get_logger(),
      "costmap ready: origin=(%.2f, %.2f) size=%.1fx%.1f m cells=%ux%u",
      grid->getOriginX(), grid->getOriginY(),
      grid->getSizeInMetersX(), grid->getSizeInMetersY(),
      grid->getSizeInCellsX(), grid->getSizeInCellsY());
  }
}

void PlannerSimNode::SetupPlanners()
{
  planner_options_.mutable_navfn()->set_allow_unknown(true);
  planner_options_.mutable_navfn()->set_use_astar(true);
  planner_options_.mutable_dijkstra()->set_allow_unknown(true);
  planner_options_.mutable_theta_star()->set_allow_unknown(true);

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
  return it == planners_.end() ? nullptr : it->second;
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

void PlannerSimNode::OnInitialPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  const auto stamped = autonomy_ros::fromRos(*msg);
  automsgs::msgs::geometry_msgs::PoseStamped initial;
  *initial.mutable_header() = stamped.header();
  *initial.mutable_pose() = stamped.pose().pose().pose();

  automsgs::msgs::geometry_msgs::PoseStamped initial_odom = initial;
  if (!TransformPoseToFrame(&initial_odom, frame_id_)) {
    return;
  }

  const auto ros_pose = autonomy_ros::toRos(initial_odom.pose());
  const double yaw = YawFromQuaternion(ros_pose.orientation);
  PublishRobotPose(
    initial_odom.pose().position().x(),
    initial_odom.pose().position().y(), yaw);

  if (!TransformPoseToFrame(&initial, map_frame_)) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(plan_request_mutex_);
    plan_request_.initial = initial;
    plan_request_.have_initial = true;
  }

  RCLCPP_INFO(
    get_logger(),
    "initialpose map=(%.2f, %.2f, yaw=%.2f) — waiting for goal_pose",
    initial.pose().position().x(), initial.pose().position().y(), yaw);

  TryPlanIfReady();
}

void PlannerSimNode::OnGoalPose(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  automsgs::msgs::geometry_msgs::PoseStamped goal =
    autonomy_ros::fromRos(*msg);
  if (!TransformPoseToFrame(&goal, map_frame_)) {
    return;
  }

  bool have_initial = false;
  {
    std::lock_guard<std::mutex> lock(plan_request_mutex_);
    plan_request_.goal = goal;
    plan_request_.have_goal = true;
    have_initial = plan_request_.have_initial;
  }

  RCLCPP_INFO(
    get_logger(),
    "goal_pose (%.2f, %.2f) — %s",
    goal.pose().position().x(), goal.pose().position().y(),
    have_initial ? "planning..." : "waiting for initialpose");

  TryPlanIfReady();
}

void PlannerSimNode::TryPlanIfReady()
{
  automsgs::msgs::geometry_msgs::PoseStamped start;
  automsgs::msgs::geometry_msgs::PoseStamped goal;
  bool ready = false;

  {
    std::lock_guard<std::mutex> lock(plan_request_mutex_);
    if (plan_request_.have_initial && plan_request_.have_goal) {
      start = plan_request_.initial;
      goal = plan_request_.goal;
      ready = true;
    }
  }

  if (!ready) {
    return;
  }

  const bool ok = PlanToGoal(start, goal);
  ResetPlanRequest();

  RCLCPP_INFO(
    get_logger(),
    ok ? "Plan cycle done — set initialpose + goal_pose for next plan"
    : "Plan failed — set initialpose + goal_pose to retry");
}

void PlannerSimNode::ResetPlanRequest()
{
  std::lock_guard<std::mutex> lock(plan_request_mutex_);
  plan_request_.have_initial = false;
  plan_request_.have_goal = false;
}

void PlannerSimNode::UpdateAndPublishCostmap()
{
  if (!costmap_) {
    return;
  }

  automsgs::msgs::sensor_msgs::PointCloud2 cloud;
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

  automsgs::msgs::map_msgs::OccupancyGrid grid;
  if (costmap_->snapshotOccupancyGrid(grid)) {
    auto msg = autonomy_ros::toRos(grid);
    msg.header.stamp = now();
    ApplyMapGridMetadata(&msg);
    costmap_pub_->publish(msg);
  }
}

bool PlannerSimNode::PlanToGoal(
  const automsgs::msgs::geometry_msgs::PoseStamped & start,
  const automsgs::msgs::geometry_msgs::PoseStamped & goal)
{
  auto planner = GetActivePlanner();
  if (!planner) {
    return false;
  }

  UpdateAndPublishCostmap();

  double origin_x = 0.0;
  double origin_y = 0.0;
  double size_x_m = 0.0;
  double size_y_m = 0.0;
  if (auto * grid = costmap_->getCostmap()) {
    origin_x = grid->getOriginX();
    origin_y = grid->getOriginY();
    size_x_m = grid->getSizeInMetersX();
    size_y_m = grid->getSizeInMetersY();
  }

  automsgs::msgs::nav_msgs::Path path;
  uint32_t code = 0;
  {
    std::lock_guard<std::mutex> costmap_lock(costmap_mutex_);
    code = planner->CreatePlan(start, goal, path, []() {return false;});
  }

  if (code != static_cast<uint32_t>(
      autonomy::planning::proto::PlannerResultCode::PLANNER_SUCCESS) ||
    path.poses_size() == 0)
  {
    RCLCPP_WARN(
      get_logger(),
      "CreatePlan failed: planner=%s code=%u (%s) "
      "start=(%.2f, %.2f) goal=(%.2f, %.2f) "
      "map origin=(%.2f, %.2f) size=%.1fx%.1f m",
      planner_id_.c_str(), code, PlannerResultName(code),
      start.pose().position().x(), start.pose().position().y(),
      goal.pose().position().x(), goal.pose().position().y(),
      origin_x, origin_y, size_x_m, size_y_m);
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    latest_plan_ = path;
  }

  auto msg = autonomy_ros::toRos(path);
  msg.header.stamp = now();
  msg.header.frame_id = map_frame_;
  for (auto & pose : msg.poses) {
    pose.header.stamp = msg.header.stamp;
    if (pose.header.frame_id.empty()) {
      pose.header.frame_id = map_frame_;
    }
  }
  plan_pub_->publish(msg);

  RCLCPP_INFO(
    get_logger(),
    "Plan ok: planner=%s poses=%d start=(%.2f, %.2f) goal=(%.2f, %.2f)",
    planner_id_.c_str(), path.poses_size(),
    start.pose().position().x(), start.pose().position().y(),
    goal.pose().position().x(), goal.pose().position().y());
  return true;
}

void PlannerSimNode::OnTick()
{
  if (robot_pose_.pending) {
    PublishRobotPose(robot_pose_.x, robot_pose_.y, robot_pose_.yaw);
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (have_odom_) {
      const double dx = latest_odom_.pose.pose.position.x - robot_pose_.x;
      const double dy = latest_odom_.pose.pose.position.y - robot_pose_.y;
      if (std::hypot(dx, dy) < kPoseConvergeDistanceM) {
        robot_pose_.pending = false;
      }
    }
  }
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
