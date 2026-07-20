/*
 * Copyright 2026 The Openbot Authors
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

#include "autonomy_exploration/exploration_node.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "autonomy/exploration/planner/exploration_planner.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/geometry_msgs/transform_stamped.h"
#include "autonomy_exploration/conversions.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace autonomy_exploration {
namespace {

double YawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
  return yaw;
}

double NormalizeAngle(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

::geometry_msgs::TransformStamped ToAutonomyTf2(
  const geometry_msgs::msg::TransformStamped & ros_tf)
{
  ::geometry_msgs::TransformStamped out;
  out.header.frame_id = ros_tf.header.frame_id;
  out.header.stamp =
    static_cast<uint64_t>(ros_tf.header.stamp.sec) * 1000000000ULL +
    static_cast<uint64_t>(ros_tf.header.stamp.nanosec);
  out.child_frame_id = ros_tf.child_frame_id;
  out.transform.translation.x = ros_tf.transform.translation.x;
  out.transform.translation.y = ros_tf.transform.translation.y;
  out.transform.translation.z = ros_tf.transform.translation.z;
  out.transform.rotation.x = ros_tf.transform.rotation.x;
  out.transform.rotation.y = ros_tf.transform.rotation.y;
  out.transform.rotation.z = ros_tf.transform.rotation.z;
  out.transform.rotation.w = ros_tf.transform.rotation.w;
  return out;
}

}  // namespace

ExplorationNode::ExplorationNode(const rclcpp::NodeOptions & options)
: Node("exploration_node", options),
  depth_sub_(this, "camera/depth/image_raw"),
  info_sub_(this, "camera/depth/camera_info")
{
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  camera_frame_ =
    declare_parameter<std::string>("camera_frame", "camera_optical_frame");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "odom");
  depth_topic_ =
    declare_parameter<std::string>("depth_topic", "camera/depth/image_raw");
  camera_info_topic_ = declare_parameter<std::string>(
    "camera_info_topic", "camera/rgb/camera_info");
  planner_period_sec_ = declare_parameter<double>("planner_period_sec", 0.5);
  waypoint_reach_dist_ = declare_parameter<double>("waypoint_reach_dist", 0.6);
  max_linear_vel_ = declare_parameter<double>("max_linear_vel", 0.25);
  max_angular_vel_ = declare_parameter<double>("max_angular_vel", 0.6);
  enable_cmd_vel_ = declare_parameter<bool>("enable_cmd_vel", true);
  const std::string config_dir =
    declare_parameter<std::string>("exploration_config_dir", "");

  // Remap synced subscribers if topics differ from constructor defaults.
  depth_sub_.unsubscribe();
  info_sub_.unsubscribe();
  depth_sub_.subscribe(this, depth_topic_);
  info_sub_.subscribe(this, camera_info_topic_);

  auto options_proto = ::autonomy::exploration::DefaultOptions();
  if (!config_dir.empty()) {
    options_proto = ::autonomy::exploration::CreateOptions(config_dir);
  }
  options_proto.set_planner_frequency(1.0 / std::max(planner_period_sec_, 0.1));

  server_ =
    std::make_unique<::autonomy::exploration::ExplorationServer>(options_proto);
  server_->Start();
  ::autonomy::transform::Buffer::Instance()->Init();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ExplorationNode::OnOdometry, this, std::placeholders::_1));

  sync_ = std::make_shared<message_filters::Synchronizer<DepthSyncPolicy>>(
    DepthSyncPolicy(10), depth_sub_, info_sub_);
  sync_->registerCallback(std::bind(
    &ExplorationNode::OnDepth, this, std::placeholders::_1,
    std::placeholders::_2));

  path_pub_ = create_publisher<nav_msgs::msg::Path>("exploration/path", 10);
  waypoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "exploration/waypoint", 10);
  costmap_pub_ =
    create_publisher<nav_msgs::msg::OccupancyGrid>("exploration/costmap", 1);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "exploration/markers", 10);
  progress_pub_ =
    create_publisher<std_msgs::msg::Float32>("exploration/progress", 10);
  if (enable_cmd_vel_) {
    cmd_vel_pub_ =
      create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

  timer_ = create_wall_timer(
    std::chrono::duration<double>(planner_period_sec_),
    std::bind(&ExplorationNode::OnTimer, this));

  RCLCPP_INFO(
    get_logger(),
    "exploration_node ready (odom=%s depth=%s camera_frame=%s)",
    odom_topic_.c_str(), depth_topic_.c_str(), camera_frame_.c_str());
}

void ExplorationNode::FeedAutonomyTf(
  const geometry_msgs::msg::TransformStamped & ros_tf) const
{
  auto * buf = ::autonomy::transform::Buffer::Instance();
  if (!buf) {
    return;
  }
  buf->setTransform(ToAutonomyTf2(ros_tf), "ros_bridge", false);
}

bool ExplorationNode::LookupMapTCamera(
  const std::string & camera_frame,
  autonomy::commsgs::geometry_msgs::Transform * out)
{
  if (!out) {
    return false;
  }
  const std::string cam =
    camera_frame.empty() ? camera_frame_ : camera_frame;
  if (!tf_buffer_->canTransform(
      map_frame_, cam, tf2::TimePointZero, tf2::durationFromSec(0.0)))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Waiting for TF %s <- %s (is habitat_odom_tf / robot_state_publisher up?)",
      map_frame_.c_str(), cam.c_str());
    return false;
  }
  try {
    const auto ros_tf = tf_buffer_->lookupTransform(
      map_frame_, cam, tf2::TimePointZero);
    FeedAutonomyTf(ros_tf);
    const auto stamped = convert::FromRos(ros_tf);
    *out = stamped.transform;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "TF %s<-%s failed: %s",
      map_frame_.c_str(), cam.c_str(), ex.what());
    return false;
  }
}

void ExplorationNode::OnOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (!msg || !server_) {
    return;
  }
  const auto odom = convert::FromRos(*msg);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    server_->UpdateOdometry(odom);
    has_odom_ = true;
  }

  try {
    geometry_msgs::msg::TransformStamped tf;
    tf.header = msg->header;
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = msg->child_frame_id.empty() ? "base_footprint"
                                                    : msg->child_frame_id;
    tf.transform.translation.x = msg->pose.pose.position.x;
    tf.transform.translation.y = msg->pose.pose.position.y;
    tf.transform.translation.z = msg->pose.pose.position.z;
    tf.transform.rotation = msg->pose.pose.orientation;
    // Prefer map←base from the ROS TF tree; otherwise synthesize from odom pose.
    if (tf_buffer_->canTransform(
        map_frame_, tf.child_frame_id, tf2::TimePointZero,
        tf2::durationFromSec(0.0)))
    {
      const auto map_t_base = tf_buffer_->lookupTransform(
        map_frame_, tf.child_frame_id, tf2::TimePointZero);
      FeedAutonomyTf(map_t_base);
    } else {
      FeedAutonomyTf(tf);
    }
    // Keep map←camera available for autonomy Buffer once the robot TF tree is up.
    if (tf_buffer_->canTransform(
        map_frame_, camera_frame_, tf2::TimePointZero,
        tf2::durationFromSec(0.0)))
    {
      FeedAutonomyTf(tf_buffer_->lookupTransform(
        map_frame_, camera_frame_, tf2::TimePointZero));
    }
  } catch (...) {
  }
}

void ExplorationNode::OnDepth(
  const sensor_msgs::msg::Image::ConstSharedPtr depth,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr info)
{
  if (!depth || !info || !server_) {
    return;
  }
  autonomy::commsgs::geometry_msgs::Transform map_t_camera;
  const std::string cam_frame =
    depth->header.frame_id.empty() ? camera_frame_ : depth->header.frame_id;
  if (!LookupMapTCamera(cam_frame, &map_t_camera)) {
    return;
  }
  const auto depth_com = convert::FromRos(*depth);
  const auto info_com = convert::FromRos(*info);
  std::lock_guard<std::mutex> lock(mutex_);
  server_->UpdateDepth(depth_com, info_com, map_t_camera);
}

void ExplorationNode::OnTimer()
{
  if (!server_) {
    return;
  }
  PublishVisualization();

  geometry_msgs::msg::PoseStamped wp;
  bool have_wp = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_odom_) {
      return;
    }
    autonomy::commsgs::geometry_msgs::PoseStamped comms_wp;
    if (server_->GetNextWaypoint(comms_wp)) {
      wp = convert::ToRos(comms_wp);
      have_wp = true;
    }
  }
  if (!have_wp) {
    return;
  }
  waypoint_pub_->publish(wp);

  // Advance when close to waypoint.
  try {
    const auto map_t_base = tf_buffer_->lookupTransform(
      map_frame_, "base_footprint", tf2::TimePointZero);
    const double dx =
      wp.pose.position.x - map_t_base.transform.translation.x;
    const double dy =
      wp.pose.position.y - map_t_base.transform.translation.y;
    const double dist = std::hypot(dx, dy);
    if (dist < waypoint_reach_dist_) {
      std::lock_guard<std::mutex> lock(mutex_);
      server_->MarkWaypointReached();
    } else if (enable_cmd_vel_ && cmd_vel_pub_) {
      PublishCmdVelTowardWaypoint(wp);
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "base TF failed: %s", ex.what());
  }
}

void ExplorationNode::PublishCmdVelTowardWaypoint(
  const geometry_msgs::msg::PoseStamped & waypoint)
{
  geometry_msgs::msg::TransformStamped map_t_base;
  try {
    map_t_base = tf_buffer_->lookupTransform(
      map_frame_, "base_footprint", tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return;
  }
  const double dx =
    waypoint.pose.position.x - map_t_base.transform.translation.x;
  const double dy =
    waypoint.pose.position.y - map_t_base.transform.translation.y;
  const double target_yaw = std::atan2(dy, dx);
  const double yaw = YawFromQuat(map_t_base.transform.rotation);
  const double yaw_err = NormalizeAngle(target_yaw - yaw);
  const double dist = std::hypot(dx, dy);

  geometry_msgs::msg::Twist cmd;
  cmd.angular.z =
    std::clamp(1.5 * yaw_err, -max_angular_vel_, max_angular_vel_);
  if (std::abs(yaw_err) < 0.6) {
    cmd.linear.x = std::clamp(0.4 * dist, 0.0, max_linear_vel_);
  }
  cmd_vel_pub_->publish(cmd);
}

void ExplorationNode::PublishVisualization()
{
  auto explorer = server_->ActiveExplorer();
  if (!explorer) {
    return;
  }

  const auto path_com = explorer->GetExplorationPath();
  auto path = convert::ToRos(path_com);
  path.header.stamp = now();
  path.header.frame_id = map_frame_;
  for (auto & pose : path.poses) {
    pose.header.stamp = path.header.stamp;
    if (pose.header.frame_id.empty()) {
      pose.header.frame_id = map_frame_;
    }
  }
  path_pub_->publish(path);

  auto grid = convert::ToRos(explorer->GetOccupancyGrid(map_frame_));
  grid.header.stamp = now();
  grid.header.frame_id = map_frame_;
  costmap_pub_->publish(grid);

  std_msgs::msg::Float32 progress;
  progress.data = server_->Progress();
  progress_pub_->publish(progress);

  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  visualization_msgs::msg::Marker path_line;
  path_line.header.frame_id = map_frame_;
  path_line.header.stamp = now();
  path_line.ns = "exploration_path";
  path_line.id = 0;
  path_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_line.action = visualization_msgs::msg::Marker::ADD;
  path_line.scale.x = 0.05;
  path_line.color.r = 0.1f;
  path_line.color.g = 0.8f;
  path_line.color.b = 0.2f;
  path_line.color.a = 1.0f;
  path_line.pose.orientation.w = 1.0;
  for (const auto & pose : path.poses) {
    geometry_msgs::msg::Point p;
    p.x = pose.pose.position.x;
    p.y = pose.pose.position.y;
    p.z = pose.pose.position.z + 0.05;
    path_line.points.push_back(p);
  }
  if (path_line.points.size() >= 2) {
    markers.markers.push_back(path_line);
  }

  autonomy::commsgs::geometry_msgs::PoseStamped comms_wp;
  if (server_->GetNextWaypoint(comms_wp)) {
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = map_frame_;
    sphere.header.stamp = now();
    sphere.ns = "waypoint";
    sphere.id = 1;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose = convert::ToRos(comms_wp).pose;
    sphere.pose.position.z += 0.15;
    sphere.scale.x = 0.35;
    sphere.scale.y = 0.35;
    sphere.scale.z = 0.35;
    sphere.color.r = 1.0f;
    sphere.color.g = 0.2f;
    sphere.color.b = 0.1f;
    sphere.color.a = 0.9f;
    markers.markers.push_back(sphere);
  }

  // Frontier / coverage targets (so markers are not only the foot waypoint).
  auto * rgbd = dynamic_cast<::autonomy::exploration::planner::ExplorationPlanner *>(
    explorer.get());
  if (rgbd) {
    const auto & targets = rgbd->hierarchical().env().targets();
    const auto & frontiers = rgbd->hierarchical().env().frontiers();
    const auto & pts_src = !targets.empty() ? targets : frontiers;
    visualization_msgs::msg::Marker pts;
    pts.header.frame_id = map_frame_;
    pts.header.stamp = now();
    pts.ns = "targets";
    pts.id = 2;
    pts.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    pts.action = visualization_msgs::msg::Marker::ADD;
    pts.pose.orientation.w = 1.0;
    pts.scale.x = 0.2;
    pts.scale.y = 0.2;
    pts.scale.z = 0.2;
    pts.color.r = 0.2f;
    pts.color.g = 0.6f;
    pts.color.b = 1.0f;
    pts.color.a = 0.85f;
    for (const auto & t : pts_src) {
      geometry_msgs::msg::Point p;
      p.x = t.x;
      p.y = t.y;
      p.z = t.z + 0.1;
      pts.points.push_back(p);
    }
    if (!pts.points.empty()) {
      markers.markers.push_back(pts);
    }
  }
  marker_pub_->publish(markers);
}

}  // namespace autonomy_exploration
