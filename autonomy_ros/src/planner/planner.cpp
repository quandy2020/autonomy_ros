#include "autonomy_ros/planner/planner.hpp"

#include <cmath>

namespace autonomy_ros::planner
{

Planner::Planner(rclcpp::Node & node, map::MapManager & map_manager)
: node_(node), map_manager_(map_manager)
{
  node_.declare_parameter("planner.waypoint_spacing", waypoint_spacing_);
  node_.declare_parameter("planner.global_frame", global_frame_);
  waypoint_spacing_ = node_.get_parameter("planner.waypoint_spacing").as_double();
  global_frame_ = node_.get_parameter("planner.global_frame").as_string();
}

void Planner::start()
{
  path_pub_ = node_.create_publisher<nav_msgs::msg::Path>("plan", 10);
  odom_sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&Planner::onOdom, this, std::placeholders::_1));
  goal_sub_ = node_.create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", 10,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { setGoal(*msg); });
  RCLCPP_INFO(node_.get_logger(), "[planner] ready (frame=%s)", global_frame_.c_str());
}

void Planner::setGoal(const geometry_msgs::msg::PoseStamped & goal)
{
  pending_goal_ = goal;
  if (!latest_odom_) {
    RCLCPP_WARN(node_.get_logger(), "[planner] waiting for odom before planning");
    return;
  }
  publishPath(latest_odom_->pose.pose, goal.pose);
  pending_goal_.reset();
}

void Planner::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = *msg;
  if (pending_goal_) {
    publishPath(msg->pose.pose, pending_goal_->pose);
    pending_goal_.reset();
  }
}

void Planner::publishPath(
  const geometry_msgs::msg::Pose & start, const geometry_msgs::msg::Pose & goal)
{
  (void)map_manager_.map();

  const double dx = goal.position.x - start.position.x;
  const double dy = goal.position.y - start.position.y;
  const double dist = std::hypot(dx, dy);
  const size_t steps = std::max<size_t>(
    2, static_cast<size_t>(std::ceil(dist / waypoint_spacing_)) + 1);

  nav_msgs::msg::Path path;
  path.header.stamp = node_.now();
  path.header.frame_id = global_frame_;

  for (size_t i = 0; i < steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps - 1);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = start.position.x + t * dx;
    pose.pose.position.y = start.position.y + t * dy;
    pose.pose.position.z = start.position.z;
    pose.pose.orientation = goal.orientation;
    path.poses.push_back(pose);
  }

  last_path_ = path;
  path_pub_->publish(path);
  RCLCPP_INFO(
    node_.get_logger(), "[planner] published path with %zu poses (%.2f m)",
    path.poses.size(), dist);
}

std::optional<nav_msgs::msg::Path> Planner::lastPath() const
{
  return last_path_;
}

}  // namespace autonomy_ros::planner
