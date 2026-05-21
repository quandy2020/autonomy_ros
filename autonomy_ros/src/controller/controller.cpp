#include "autonomy_ros/controller/controller.hpp"

#include <algorithm>
#include <cmath>

namespace autonomy_ros::controller
{

Controller::Controller(rclcpp::Node & node)
: node_(node)
{
  node_.declare_parameter("controller.max_linear_vel", max_linear_vel_);
  node_.declare_parameter("controller.max_angular_vel", max_angular_vel_);
  node_.declare_parameter("controller.goal_tolerance", goal_tolerance_);
  node_.declare_parameter("controller.lookahead", lookahead_);
  node_.declare_parameter("controller.base_frame", base_frame_);
  max_linear_vel_ = node_.get_parameter("controller.max_linear_vel").as_double();
  default_max_linear_vel_ = max_linear_vel_;
  max_angular_vel_ = node_.get_parameter("controller.max_angular_vel").as_double();
  goal_tolerance_ = node_.get_parameter("controller.goal_tolerance").as_double();
  lookahead_ = node_.get_parameter("controller.lookahead").as_double();
  base_frame_ = node_.get_parameter("controller.base_frame").as_string();
}

void Controller::start()
{
  cmd_pub_ = node_.create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
  path_sub_ = node_.create_subscription<nav_msgs::msg::Path>(
    "plan", 10, std::bind(&Controller::onPath, this, std::placeholders::_1));
  odom_sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&Controller::onOdom, this, std::placeholders::_1));
  timer_ = node_.create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&Controller::controlStep, this));
  RCLCPP_INFO(node_.get_logger(), "[controller] ready");
}

void Controller::onPath(const nav_msgs::msg::Path::SharedPtr msg)
{
  active_path_ = *msg;
  path_index_ = 0;
  RCLCPP_INFO(node_.get_logger(), "[controller] received plan (%zu poses)", msg->poses.size());
}

void Controller::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = *msg;
}

void Controller::setEnabled(bool enabled) { enabled_.store(enabled); }
bool Controller::enabled() const { return enabled_.load(); }
double Controller::goalTolerance() const { return goal_tolerance_; }

void Controller::setMaxLinearVel(double max_linear)
{
  if (max_linear > 0.0) {
    max_linear_vel_ = max_linear;
  } else {
    max_linear_vel_ = default_max_linear_vel_;
  }
}

double Controller::defaultMaxLinearVel() const { return default_max_linear_vel_; }

void Controller::controlStep()
{
  if (!enabled_.load()) {
    geometry_msgs::msg::TwistStamped stop;
    stop.header.stamp = node_.now();
    stop.header.frame_id = base_frame_;
    cmd_pub_->publish(stop);
    return;
  }
  if (!active_path_ || !latest_odom_ || active_path_->poses.empty()) {
    return;
  }

  const auto & pose = latest_odom_->pose.pose;
  const double px = pose.position.x;
  const double py = pose.position.y;

  const auto & goal = active_path_->poses.back().pose;
  const double gx = goal.position.x;
  const double gy = goal.position.y;
  const double dist_goal = std::hypot(gx - px, gy - py);

  if (dist_goal < goal_tolerance_) {
    geometry_msgs::msg::TwistStamped stop;
    stop.header.stamp = node_.now();
    stop.header.frame_id = base_frame_;
    cmd_pub_->publish(stop);
    active_path_.reset();
    RCLCPP_INFO(node_.get_logger(), "[controller] goal reached");
    return;
  }

  while (path_index_ + 1 < active_path_->poses.size()) {
    const auto & wp = active_path_->poses[path_index_].pose.position;
    if (std::hypot(wp.x - px, wp.y - py) < lookahead_) {
      ++path_index_;
    } else {
      break;
    }
  }

  const auto & target = active_path_->poses[path_index_].pose.position;
  const double dx = target.x - px;
  const double dy = target.y - py;
  const double yaw = std::atan2(
    2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
    1.0 - 2.0 * (pose.orientation.y * pose.orientation.y +
      pose.orientation.z * pose.orientation.z));
  const double heading_err = std::atan2(dy, dx) - yaw;

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_.now();
  cmd.header.frame_id = base_frame_;
  cmd.twist.linear.x = std::min(max_linear_vel_, 0.5 * dist_goal);
  cmd.twist.angular.z =
    std::clamp(2.0 * heading_err, -max_angular_vel_, max_angular_vel_);
  cmd_pub_->publish(cmd);
}

}  // namespace autonomy_ros::controller
