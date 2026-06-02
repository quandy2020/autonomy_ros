// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autonomy_simulator/fake_robot_node.hpp"

#include <cmath>
#include <functional>

#include "tf2/LinearMath/Quaternion.h"

namespace autonomy_simulator
{

FakeRobotNode::FakeRobotNode()
: Node("fake_robot_node")
{
  loadParameters();
  initState();

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, qos);
  joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>(
    joint_states_topic_, qos);
  tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>(tf_topic_, qos);

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    cmd_vel_topic_, qos,
    std::bind(&FakeRobotNode::onCmdVel, this, std::placeholders::_1));
  set_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "fake_robot/set_pose", qos,
    std::bind(&FakeRobotNode::onSetPose, this, std::placeholders::_1));

  const auto period_ms = static_cast<int>(1000.0 / update_rate_hz_);
  update_timer_ = create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&FakeRobotNode::onUpdate, this));

  RCLCPP_INFO(
    get_logger(),
    "[fake_robot] odom=%s cmd_vel=%s (%s) wheels: sep=%.3f r=%.3f",
    odom_topic_.c_str(), cmd_vel_topic_.c_str(), "TwistStamped",
    wheel_separation_, wheel_radius_);
}

void FakeRobotNode::loadParameters()
{
  declare_parameter<std::string>("odom_topic", odom_topic_);
  declare_parameter<std::string>("cmd_vel_topic", cmd_vel_topic_);
  declare_parameter<std::string>("joint_states_topic", joint_states_topic_);
  declare_parameter<std::string>("tf_topic", tf_topic_);
  declare_parameter<std::string>("odom_frame", odom_frame_);
  declare_parameter<std::string>("base_frame", base_frame_);
  declare_parameter<std::string>("joint_states_frame", joint_states_frame_);
  declare_parameter<double>("wheels.separation", wheel_separation_);
  declare_parameter<double>("wheels.radius", wheel_radius_);
  declare_parameter<double>("cmd_vel_timeout", cmd_vel_timeout_);
  declare_parameter<double>("update_rate_hz", update_rate_hz_);

  odom_topic_ = get_parameter("odom_topic").as_string();
  cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
  joint_states_topic_ = get_parameter("joint_states_topic").as_string();
  tf_topic_ = get_parameter("tf_topic").as_string();
  odom_frame_ = get_parameter("odom_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  joint_states_frame_ = get_parameter("joint_states_frame").as_string();
  wheel_separation_ = get_parameter("wheels.separation").as_double();
  wheel_radius_ = get_parameter("wheels.radius").as_double();
  cmd_vel_timeout_ = get_parameter("cmd_vel_timeout").as_double();
  update_rate_hz_ = get_parameter("update_rate_hz").as_double();
}

void FakeRobotNode::initState()
{
  wheel_speed_cmd_ = {0.0, 0.0};
  last_wheel_pos_ = {0.0, 0.0};
  last_wheel_vel_ = {0.0, 0.0};
  goal_linear_ = 0.0;
  goal_angular_ = 0.0;
  pose_ = {0.0F, 0.0F, 0.0F};
  vel_ = {0.0F, 0.0F, 0.0F};

  odom_.header.frame_id = odom_frame_;
  odom_.child_frame_id = base_frame_;

  joint_states_.header.frame_id = joint_states_frame_;
  joint_states_.name = {"wheel_left_joint", "wheel_right_joint"};
  joint_states_.position.resize(2, 0.0);
  joint_states_.velocity.resize(2, 0.0);
  joint_states_.effort.resize(2, 0.0);

  prev_update_time_ = now();
  last_cmd_vel_time_ = now();
}

void FakeRobotNode::onSetPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  pose_[0] = static_cast<float>(msg->pose.position.x);
  pose_[1] = static_cast<float>(msg->pose.position.y);
  const auto & q = msg->pose.orientation;
  pose_[2] = static_cast<float>(std::atan2(2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
  vel_ = {0.0F, 0.0F, 0.0F};
  wheel_speed_cmd_ = {0.0, 0.0};
  goal_linear_ = 0.0;
  goal_angular_ = 0.0;
  RCLCPP_INFO(
    get_logger(), "[fake_robot] pose reset to (%.3f, %.3f, %.3f)",
    pose_[0], pose_[1], pose_[2]);
}

void FakeRobotNode::onCmdVel(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  last_cmd_vel_time_ = now();
  goal_linear_ = msg->twist.linear.x;
  goal_angular_ = msg->twist.angular.z;

  wheel_speed_cmd_[kLeft] = goal_linear_ - (goal_angular_ * wheel_separation_ / 2.0);
  wheel_speed_cmd_[kRight] = goal_linear_ + (goal_angular_ * wheel_separation_ / 2.0);
}

void FakeRobotNode::onUpdate()
{
  const auto time_now = now();
  const rclcpp::Duration duration(time_now - prev_update_time_);
  prev_update_time_ = time_now;

  if ((time_now - last_cmd_vel_time_).seconds() > cmd_vel_timeout_) {
    wheel_speed_cmd_ = {0.0, 0.0};
  }

  integrateOdometry(duration);

  odom_.header.stamp = time_now;
  odom_pub_->publish(odom_);

  publishJointStates(time_now);
  publishTf(time_now);
}

bool FakeRobotNode::integrateOdometry(const rclcpp::Duration & duration)
{
  const double step_time = duration.seconds();
  if (step_time <= 0.0) {
    return false;
  }

  double wheel_l = 0.0;
  double wheel_r = 0.0;

  const double v_left = wheel_speed_cmd_[kLeft];
  const double v_right = wheel_speed_cmd_[kRight];
  const double w_left = v_left / wheel_radius_;
  const double w_right = v_right / wheel_radius_;

  last_wheel_vel_[kLeft] = w_left;
  last_wheel_vel_[kRight] = w_right;

  wheel_l = w_left * step_time;
  wheel_r = w_right * step_time;

  if (std::isnan(wheel_l)) {
    wheel_l = 0.0;
  }
  if (std::isnan(wheel_r)) {
    wheel_r = 0.0;
  }

  last_wheel_pos_[kLeft] += wheel_l;
  last_wheel_pos_[kRight] += wheel_r;

  const double delta_s = wheel_radius_ * (wheel_r + wheel_l) / 2.0;
  const double delta_theta = wheel_radius_ * (wheel_r - wheel_l) / wheel_separation_;

  pose_[0] += static_cast<float>(delta_s * std::cos(pose_[2] + delta_theta / 2.0));
  pose_[1] += static_cast<float>(delta_s * std::sin(pose_[2] + delta_theta / 2.0));
  pose_[2] += static_cast<float>(delta_theta);

  vel_[0] = static_cast<float>(delta_s / step_time);
  vel_[1] = 0.0F;
  vel_[2] = static_cast<float>(delta_theta / step_time);

  odom_.pose.pose.position.x = pose_[0];
  odom_.pose.pose.position.y = pose_[1];
  odom_.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose_[2]);
  odom_.pose.pose.orientation.x = q.x();
  odom_.pose.pose.orientation.y = q.y();
  odom_.pose.pose.orientation.z = q.z();
  odom_.pose.pose.orientation.w = q.w();

  odom_.twist.twist.linear.x = vel_[0];
  odom_.twist.twist.angular.z = vel_[2];

  return true;
}

void FakeRobotNode::publishJointStates(const rclcpp::Time & stamp)
{
  joint_states_.header.stamp = stamp;
  joint_states_.position[kLeft] = last_wheel_pos_[kLeft];
  joint_states_.position[kRight] = last_wheel_pos_[kRight];
  joint_states_.velocity[kLeft] = last_wheel_vel_[kLeft];
  joint_states_.velocity[kRight] = last_wheel_vel_[kRight];
  joint_states_pub_->publish(joint_states_);
}

void FakeRobotNode::publishTf(const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped odom_tf;
  odom_tf.header.stamp = stamp;
  odom_tf.header.frame_id = odom_.header.frame_id;
  odom_tf.child_frame_id = odom_.child_frame_id;
  odom_tf.transform.translation.x = odom_.pose.pose.position.x;
  odom_tf.transform.translation.y = odom_.pose.pose.position.y;
  odom_tf.transform.translation.z = odom_.pose.pose.position.z;
  odom_tf.transform.rotation = odom_.pose.pose.orientation;

  tf2_msgs::msg::TFMessage msg;
  msg.transforms.push_back(odom_tf);
  tf_pub_->publish(msg);
}

}  // namespace autonomy_simulator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy_simulator::FakeRobotNode>());
  rclcpp::shutdown();
  return 0;
}
