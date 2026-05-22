// Copyright 2025 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autonomy_ros/visualization/visualizer.hpp"

namespace autonomy_ros::visualization
{

Visualizer::Visualizer(rclcpp::Node & node)
: node_(node)
{
  node_.declare_parameter("visualization.frame_id", frame_id_);
  frame_id_ = node_.get_parameter("visualization.frame_id").as_string();
}

void Visualizer::start()
{
  marker_pub_ = node_.create_publisher<visualization_msgs::msg::MarkerArray>(
    "visualization/markers", 10);
  goal_sub_ = node_.create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", 10, std::bind(&Visualizer::onGoal, this, std::placeholders::_1));
  RCLCPP_INFO(
    node_.get_logger(), "[visualization] markers (frame=%s, plan/odom via Autonomy)",
    frame_id_.c_str());
}

void Visualizer::onPlan(const nav_msgs::msg::Path & path)
{
  latest_path_ = path;
  publishMarkers();
}

void Visualizer::onRobotPose(const nav_msgs::msg::Odometry & odom)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = odom.header;
  pose.pose = odom.pose.pose;
  latest_robot_pose_ = pose;
  publishMarkers();
}

void Visualizer::onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (msg) {
    latest_goal_ = *msg;
    publishMarkers();
  }
}

void Visualizer::publishMarkers()
{
  if (!marker_pub_) {
    return;
  }
  visualization_msgs::msg::MarkerArray array;
  const auto stamp = node_.now();

  if (latest_path_ && latest_path_->poses.size() >= 2) {
    visualization_msgs::msg::Marker line;
    line.header.stamp = stamp;
    line.header.frame_id = frame_id_;
    line.ns = "plan";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.03;
    line.color.r = 0.1f;
    line.color.g = 0.8f;
    line.color.b = 0.2f;
    line.color.a = 1.0f;
    line.pose.orientation.w = 1.0;
    for (const auto & ps : latest_path_->poses) {
      line.points.push_back(ps.pose.position);
    }
    array.markers.push_back(line);
  }

  if (latest_robot_pose_) {
    visualization_msgs::msg::Marker robot;
    robot.header.stamp = stamp;
    robot.header.frame_id = frame_id_;
    robot.ns = "robot";
    robot.id = 2;
    robot.type = visualization_msgs::msg::Marker::ARROW;
    robot.action = visualization_msgs::msg::Marker::ADD;
    robot.pose = latest_robot_pose_->pose;
    robot.scale.x = 0.25;
    robot.scale.y = 0.08;
    robot.scale.z = 0.08;
    robot.color.r = 0.2f;
    robot.color.g = 0.4f;
    robot.color.b = 1.0f;
    robot.color.a = 1.0f;
    array.markers.push_back(robot);
  }

  if (latest_goal_) {
    visualization_msgs::msg::Marker goal;
    goal.header.stamp = stamp;
    goal.header.frame_id = frame_id_;
    goal.ns = "goal";
    goal.id = 1;
    goal.type = visualization_msgs::msg::Marker::SPHERE;
    goal.action = visualization_msgs::msg::Marker::ADD;
    goal.pose = latest_goal_->pose;
    goal.scale.x = 0.15;
    goal.scale.y = 0.15;
    goal.scale.z = 0.15;
    goal.color.r = 1.0f;
    goal.color.a = 1.0f;
    array.markers.push_back(goal);
  }

  marker_pub_->publish(array);
}

}  // namespace autonomy_ros::visualization
