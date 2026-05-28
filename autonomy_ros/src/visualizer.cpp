/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
 * email: quandy2020@126.com
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

#include "autonomy_ros/visualizer.hpp"

namespace autonomy_ros
{

Visualizer::Visualizer(rclcpp::Node & node, std::string frame_id)
: node_(node), frame_id_(std::move(frame_id))
{
  plan_publisher_ = node_.create_publisher<nav_msgs::msg::Path>(kPlanTopic, 10);
  goal_publisher_ = node_.create_publisher<geometry_msgs::msg::PoseStamped>(
    kNavigationGoalTopic, 10);
  robot_pose_publisher_ = node_.create_publisher<geometry_msgs::msg::PoseStamped>(
    kRobotPoseTopic, 10);

  RCLCPP_INFO(
    node_.get_logger(),
    "[visualization] plan=%s goal=%s robot_pose=%s (frame=%s)",
    kPlanTopic, kNavigationGoalTopic, kRobotPoseTopic, frame_id_.c_str());
}

void Visualizer::OnGlobalPath(const nav_msgs::msg::Path & path)
{
  if (!plan_publisher_) {
    return;
  }

  nav_msgs::msg::Path published = path;
  if (published.header.frame_id.empty()) {
    published.header.frame_id = frame_id_;
  }
  if (published.header.stamp.sec == 0 && published.header.stamp.nanosec == 0) {
    published.header.stamp = node_.now();
  }

  for (auto & pose : published.poses) {
    if (pose.header.frame_id.empty()) {
      pose.header.frame_id = published.header.frame_id;
    }
    if (pose.header.stamp.sec == 0 && pose.header.stamp.nanosec == 0) {
      pose.header.stamp = published.header.stamp;
    }
  }

  plan_publisher_->publish(published);
}

void Visualizer::OnNavigationGoal(const geometry_msgs::msg::PoseStamped & goal)
{
  if (!goal_publisher_) {
    return;
  }
  geometry_msgs::msg::PoseStamped stamped = goal;
  if (stamped.header.frame_id.empty()) {
    stamped.header.frame_id = frame_id_;
  }
  if (stamped.header.stamp.sec == 0 && stamped.header.stamp.nanosec == 0) {
    stamped.header.stamp = node_.now();
  }
  goal_publisher_->publish(stamped);
}

void Visualizer::OnRobotPose(const nav_msgs::msg::Odometry & odom)
{
  if (!robot_pose_publisher_) {
    return;
  }
  geometry_msgs::msg::PoseStamped pose;
  pose.header = odom.header;
  if (pose.header.frame_id.empty()) {
    pose.header.frame_id = frame_id_;
  }
  pose.pose = odom.pose.pose;
  robot_pose_publisher_->publish(pose);
}

}  // namespace autonomy_ros
