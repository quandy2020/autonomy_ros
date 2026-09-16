/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

#ifndef AUTONOMY_ROS__RVIZ_TOOLS_HPP_
#define AUTONOMY_ROS__RVIZ_TOOLS_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// rclcpp_action already specializes std::hash/less for the same GoalUUID
// underlying type (std::array<uint8_t,16>); skip autolink's copies.
#define AUTOLINK_SKIP_GOAL_UUID_STD_HASH

#include "autolink/action/create_client.hpp"
#include "autolink/node/node.hpp"
#include <automsgs/actions/nav_actions.pb.h>
#include "autonomy_ros/action/navigate_through_poses.hpp"
#include "autonomy_ros/action/navigate_to_pose.hpp"
#include "autonomy_ros/constants.hpp"
#include "autonomy_ros/options.hpp"
#include "autonomy_ros/srv/cancel_task.hpp"
#include "autonomy_ros/srv/set_initial_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace autonomy_ros
{
class Visualizer;

/**
 * @brief ROS Nav2-style API bridged to autonomy via autolink actions.
 *
 * Does not link libautonomy; sends NavigateToPose / NavigateThroughPoses
 * goals to the autonomy process over autolink.
 */
class RvizTools
{
public:
  using NavigateToPose = autonomy_ros::action::NavigateToPose;
  using NavigateThroughPoses = autonomy_ros::action::NavigateThroughPoses;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;
  using GoalHandleNavigateThroughPoses =
    rclcpp_action::ServerGoalHandle<NavigateThroughPoses>;

  using AlNavigateToPose = automsgs::actions::NavigateToPoseAction;
  using AlNavigateThroughPoses = automsgs::actions::NavigateThroughPosesAction;
  using AlNavToPoseClient = autolink::action::Client<AlNavigateToPose>;
  using AlNavThroughClient = autolink::action::Client<AlNavigateThroughPoses>;

  RvizTools(
    rclcpp::Node & node,
    std::shared_ptr<autolink::Node> al_node,
    const Options & options,
    Visualizer * visualizer = nullptr);

  ~RvizTools();

private:
  void OnGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void OnWaypoints(const nav_msgs::msg::Path::SharedPtr msg);

  rclcpp_action::GoalResponse HandleNavigateToPoseGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal);
  rclcpp_action::CancelResponse HandleNavigateToPoseCancel(
    const std::shared_ptr<GoalHandleNavigateToPose> handle);
  void HandleNavigateToPoseAccepted(
    const std::shared_ptr<GoalHandleNavigateToPose> handle);
  void ExecuteNavigateToPose(const std::shared_ptr<GoalHandleNavigateToPose> handle);

  rclcpp_action::GoalResponse HandleNavigateThroughPosesGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateThroughPoses::Goal> goal);
  rclcpp_action::CancelResponse HandleNavigateThroughPosesCancel(
    const std::shared_ptr<GoalHandleNavigateThroughPoses> handle);
  void HandleNavigateThroughPosesAccepted(
    const std::shared_ptr<GoalHandleNavigateThroughPoses> handle);
  void ExecuteNavigateThroughPoses(
    const std::shared_ptr<GoalHandleNavigateThroughPoses> handle);

  void OnSetInitialPose(
    const std::shared_ptr<autonomy_ros::srv::SetInitialPose::Request> request,
    std::shared_ptr<autonomy_ros::srv::SetInitialPose::Response> response);
  void OnCancelTask(
    const std::shared_ptr<autonomy_ros::srv::CancelTask::Request> request,
    std::shared_ptr<autonomy_ros::srv::CancelTask::Response> response);

  void CancelRunning();
  bool SendAlNavigateToPose(const geometry_msgs::msg::PoseStamped & pose);
  bool SendAlNavigateThrough(
    const std::vector<geometry_msgs::msg::PoseStamped> & poses);

  rclcpp::Node & node_;
  std::shared_ptr<autolink::Node> al_node_;
  Options options_;
  Visualizer * visualizer_{nullptr};

  std::shared_ptr<AlNavToPoseClient> al_nav_to_pose_;
  std::shared_ptr<AlNavThroughClient> al_nav_through_;

  rclcpp_action::Server<NavigateToPose>::SharedPtr navigate_to_pose_server_;
  rclcpp_action::Server<NavigateThroughPoses>::SharedPtr navigate_through_poses_server_;
  rclcpp::Service<autonomy_ros::srv::SetInitialPose>::SharedPtr set_initial_pose_srv_;
  rclcpp::Service<autonomy_ros::srv::CancelTask>::SharedPtr cancel_task_srv_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoints_sub_;

  std::atomic<bool> cancel_goal_{false};
  std::shared_ptr<AlNavToPoseClient::GoalHandle> active_to_pose_handle_;
  std::shared_ptr<AlNavThroughClient::GoalHandle> active_through_handle_;
  std::thread worker_thread_;
};

}  // namespace autonomy_ros

#endif  // AUTONOMY_ROS__RVIZ_TOOLS_HPP_
