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

#include "autonomy_ros/rviz_tools.hpp"

#include <chrono>

#include "autolink/action/types.hpp"
#include "autonomy_ros/conversions/conversions.hpp"
#include "autonomy_ros/msg/waypoint_status.hpp"
#include "autonomy_ros/visualizer.hpp"

namespace autonomy_ros
{

RvizTools::RvizTools(
  rclcpp::Node & node,
  std::shared_ptr<autolink::Node> al_node,
  const Options & options,
  Visualizer * visualizer)
: node_(node)
, al_node_(std::move(al_node))
, options_(options)
, visualizer_(visualizer)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  al_nav_to_pose_ = autolink::action::CreateClient<AlNavigateToPose>(
    al_node_, options_.autolink.navigate_to_pose);
  al_nav_through_ = autolink::action::CreateClient<AlNavigateThroughPoses>(
    al_node_, options_.autolink.navigate_through_poses);

  navigate_to_pose_server_ = rclcpp_action::create_server<NavigateToPose>(
    &node_,
    kNavigateToPoseAction,
    std::bind(&RvizTools::HandleNavigateToPoseGoal, this, _1, _2),
    std::bind(&RvizTools::HandleNavigateToPoseCancel, this, _1),
    std::bind(&RvizTools::HandleNavigateToPoseAccepted, this, _1));

  navigate_through_poses_server_ = rclcpp_action::create_server<NavigateThroughPoses>(
    &node_,
    kNavigateThroughPosesAction,
    std::bind(&RvizTools::HandleNavigateThroughPosesGoal, this, _1, _2),
    std::bind(&RvizTools::HandleNavigateThroughPosesCancel, this, _1),
    std::bind(&RvizTools::HandleNavigateThroughPosesAccepted, this, _1));

  set_initial_pose_srv_ = node_.create_service<autonomy_ros::srv::SetInitialPose>(
    kSetInitialPoseService,
    std::bind(&RvizTools::OnSetInitialPose, this, _1, _2));

  cancel_task_srv_ = node_.create_service<autonomy_ros::srv::CancelTask>(
    kCancelTaskService,
    std::bind(&RvizTools::OnCancelTask, this, _1, _2));

  goal_pose_sub_ = node_.create_subscription<geometry_msgs::msg::PoseStamped>(
    options_.navigation.goal_pose_topic, 10,
    std::bind(&RvizTools::OnGoalPose, this, _1));

  waypoints_sub_ = node_.create_subscription<nav_msgs::msg::Path>(
    options_.navigation.waypoints_topic, 10,
    std::bind(&RvizTools::OnWaypoints, this, _1));

  RCLCPP_INFO(
    node_.get_logger(),
    "[nav] ROS actions → autolink %s / %s",
    options_.autolink.navigate_to_pose.c_str(),
    options_.autolink.navigate_through_poses.c_str());
}

RvizTools::~RvizTools()
{
  CancelRunning();
}

void RvizTools::CancelRunning()
{
  cancel_goal_.store(true);
  if (active_to_pose_handle_ && al_nav_to_pose_) {
    al_nav_to_pose_->AsyncCancelGoal(active_to_pose_handle_);
  }
  if (active_through_handle_ && al_nav_through_) {
    al_nav_through_->AsyncCancelGoal(active_through_handle_);
  }
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
  active_to_pose_handle_.reset();
  active_through_handle_.reset();
  cancel_goal_.store(false);
}

bool RvizTools::SendAlNavigateToPose(const geometry_msgs::msg::PoseStamped & pose)
{
  if (!al_nav_to_pose_) {
    return false;
  }
  if (!al_nav_to_pose_->WaitForActionServer(std::chrono::seconds(2))) {
    RCLCPP_WARN(node_.get_logger(), "[nav] autolink navigate_to_pose not ready");
    return false;
  }

  AlNavigateToPose::Goal goal;
  *goal.mutable_pose() = fromRos(pose);

  auto handle_future = al_nav_to_pose_->AsyncSendGoal(goal);
  if (handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    return false;
  }
  active_to_pose_handle_ = handle_future.get();
  if (!active_to_pose_handle_) {
    return false;
  }

  auto result_future = al_nav_to_pose_->AsyncGetResult(active_to_pose_handle_);
  while (result_future.wait_for(std::chrono::milliseconds(100)) !=
    std::future_status::ready)
  {
    if (cancel_goal_.load()) {
      al_nav_to_pose_->AsyncCancelGoal(active_to_pose_handle_);
      return false;
    }
  }
  const auto wrapped = result_future.get();
  active_to_pose_handle_.reset();
  return wrapped.code == autolink::action::ResultCode::SUCCEEDED;
}

bool RvizTools::SendAlNavigateThrough(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  if (!al_nav_through_ || poses.empty()) {
    return false;
  }
  if (!al_nav_through_->WaitForActionServer(std::chrono::seconds(2))) {
    RCLCPP_WARN(
      node_.get_logger(), "[nav] autolink navigate_through_poses not ready");
    return false;
  }

  AlNavigateThroughPoses::Goal goal;
  for (const auto & pose : poses) {
    *goal.add_poses() = fromRos(pose);
  }

  auto handle_future = al_nav_through_->AsyncSendGoal(goal);
  if (handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    return false;
  }
  active_through_handle_ = handle_future.get();
  if (!active_through_handle_) {
    return false;
  }

  auto result_future = al_nav_through_->AsyncGetResult(active_through_handle_);
  while (result_future.wait_for(std::chrono::milliseconds(100)) !=
    std::future_status::ready)
  {
    if (cancel_goal_.load()) {
      al_nav_through_->AsyncCancelGoal(active_through_handle_);
      return false;
    }
  }
  const auto wrapped = result_future.get();
  active_through_handle_.reset();
  return wrapped.code == autolink::action::ResultCode::SUCCEEDED;
}

void RvizTools::OnGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  if (visualizer_) {
    visualizer_->OnNavigationGoal(*msg);
  }
  CancelRunning();
  worker_thread_ = std::thread([this, goal = *msg]() {
    const bool ok = SendAlNavigateToPose(goal);
    if (!cancel_goal_.load()) {
      RCLCPP_INFO(node_.get_logger(), "[nav] topic A->B %s", ok ? "ok" : "failed");
    }
  });
}

void RvizTools::OnWaypoints(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (!msg || msg->poses.empty()) {
    return;
  }
  if (visualizer_) {
    visualizer_->OnNavigationGoal(msg->poses.back());
  }
  CancelRunning();
  worker_thread_ = std::thread([this, poses = msg->poses]() {
    const bool ok = SendAlNavigateThrough(poses);
    if (!cancel_goal_.load()) {
      RCLCPP_INFO(
        node_.get_logger(), "[nav] topic multi-point (%zu) %s",
        poses.size(), ok ? "ok" : "failed");
    }
  });
}

rclcpp_action::GoalResponse RvizTools::HandleNavigateToPoseGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const NavigateToPose::Goal> goal)
{
  return goal ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
              : rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse RvizTools::HandleNavigateToPoseCancel(
  const std::shared_ptr<GoalHandleNavigateToPose>)
{
  cancel_goal_.store(true);
  if (active_to_pose_handle_ && al_nav_to_pose_) {
    al_nav_to_pose_->AsyncCancelGoal(active_to_pose_handle_);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RvizTools::HandleNavigateToPoseAccepted(
  const std::shared_ptr<GoalHandleNavigateToPose> handle)
{
  CancelRunning();
  worker_thread_ = std::thread(&RvizTools::ExecuteNavigateToPose, this, handle);
}

void RvizTools::ExecuteNavigateToPose(
  const std::shared_ptr<GoalHandleNavigateToPose> handle)
{
  const auto goal = handle->get_goal();
  auto result = std::make_shared<NavigateToPose::Result>();
  if (visualizer_) {
    visualizer_->OnNavigationGoal(goal->pose);
  }

  const bool ok = SendAlNavigateToPose(goal->pose);
  if (handle->is_canceling() || cancel_goal_.load()) {
    result->error_code = NavigateToPose::Result::CANCELED;
    result->error_msg = "canceled";
    handle->canceled(result);
    return;
  }
  if (ok) {
    result->error_code = NavigateToPose::Result::NONE;
    result->error_msg = "ok";
    handle->succeed(result);
  } else {
    result->error_code = NavigateToPose::Result::FAILED;
    result->error_msg = "navigation failed";
    handle->abort(result);
  }
}

rclcpp_action::GoalResponse RvizTools::HandleNavigateThroughPosesGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const NavigateThroughPoses::Goal> goal)
{
  if (!goal || goal->poses.empty()) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RvizTools::HandleNavigateThroughPosesCancel(
  const std::shared_ptr<GoalHandleNavigateThroughPoses>)
{
  cancel_goal_.store(true);
  if (active_through_handle_ && al_nav_through_) {
    al_nav_through_->AsyncCancelGoal(active_through_handle_);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RvizTools::HandleNavigateThroughPosesAccepted(
  const std::shared_ptr<GoalHandleNavigateThroughPoses> handle)
{
  CancelRunning();
  worker_thread_ = std::thread(&RvizTools::ExecuteNavigateThroughPoses, this, handle);
}

void RvizTools::ExecuteNavigateThroughPoses(
  const std::shared_ptr<GoalHandleNavigateThroughPoses> handle)
{
  const auto goal = handle->get_goal();
  auto result = std::make_shared<NavigateThroughPoses::Result>();
  if (visualizer_ && !goal->poses.empty()) {
    visualizer_->OnNavigationGoal(goal->poses.back());
  }

  for (size_t i = 0; i < goal->poses.size(); ++i) {
    autonomy_ros::msg::WaypointStatus status;
    status.waypoint_index = static_cast<uint16_t>(i);
    status.waypoint_pose = goal->poses[i];
    status.waypoint_status = autonomy_ros::msg::WaypointStatus::PENDING;
    result->waypoint_statuses.push_back(status);
  }

  const bool ok = SendAlNavigateThrough(goal->poses);
  if (handle->is_canceling() || cancel_goal_.load()) {
    result->error_code = NavigateThroughPoses::Result::CANCELED;
    result->error_msg = "canceled";
    handle->canceled(result);
    return;
  }

  const uint8_t wp = ok ? autonomy_ros::msg::WaypointStatus::COMPLETED
                        : autonomy_ros::msg::WaypointStatus::FAILED;
  for (auto & status : result->waypoint_statuses) {
    status.waypoint_status = wp;
  }

  if (ok) {
    result->error_code = NavigateThroughPoses::Result::NONE;
    result->error_msg = "ok";
    handle->succeed(result);
  } else {
    result->error_code = NavigateThroughPoses::Result::FAILED;
    result->error_msg = "navigation failed";
    handle->abort(result);
  }
}

void RvizTools::OnSetInitialPose(
  const std::shared_ptr<autonomy_ros::srv::SetInitialPose::Request> request,
  std::shared_ptr<autonomy_ros::srv::SetInitialPose::Response> response)
{
  RCLCPP_INFO(
    node_.get_logger(),
    "[nav] set_initial_pose frame=%s x=%.3f y=%.3f",
    request->pose.header.frame_id.c_str(),
    request->pose.pose.pose.position.x,
    request->pose.pose.pose.position.y);
  response->success = true;
  response->message = "accepted (forward to localization via autolink if configured)";
}

void RvizTools::OnCancelTask(
  const std::shared_ptr<autonomy_ros::srv::CancelTask::Request>,
  std::shared_ptr<autonomy_ros::srv::CancelTask::Response> response)
{
  CancelRunning();
  response->success = true;
  response->message = "canceled";
}

}  // namespace autonomy_ros
