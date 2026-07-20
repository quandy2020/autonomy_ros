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

#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "autonomy/exploration/exploration_options.hpp"
#include "autonomy/exploration/exploration_server.hpp"

namespace autonomy_exploration {

/**
 * @brief ROS 2 node wrapping autonomy::exploration::ExplorationServer.
 *
 * Consumes Habitat (or any) odom + RGB-D, drives hierarchical exploration,
 * and publishes path / waypoint / local costmap for RViz2.
 */
class ExplorationNode : public rclcpp::Node
{
public:
  explicit ExplorationNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using DepthSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;

  void OnOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void OnDepth(
    const sensor_msgs::msg::Image::ConstSharedPtr depth,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr info);
  void OnTimer();
  void PublishVisualization();
  void PublishCmdVelTowardWaypoint(
    const geometry_msgs::msg::PoseStamped & waypoint);
  bool LookupMapTCamera(
    const std::string & camera_frame,
    autonomy::commsgs::geometry_msgs::Transform * out);
  void FeedAutonomyTf(
    const geometry_msgs::msg::TransformStamped & ros_tf) const;

  std::string map_frame_;
  std::string camera_frame_;
  std::string odom_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  double planner_period_sec_{0.5};
  double waypoint_reach_dist_{0.6};
  double max_linear_vel_{0.25};
  double max_angular_vel_{0.6};
  bool enable_cmd_vel_{true};

  std::unique_ptr<::autonomy::exploration::ExplorationServer> server_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
  std::shared_ptr<message_filters::Synchronizer<DepthSyncPolicy>> sync_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr progress_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  mutable std::mutex mutex_;
  bool has_odom_{false};
};

}  // namespace autonomy_exploration
