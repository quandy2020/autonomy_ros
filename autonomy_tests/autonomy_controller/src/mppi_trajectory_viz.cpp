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
 * @brief Implements MPPI trajectory RViz visualization.
 */

#include "autonomy_controller/mppi_trajectory_viz.hpp"

#include <algorithm>
#include <cmath>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace autonomy_controller
{

void MppiTrajectoryVisualizer::Configure(
  rclcpp::Node * node,
  const std::string & frame_id,
  int max_candidates,
  double line_width)
{
  if (!node) {
    return;
  }
  frame_id_ = frame_id;
  max_candidates_ = std::max(1, max_candidates);
  line_width_ = line_width;

  const auto qos = rclcpp::QoS(10);
  candidates_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "controller_sim/mppi_candidates", qos);
  optimal_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "controller_sim/mppi_optimal_path", qos);
}

void MppiTrajectoryVisualizer::Publish(
  autonomy::control::controller::mppi_controller::MPPIController * mppi,
  const rclcpp::Time & stamp)
{
  if (!mppi || !candidates_pub_ || !optimal_path_pub_) {
    return;
  }
  if (candidates_pub_->get_subscription_count() == 0 &&
    optimal_path_pub_->get_subscription_count() == 0)
  {
    return;
  }

  const auto & opts = mppi->GetMppiOptions();
  size_t time_step = 4;
  if (opts.has_trajectory_visualizer() &&
    opts.trajectory_visualizer().time_step() > 0)
  {
    time_step = static_cast<size_t>(opts.trajectory_visualizer().time_step());
  }

  const auto & candidates = mppi->GetGeneratedTrajectories();
  visualization_msgs::msg::MarkerArray markers;
  const size_t n_rows = static_cast<size_t>(candidates.x.rows());
  const size_t n_cols = static_cast<size_t>(candidates.x.cols());
  if (n_rows == 0 || n_cols == 0) {
    return;
  }

  const size_t max_lines = static_cast<size_t>(max_candidates_);
  const size_t row_step = std::max(size_t(1), (n_rows + max_lines - 1) / max_lines);
  markers.markers.reserve(max_lines + 2);

  int marker_id = 0;
  for (size_t i = 0; i < n_rows && marker_id < static_cast<int>(max_lines); i += row_step) {
    visualization_msgs::msg::Marker line;
    line.header.frame_id = frame_id_;
    line.header.stamp = stamp;
    line.ns = "mppi_candidates";
    line.id = marker_id++;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = line_width_;
    const float t = n_rows > 1 ?
      static_cast<float>(i) / static_cast<float>(n_rows - 1) : 0.0f;
    line.color.r = 0.1f;
    line.color.g = 0.55f + 0.35f * t;
    line.color.b = 0.9f - 0.4f * t;
    line.color.a = 0.55f;

    line.points.reserve((n_cols + time_step - 1) / time_step);
    for (size_t j = 0; j < n_cols; j += time_step) {
      const float x =
        candidates.x(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j));
      const float y =
        candidates.y(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j));
      if (!std::isfinite(x) || !std::isfinite(y)) {
        continue;
      }
      geometry_msgs::msg::Point p;
      p.x = static_cast<double>(x);
      p.y = static_cast<double>(y);
      p.z = 0.04;
      line.points.push_back(p);
    }
    if (line.points.size() >= 2) {
      markers.markers.push_back(std::move(line));
    } else {
      --marker_id;
    }
  }

  for (int id = marker_id; id < published_candidates_; ++id) {
    visualization_msgs::msg::Marker del;
    del.header.frame_id = frame_id_;
    del.header.stamp = stamp;
    del.ns = "mppi_candidates";
    del.id = id;
    del.action = visualization_msgs::msg::Marker::DELETE;
    markers.markers.push_back(std::move(del));
  }
  published_candidates_ = marker_id;

  if (optimal_path_pub_->get_subscription_count() > 0) {
    const auto optimal = mppi->GetOptimizedTrajectory();
    nav_msgs::msg::Path optimal_path;
    optimal_path.header.frame_id = frame_id_;
    optimal_path.header.stamp = stamp;

    visualization_msgs::msg::Marker opt_line;
    opt_line.header = optimal_path.header;
    opt_line.ns = "mppi_optimal";
    opt_line.id = 0;
    opt_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    opt_line.action = visualization_msgs::msg::Marker::ADD;
    opt_line.pose.orientation.w = 1.0;
    opt_line.scale.x = std::max(0.02, line_width_ * 2.5);
    opt_line.color.r = 1.0f;
    opt_line.color.g = 0.45f;
    opt_line.color.b = 0.05f;
    opt_line.color.a = 0.95f;

    const size_t opt_rows = static_cast<size_t>(optimal.rows());
    opt_line.points.reserve(opt_rows);
    optimal_path.poses.reserve(opt_rows);
    for (size_t i = 0; i < opt_rows; ++i) {
      const float x = optimal(static_cast<Eigen::Index>(i), 0);
      const float y = optimal(static_cast<Eigen::Index>(i), 1);
      if (!std::isfinite(x) || !std::isfinite(y)) {
        continue;
      }
      geometry_msgs::msg::Point p;
      p.x = static_cast<double>(x);
      p.y = static_cast<double>(y);
      p.z = 0.06;
      opt_line.points.push_back(p);

      geometry_msgs::msg::PoseStamped ps;
      ps.header = optimal_path.header;
      ps.pose.position = p;
      const double yaw = static_cast<double>(
        optimal(static_cast<Eigen::Index>(i), 2));
      ps.pose.orientation.z = std::sin(yaw * 0.5);
      ps.pose.orientation.w = std::cos(yaw * 0.5);
      optimal_path.poses.push_back(ps);
    }
    if (opt_line.points.size() >= 2) {
      markers.markers.push_back(std::move(opt_line));
    }
    optimal_path_pub_->publish(optimal_path);
  }

  if (!markers.markers.empty()) {
    candidates_pub_->publish(markers);
  }
}

}  // namespace autonomy_controller
