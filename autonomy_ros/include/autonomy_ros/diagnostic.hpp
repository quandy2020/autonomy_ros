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

#ifndef AUTONOMY_ROS__DIAGNOSTIC_HPP_
#define AUTONOMY_ROS__DIAGNOSTIC_HPP_

#include <functional>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy_ros
{

struct SystemHealthSnapshot
{
  bool core_running{false};
  bool odometry_received{false};
  bool static_map_loaded{false};
  bool controller_enabled{false};
};

/** @brief Periodic publisher for the ROS diagnostics topic. */
class DiagnosticPublisher
{
public:
  using SnapshotProvider = std::function<SystemHealthSnapshot()>;

  DiagnosticPublisher(
    rclcpp::Node & node,
    SnapshotProvider snapshot_provider,
    int publish_period_seconds = 1);

private:
  void Publish();

  rclcpp::Node & node_;
  SnapshotProvider snapshot_provider_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autonomy_ros

#endif  // AUTONOMY_ROS__DIAGNOSTIC_HPP_
