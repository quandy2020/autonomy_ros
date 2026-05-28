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

#ifndef AUTONOMY_ROS__BRIDGE_HPP_
#define AUTONOMY_ROS__BRIDGE_HPP_

#include <functional>
#include <memory>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/map/map_server.hpp"
#include "autonomy_ros/options.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace autonomy_ros
{

/**
 * @brief ROS ingress/egress for TF, map, costmap, sensors, and cmd_vel.
 *
 * Construct once from @p options; wire callbacks for odometry and laser scan.
 */
class RosBridge
{
public:
  using OdomCallback = std::function<void(const nav_msgs::msg::Odometry::SharedPtr &)>;
  using ScanCallback = std::function<void(const sensor_msgs::msg::LaserScan::SharedPtr &)>;

  RosBridge(
    rclcpp::Node & node,
    ::autonomy::map::MapServer * map_server,
    ::autonomy::map::costmap_2d::Costmap2DWrapper * costmap_wrapper,
    const Options & options,
    OdomCallback on_odom = {},
    ScanCallback on_scan = {});

  void PublishMap(const ::autonomy::commsgs::map_msgs::OccupancyGrid::SharedPtr & map);
  void PublishCmdVel(const ::autonomy::commsgs::geometry_msgs::TwistStamped & cmd);
  void PublishCmdVelZero();

private:
  void OnTf(const tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static);
  void OnCostmapTimer();
  void OnReloadMap(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> & request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp::Node & node_;
  ::autonomy::map::MapServer * map_server_{nullptr};
  ::autonomy::map::costmap_2d::Costmap2DWrapper * costmap_wrapper_{nullptr};

  CoreOptions core_options_;
  RosOptions ros_options_;

  std::string base_frame_;
  double max_linear_vel_{0.0};
  bool publish_map_{true};

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_map_srv_;
  rclcpp::TimerBase::SharedPtr costmap_timer_;
};

}  // namespace autonomy_ros

#endif  // AUTONOMY_ROS__BRIDGE_HPP_
