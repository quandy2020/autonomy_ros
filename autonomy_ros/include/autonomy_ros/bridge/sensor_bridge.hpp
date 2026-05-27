// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTONOMY_ROS__BRIDGE__SENSOR_BRIDGE_HPP_
#define AUTONOMY_ROS__BRIDGE__SENSOR_BRIDGE_HPP_

#include <functional>
#include <memory>
#include <string>

#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy_ros/system/options.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace autonomy_ros::bridge
{

/**
 * @brief Callbacks from ROS sensor topics into autonomy core (or listeners).
 *
 * Only handlers that are set cause subscriptions to be created.
 */
struct SensorBridgeHandlers
{
  std::function<void(const nav_msgs::msg::Odometry::SharedPtr &)> on_odom;
  std::function<void(const ::autonomy::commsgs::sensor_msgs::LaserScan &)> on_laser_scan;
  std::function<void(const ::autonomy::commsgs::sensor_msgs::PointCloud2 &)> on_point_cloud;
  std::function<void(const ::autonomy::commsgs::sensor_msgs::Range &)> on_range;
};

/**
 * @brief ROS sensor ingress: odometry, laser scan, optional point cloud and range.
 */
class SensorBridge
{
public:
  SensorBridge(
    rclcpp::Node & node,
    const system::AutonomyRosOptions & ros_options,
    SensorBridgeHandlers handlers);
  ~SensorBridge();

private:
  void setupSubscriptions();

  rclcpp::Node & node_;
  system::AutonomyRosOptions ros_options_;
  SensorBridgeHandlers handlers_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub_;
};

}  // namespace autonomy_ros::bridge

#endif  // AUTONOMY_ROS__BRIDGE__SENSOR_BRIDGE_HPP_
