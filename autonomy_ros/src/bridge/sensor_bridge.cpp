// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autonomy_ros/bridge/sensor_bridge.hpp"

#include "autonomy_ros/constants.hpp"
#include "autonomy_ros/conversions/conversions.hpp"

namespace autonomy_ros::bridge
{

namespace
{

/** @brief ROS message passed through to @p handler (e.g. odometry for listeners). */
template<typename RosMsg, typename QoST, typename Handler>
void subscribeRos(
  rclcpp::Node & node, typename rclcpp::Subscription<RosMsg>::SharedPtr & sub,
  const std::string & topic, QoST qos, Handler handler, const char * log_fmt)
{
  if (!handler) {
    return;
  }
  sub = node.create_subscription<RosMsg>(
    topic, qos,
    [handler = std::move(handler)](const typename RosMsg::SharedPtr msg) {
      if (msg) {
        handler(msg);
      }
    });
  RCLCPP_INFO(node.get_logger(), log_fmt, topic.c_str());
}

/** @brief ROS message converted with @p from_ros before invoking @p handler. */
template<typename RosMsg, typename Commsg, typename QoST, typename Handler>
void subscribeCommsg(
  rclcpp::Node & node, typename rclcpp::Subscription<RosMsg>::SharedPtr & sub,
  const std::string & topic, QoST qos, Handler handler,
  Commsg (*from_ros)(const RosMsg &), const char * log_fmt)
{
  if (!handler) {
    return;
  }
  sub = node.create_subscription<RosMsg>(
    topic, qos,
    [handler = std::move(handler), from_ros](const typename RosMsg::SharedPtr msg) {
      if (msg) {
        handler(from_ros(*msg));
      }
    });
  RCLCPP_INFO(node.get_logger(), log_fmt, topic.c_str());
}

}  // namespace

SensorBridge::SensorBridge(
  rclcpp::Node & node, const AutonomyRosOptions & ros_options,
  SensorBridgeHandlers handlers)
: node_(node), ros_options_(ros_options), handlers_(std::move(handlers))
{
  setupSubscriptions();
}

SensorBridge::~SensorBridge()
{
  odom_sub_.reset();
  laser_sub_.reset();
  point_cloud_sub_.reset();
  range_sub_.reset();
  image_sub_.reset();
  imu_sub_.reset();
}

void SensorBridge::setupSubscriptions()
{
  subscribeRos<nav_msgs::msg::Odometry>(
    node_, odom_sub_, ros_options_.odom_topic, constants::defaults::kQueueDepth,
    handlers_.on_odom, "[sensor_bridge] odom %s -> autonomy");

  if (ros_options_.scan_enabled) {
    subscribeCommsg<sensor_msgs::msg::LaserScan, conversions::LaserScan>(
      node_, laser_sub_, ros_options_.scan_topic, rclcpp::SensorDataQoS(),
      handlers_.on_laser_scan, conversions::fromRos,
      "[sensor_bridge] laser %s -> costmap");
  }

  const auto & sb = ros_options_.sensor_bridge;

  if (sb.point_cloud_enabled) {
    subscribeCommsg<sensor_msgs::msg::PointCloud2, conversions::PointCloud2>(
      node_, point_cloud_sub_, sb.point_cloud_topic, rclcpp::SensorDataQoS(),
      handlers_.on_point_cloud, conversions::fromRos,
      "[sensor_bridge] point_cloud %s -> costmap");
  }

  if (sb.range_enabled) {
    subscribeCommsg<sensor_msgs::msg::Range, conversions::Range>(
      node_, range_sub_, sb.range_topic, constants::defaults::kQueueDepth,
      handlers_.on_range, conversions::fromRos,
      "[sensor_bridge] range %s -> costmap");
  }

  if (sb.image_enabled) {
    subscribeCommsg<sensor_msgs::msg::Image, conversions::Image>(
      node_, image_sub_, sb.image_topic, rclcpp::SensorDataQoS(),
      handlers_.on_image, conversions::fromRos,
      "[sensor_bridge] image %s (handler registered)");
  }

  if (sb.imu_enabled) {
    subscribeCommsg<sensor_msgs::msg::Imu, conversions::Imu>(
      node_, imu_sub_, sb.imu_topic, rclcpp::SensorDataQoS(),
      handlers_.on_imu, conversions::fromRos,
      "[sensor_bridge] imu %s (handler registered)");
  }
}

}  // namespace autonomy_ros::bridge
