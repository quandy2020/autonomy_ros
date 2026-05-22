// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autonomy_ros/bridge/map_bridge.hpp"

#include "autonomy_ros/conversions/conversions.hpp"

namespace autonomy_ros::bridge
{

MapBridge::MapBridge(rclcpp::Node & node)
: node_(node)
{
  node_.declare_parameter<std::string>("autonomy.map_topic", map_topic_);
  node_.declare_parameter<bool>("autonomy.publish_map", publish_map_);
  map_topic_ = node_.get_parameter("autonomy.map_topic").as_string();
  publish_map_ = node_.get_parameter("autonomy.publish_map").as_bool();
}

void MapBridge::start(::autonomy::map::MapServer * map_server)
{
  map_server_ = map_server;

  if (publish_map_) {
    map_pub_ = node_.create_publisher<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).transient_local());
  }

  map_sub_ = node_.create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, rclcpp::QoS(1).transient_local(),
    std::bind(&MapBridge::onMap, this, std::placeholders::_1));

  reload_map_srv_ = node_.create_service<std_srvs::srv::Trigger>(
    "reload_map",
    std::bind(
      &MapBridge::handleReloadMap, this, std::placeholders::_1,
      std::placeholders::_2));

  if (map_server_) {
    if (const auto map = map_server_->GetStaticMapShared()) {
      publishFromCore(map);
      RCLCPP_INFO(
        node_.get_logger(), "[map_bridge] published cached map %ux%u",
        map->info.width, map->info.height);
    }
  }

  RCLCPP_INFO(
    node_.get_logger(),
    "[map_bridge] topic=%s publish=%s (costmap via core MapServer)",
    map_topic_.c_str(), publish_map_ ? "true" : "false");
}

void MapBridge::stop()
{
  reload_map_srv_.reset();
  map_sub_.reset();
  map_pub_.reset();
  map_server_ = nullptr;
}

void MapBridge::publishFromCore(
  const ::autonomy::commsgs::map_msgs::OccupancyGrid::SharedPtr & map)
{
  if (!publish_map_ || !map_pub_ || !map) {
    return;
  }
  auto ros_map = conversions::toRos(*map);
  ros_map.header.stamp = node_.now();
  map_pub_->publish(ros_map);
}

void MapBridge::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (!msg || !map_server_) {
    return;
  }
  const auto core_map = conversions::fromRos(*msg);
  if (!map_server_->SetStaticMap(core_map)) {
    RCLCPP_WARN(node_.get_logger(), "[map_bridge] rejected invalid /map");
    return;
  }
  map_server_->PublishMap();
  RCLCPP_INFO_THROTTLE(
    node_.get_logger(), *node_.get_clock(), 5000,
    "[map_bridge] external map %ux%u", msg->info.width, msg->info.height);
}

void MapBridge::handleReloadMap(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!map_server_) {
    response->success = false;
    response->message = "map_server not available";
    return;
  }
  if (!map_server_->ReloadMap()) {
    response->success = false;
    response->message = "ReloadMap failed (check map_file in lua config)";
    return;
  }
  map_server_->PublishMap();
  response->success = true;
  response->message = "map reloaded";
  RCLCPP_INFO(node_.get_logger(), "[map_bridge] map reloaded from file");
}

}  // namespace autonomy_ros::bridge
