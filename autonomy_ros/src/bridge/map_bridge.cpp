// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autonomy_ros/bridge/map_bridge.hpp"

#include <functional>

#include "autonomy_ros/conversions/conversions.hpp"

namespace autonomy_ros::bridge
{

MapBridge::MapBridge(
  rclcpp::Node & node, ::autonomy::map::MapServer * map_server,
  const AutonomyRosOptions & ros_options)
: node_(node),
  map_server_(map_server),
  publish_map_(ros_options.publish_map),
  map_topic_(ros_options.map_topic)
{
  if (publish_map_) {
    map_pub_ = node_.create_publisher<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).transient_local());
  }

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

MapBridge::~MapBridge()
{
  reload_map_srv_.reset();
  map_pub_.reset();
  map_server_ = nullptr;
}

void MapBridge::publishFromCore(
  const ::autonomy::commsgs::map_msgs::OccupancyGrid::SharedPtr & map)
{
  if (!publish_map_ || !map_pub_ || !map) {
    return;
  }
  const auto expected =
    static_cast<size_t>(map->info.width) * static_cast<size_t>(map->info.height);
  if (map->data.size() != expected) {
    RCLCPP_WARN(
      node_.get_logger(),
      "[map_bridge] skip publish: data size %zu != %u*%u",
      map->data.size(), map->info.width, map->info.height);
    return;
  }

  size_t occupied = 0;
  size_t unknown = 0;
  for (int16_t cell : map->data) {
    if (cell == 100) {
      ++occupied;
    } else if (cell < 0) {
      ++unknown;
    }
  }
  if (occupied == 0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "[map_bridge] map has no occupied cells (unknown=%zu); check map_file / "
      "occupancy conversion",
      unknown);
  }

  auto ros_map = conversions::toRos(*map);
  ros_map.header.stamp = node_.now();
  map_pub_->publish(ros_map);
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
