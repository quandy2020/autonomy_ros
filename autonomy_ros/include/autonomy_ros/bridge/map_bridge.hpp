// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTONOMY_ROS__BRIDGE__MAP_BRIDGE_HPP_
#define AUTONOMY_ROS__BRIDGE__MAP_BRIDGE_HPP_

#include <memory>
#include <string>

#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/map/map_server.hpp"
#include "autonomy_ros/constants.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace autonomy_ros::bridge
{

/**
 * @brief Map ROS topic: publish core map; external /map updates via MapServer::PublishMap.
 *
 * Costmap sync is handled inside autonomy core (MapServer callback). This bridge only
 * republishes to ROS and accepts external map messages.
 */
class MapBridge
{
public:
  explicit MapBridge(rclcpp::Node & node);

  void start(::autonomy::map::MapServer * map_server);
  void stop();

  /** @brief Publish a core map snapshot to ROS (no costmap side effects). */
  void publishFromCore(const ::autonomy::commsgs::map_msgs::OccupancyGrid::SharedPtr & map);

private:
  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void handleReloadMap(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp::Node & node_;
  ::autonomy::map::MapServer * map_server_{nullptr};
  bool publish_map_{true};
  std::string map_topic_{constants::topics::kMap};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_map_srv_;
};

}  // namespace autonomy_ros::bridge

#endif  // AUTONOMY_ROS__BRIDGE__MAP_BRIDGE_HPP_
