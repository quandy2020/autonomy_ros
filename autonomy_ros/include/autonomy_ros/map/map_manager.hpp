// Copyright 2025 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTONOMY_ROS__MAP__MAP_MANAGER_HPP_
#define AUTONOMY_ROS__MAP__MAP_MANAGER_HPP_

#include <mutex>
#include <optional>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy_ros::map
{

/**
 * @class autonomy_ros::map::MapManager
 * @brief Caches the latest occupancy grid for future cost-aware planning
 *
 * Subscribes to /map with transient-local QoS (compatible with map_server).
 * The current Planner uses straight-line paths in odom and does not query
 * the map yet; MapManager keeps the interface ready for collision checks.
 */
class MapManager
{
public:
  /**
   * @brief Constructor for autonomy_ros::map::MapManager
   * @param node Parent node used to create subscriptions
   */
  explicit MapManager(rclcpp::Node & node);

  /**
   * @brief Create subscription to nav_msgs/OccupancyGrid on topic map
   */
  void start();

  /**
   * @brief Thread-safe access to the last received map
   * @return Shared pointer to grid, or nullopt if no map received yet
   */
  std::optional<nav_msgs::msg::OccupancyGrid::SharedPtr> map() const;

private:
  /**
   * @brief Store incoming map and log dimensions (throttled)
   * @param msg Occupancy grid from SLAM or map_server
   */
  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  rclcpp::Node & node_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  mutable std::mutex mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
};

}  // namespace autonomy_ros::map

#endif  // AUTONOMY_ROS__MAP__MAP_MANAGER_HPP_
