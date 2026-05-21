#include "autonomy_ros/map/map_manager.hpp"

namespace autonomy_ros::map
{

MapManager::MapManager(rclcpp::Node & node)
: node_(node)
{
}

void MapManager::start()
{
  map_sub_ = node_.create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(1).transient_local(),
    std::bind(&MapManager::onMap, this, std::placeholders::_1));
  RCLCPP_INFO(node_.get_logger(), "[map] subscribed to /map");
}

std::optional<nav_msgs::msg::OccupancyGrid::SharedPtr> MapManager::map() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!latest_map_) {
    return std::nullopt;
  }
  return latest_map_;
}

void MapManager::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_map_ = msg;
  RCLCPP_INFO_THROTTLE(
    node_.get_logger(), *node_.get_clock(), 5000,
    "[map] received map %ux%u resolution=%.3f", msg->info.width, msg->info.height,
    msg->info.resolution);
}

}  // namespace autonomy_ros::map
