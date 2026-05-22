// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autonomy_ros/conversions/map_msgs.hpp"

#include "autonomy_ros/conversions/detail.hpp"
#include "autonomy_ros/conversions/geometry_msgs.hpp"
#include "autonomy_ros/conversions/std_msgs.hpp"

namespace autonomy_ros::conversions
{

namespace
{

void copyOccupancyDataFromRos(
  const std::vector<int8_t> & from,
  std::vector<int16_t> & to)
{
  to.resize(from.size());
  for (size_t i = 0; i < from.size(); ++i) {
    to[i] = static_cast<int16_t>(from[i]);
  }
}

void copyOccupancyDataToRos(
  const std::vector<int16_t> & from,
  std::vector<int8_t> & to)
{
  to.resize(from.size());
  for (size_t i = 0; i < from.size(); ++i) {
    to[i] = static_cast<int8_t>(from[i]);
  }
}

}  // namespace

GridCells fromRos(const nav_msgs::msg::GridCells & from)
{
  GridCells to;
  detail::copyHeader(from.header, to.header);
  to.cell_width = static_cast<float>(from.cell_width);
  to.cell_height = static_cast<float>(from.cell_height);
  to.cells.reserve(from.cells.size());
  for (const auto & cell : from.cells) {
    ::autonomy::commsgs::geometry_msgs::Point point;
    detail::copyPoint(cell, point);
    to.cells.push_back(point);
  }
  return to;
}

nav_msgs::msg::GridCells toRos(const GridCells & from)
{
  nav_msgs::msg::GridCells to;
  detail::copyHeader(from.header, to.header);
  to.cell_width = from.cell_width;
  to.cell_height = from.cell_height;
  to.cells.reserve(from.cells.size());
  for (const auto & cell : from.cells) {
    geometry_msgs::msg::Point ros_point;
    detail::copyPoint(cell, ros_point);
    to.cells.push_back(ros_point);
  }
  return to;
}

MapMetaData fromRos(const nav_msgs::msg::MapMetaData & from)
{
  MapMetaData to;
  detail::copyTime(from.map_load_time, to.map_load_time);
  to.resolution = static_cast<float>(from.resolution);
  to.width = from.width;
  to.height = from.height;
  detail::copyPose(from.origin, to.origin);
  return to;
}

nav_msgs::msg::MapMetaData toRos(const MapMetaData & from)
{
  nav_msgs::msg::MapMetaData to;
  to.map_load_time = detail::toRosTime(from.map_load_time);
  to.resolution = from.resolution;
  to.width = from.width;
  to.height = from.height;
  detail::copyPose(from.origin, to.origin);
  return to;
}

OccupancyGrid fromRos(const nav_msgs::msg::OccupancyGrid & from)
{
  OccupancyGrid to;
  detail::copyHeader(from.header, to.header);
  to.info = fromRos(from.info);
  copyOccupancyDataFromRos(from.data, to.data);
  return to;
}

nav_msgs::msg::OccupancyGrid toRos(const OccupancyGrid & from)
{
  nav_msgs::msg::OccupancyGrid to;
  detail::copyHeader(from.header, to.header);
  to.info = toRos(from.info);
  copyOccupancyDataToRos(from.data, to.data);
  return to;
}

OccupancyGridUpdate fromRos(const map_msgs::msg::OccupancyGridUpdate & from)
{
  OccupancyGridUpdate to;
  detail::copyHeader(from.header, to.header);
  to.x = from.x;
  to.y = from.y;
  to.width = from.width;
  to.height = from.height;
  to.data.assign(from.data.begin(), from.data.end());
  return to;
}

map_msgs::msg::OccupancyGridUpdate toRos(const OccupancyGridUpdate & from)
{
  map_msgs::msg::OccupancyGridUpdate to;
  detail::copyHeader(from.header, to.header);
  to.x = from.x;
  to.y = from.y;
  to.width = from.width;
  to.height = from.height;
  to.data.assign(from.data.begin(), from.data.end());
  return to;
}

Octomap fromRos(const octomap_msgs::msg::Octomap & from)
{
  Octomap to;
  detail::copyHeader(from.header, to.header);
  to.binary = from.binary;
  to.id = from.id;
  to.resolution = from.resolution;
  to.data.assign(from.data.begin(), from.data.end());
  return to;
}

octomap_msgs::msg::Octomap toRos(const Octomap & from)
{
  octomap_msgs::msg::Octomap to;
  detail::copyHeader(from.header, to.header);
  to.binary = from.binary;
  to.id = from.id;
  to.resolution = from.resolution;
  to.data.reserve(from.data.size());
  for (const auto value : from.data) {
    to.data.push_back(static_cast<int8_t>(value));
  }
  return to;
}

OctomapWithPose fromRos(const octomap_msgs::msg::OctomapWithPose & from)
{
  OctomapWithPose to;
  detail::copyHeader(from.header, to.header);
  detail::copyPose(from.origin, to.origin);
  to.octomap = fromRos(from.octomap);
  return to;
}

octomap_msgs::msg::OctomapWithPose toRos(const OctomapWithPose & from)
{
  octomap_msgs::msg::OctomapWithPose to;
  detail::copyHeader(from.header, to.header);
  detail::copyPose(from.origin, to.origin);
  to.octomap = toRos(from.octomap);
  return to;
}

GridMapInfo fromRos(const grid_map_msgs::msg::GridMapInfo & from)
{
  GridMapInfo to;
  to.resolution = static_cast<float>(from.resolution);
  to.length_x = static_cast<float>(from.length_x);
  to.length_y = static_cast<float>(from.length_y);
  detail::copyPose(from.pose, to.pose);
  return to;
}

grid_map_msgs::msg::GridMapInfo toRos(const GridMapInfo & from)
{
  grid_map_msgs::msg::GridMapInfo to;
  to.resolution = from.resolution;
  to.length_x = from.length_x;
  to.length_y = from.length_y;
  detail::copyPose(from.pose, to.pose);
  return to;
}

GridMap fromRos(const grid_map_msgs::msg::GridMap & from)
{
  GridMap to;
  to.info = fromRos(from.info);
  detail::copyHeader(from.header, to.info.header);
  to.layers = from.layers;
  to.basic_layers = from.basic_layers;
  to.data.reserve(from.data.size());
  for (const auto & layer : from.data) {
    to.data.push_back(fromRos(layer));
  }
  to.outer_start_index = from.outer_start_index;
  to.inner_start_index = from.inner_start_index;
  return to;
}

grid_map_msgs::msg::GridMap toRos(const GridMap & from)
{
  grid_map_msgs::msg::GridMap to;
  detail::copyHeader(from.info.header, to.header);
  to.info = toRos(from.info);
  to.layers = from.layers;
  to.basic_layers = from.basic_layers;
  to.data.reserve(from.data.size());
  for (const auto & layer : from.data) {
    to.data.push_back(toRos(layer));
  }
  to.outer_start_index = static_cast<uint16_t>(from.outer_start_index);
  to.inner_start_index = static_cast<uint16_t>(from.inner_start_index);
  return to;
}

}  // namespace autonomy_ros::conversions
