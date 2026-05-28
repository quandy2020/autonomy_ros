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

// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autonomy_ros/conversions/map_msgs.hpp"

#include <cmath>

#include "autonomy_ros/conversions/detail.hpp"
#include "autonomy_ros/conversions/geometry_msgs.hpp"
#include "autonomy_ros/conversions/std_msgs.hpp"

namespace autonomy_ros
{

namespace
{

// nav_msgs/OccupancyGrid (ROS): -1 unknown, 0 free, 1..100 occupied probability.
// Some SLAM stacks / PGM caches use 0..255 (0 free, 255 occupied, 205 unknown).
constexpr int16_t kOccUnknown = -1;
constexpr int16_t kOccFree = 0;
constexpr int16_t kOccOccupied = 100;

int16_t occupancyCellFromRos(int8_t ros_cell)
{
  if (ros_cell < 0) {
    return kOccUnknown;
  }
  if (ros_cell <= kOccOccupied) {
    return static_cast<int16_t>(ros_cell);
  }
  // Legacy 0..255 in a signed byte (e.g. -51 for 205): remap by unsigned view.
  const uint8_t raw = static_cast<uint8_t>(ros_cell);
  if (raw >= 254) {
    return kOccFree;
  }
  if (raw <= 1) {
    return kOccOccupied;
  }
  if (raw == 205) {
    return kOccUnknown;
  }
  return static_cast<int16_t>(
    std::round((static_cast<double>(raw) / 255.0) * static_cast<double>(kOccOccupied)));
}

int8_t occupancyCellToRos(int16_t core_cell)
{
  if (core_cell < 0) {
    return static_cast<int8_t>(kOccUnknown);
  }
  if (core_cell <= kOccOccupied) {
    return static_cast<int8_t>(core_cell);
  }
  // Core stored legacy 0..255 (e.g. raw PGM cache in int16).
  if (core_cell >= 254) {
    return static_cast<int8_t>(kOccFree);
  }
  if (core_cell <= 1) {
    return static_cast<int8_t>(kOccOccupied);
  }
  if (core_cell == 205) {
    return static_cast<int8_t>(kOccUnknown);
  }
  return static_cast<int8_t>(std::round(
    (static_cast<double>(core_cell) / 255.0) * static_cast<double>(kOccOccupied)));
}

void copyOccupancyDataFromRos(
  const std::vector<int8_t> & from,
  std::vector<int16_t> & to)
{
  to.resize(from.size());
  for (size_t i = 0; i < from.size(); ++i) {
    to[i] = occupancyCellFromRos(from[i]);
  }
}

void copyOccupancyDataToRos(
  const std::vector<int16_t> & from,
  std::vector<int8_t> & to)
{
  to.resize(from.size());
  for (size_t i = 0; i < from.size(); ++i) {
    to[i] = occupancyCellToRos(from[i]);
  }
}

}  // namespace

GridCells fromRos(const nav_msgs::msg::GridCells & from)
{
  GridCells to;
  copyHeader(from.header, to.header);
  to.cell_width = static_cast<float>(from.cell_width);
  to.cell_height = static_cast<float>(from.cell_height);
  to.cells.reserve(from.cells.size());
  for (const auto & cell : from.cells) {
    ::autonomy::commsgs::geometry_msgs::Point point;
    copyPoint(cell, point);
    to.cells.push_back(point);
  }
  return to;
}

nav_msgs::msg::GridCells toRos(const GridCells & from)
{
  nav_msgs::msg::GridCells to;
  copyHeader(from.header, to.header);
  to.cell_width = from.cell_width;
  to.cell_height = from.cell_height;
  to.cells.reserve(from.cells.size());
  for (const auto & cell : from.cells) {
    geometry_msgs::msg::Point ros_point;
    copyPoint(cell, ros_point);
    to.cells.push_back(ros_point);
  }
  return to;
}

MapMetaData fromRos(const nav_msgs::msg::MapMetaData & from)
{
  MapMetaData to;
  copyTime(from.map_load_time, to.map_load_time);
  to.resolution = static_cast<float>(from.resolution);
  to.width = from.width;
  to.height = from.height;
  copyPose(from.origin, to.origin);
  return to;
}

nav_msgs::msg::MapMetaData toRos(const MapMetaData & from)
{
  nav_msgs::msg::MapMetaData to;
  to.map_load_time = toRosTime(from.map_load_time);
  to.resolution = from.resolution;
  to.width = from.width;
  to.height = from.height;
  copyPose(from.origin, to.origin);
  return to;
}

OccupancyGrid fromRos(const nav_msgs::msg::OccupancyGrid & from)
{
  OccupancyGrid to;
  copyHeader(from.header, to.header);
  to.info = fromRos(from.info);
  copyOccupancyDataFromRos(from.data, to.data);
  return to;
}

nav_msgs::msg::OccupancyGrid toRos(const OccupancyGrid & from)
{
  nav_msgs::msg::OccupancyGrid to;
  copyHeader(from.header, to.header);
  to.info = toRos(from.info);
  copyOccupancyDataToRos(from.data, to.data);
  return to;
}

OccupancyGridUpdate fromRos(const map_msgs::msg::OccupancyGridUpdate & from)
{
  OccupancyGridUpdate to;
  copyHeader(from.header, to.header);
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
  copyHeader(from.header, to.header);
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
  copyHeader(from.header, to.header);
  to.binary = from.binary;
  to.id = from.id;
  to.resolution = from.resolution;
  to.data.assign(from.data.begin(), from.data.end());
  return to;
}

octomap_msgs::msg::Octomap toRos(const Octomap & from)
{
  octomap_msgs::msg::Octomap to;
  copyHeader(from.header, to.header);
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
  copyHeader(from.header, to.header);
  copyPose(from.origin, to.origin);
  to.octomap = fromRos(from.octomap);
  return to;
}

octomap_msgs::msg::OctomapWithPose toRos(const OctomapWithPose & from)
{
  octomap_msgs::msg::OctomapWithPose to;
  copyHeader(from.header, to.header);
  copyPose(from.origin, to.origin);
  to.octomap = toRos(from.octomap);
  return to;
}

GridMapInfo fromRos(const grid_map_msgs::msg::GridMapInfo & from)
{
  GridMapInfo to;
  to.resolution = static_cast<float>(from.resolution);
  to.length_x = static_cast<float>(from.length_x);
  to.length_y = static_cast<float>(from.length_y);
  copyPose(from.pose, to.pose);
  return to;
}

grid_map_msgs::msg::GridMapInfo toRos(const GridMapInfo & from)
{
  grid_map_msgs::msg::GridMapInfo to;
  to.resolution = from.resolution;
  to.length_x = from.length_x;
  to.length_y = from.length_y;
  copyPose(from.pose, to.pose);
  return to;
}

GridMap fromRos(const grid_map_msgs::msg::GridMap & from)
{
  GridMap to;
  to.info = fromRos(from.info);
  copyHeader(from.header, to.info.header);
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
  copyHeader(from.info.header, to.header);
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

}  // namespace autonomy_ros
