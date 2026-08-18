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

#include <google/protobuf/repeated_field.h>

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
    return kOccOccupied;
  }
  if (raw <= 1) {
    return kOccFree;
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
    return static_cast<int8_t>(kOccOccupied);
  }
  if (core_cell <= 1) {
    return static_cast<int8_t>(kOccFree);
  }
  if (core_cell == 205) {
    return static_cast<int8_t>(kOccUnknown);
  }
  return static_cast<int8_t>(std::round(
    (static_cast<double>(core_cell) / 255.0) * static_cast<double>(kOccOccupied)));
}

void copyOccupancyDataFromRos(
  const std::vector<int8_t> & from,
  google::protobuf::RepeatedField<int32_t> * to)
{
  to->Clear();
  to->Reserve(static_cast<int>(from.size()));
  for (const auto cell : from) {
    to->Add(static_cast<int32_t>(occupancyCellFromRos(cell)));
  }
}

void copyOccupancyDataToRos(
  const google::protobuf::RepeatedField<int32_t> & from,
  std::vector<int8_t> & to)
{
  to.resize(static_cast<size_t>(from.size()));
  for (int i = 0; i < from.size(); ++i) {
    to[static_cast<size_t>(i)] = occupancyCellToRos(static_cast<int16_t>(from.Get(i)));
  }
}

}  // namespace

GridCells fromRos(const nav_msgs::msg::GridCells & from)
{
  GridCells to;
  copyHeader(from.header, *to.mutable_header());
  to.set_cell_width(static_cast<float>(from.cell_width));
  to.set_cell_height(static_cast<float>(from.cell_height));
  for (const auto & cell : from.cells) {
    copyPoint(cell, *to.add_cells());
  }
  return to;
}

nav_msgs::msg::GridCells toRos(const GridCells & from)
{
  nav_msgs::msg::GridCells to;
  copyHeader(from.header(), to.header);
  to.cell_width = from.cell_width();
  to.cell_height = from.cell_height();
  to.cells.reserve(static_cast<size_t>(from.cells_size()));
  for (const auto & cell : from.cells()) {
    geometry_msgs::msg::Point ros_point;
    copyPoint(cell, ros_point);
    to.cells.push_back(ros_point);
  }
  return to;
}

MapMetaData fromRos(const nav_msgs::msg::MapMetaData & from)
{
  MapMetaData to;
  copyTime(from.map_load_time, *to.mutable_map_load_time());
  to.set_resolution(static_cast<float>(from.resolution));
  to.set_width(from.width);
  to.set_height(from.height);
  copyPose(from.origin, *to.mutable_origin());
  return to;
}

nav_msgs::msg::MapMetaData toRos(const MapMetaData & from)
{
  nav_msgs::msg::MapMetaData to;
  to.map_load_time = toRosTime(from.map_load_time());
  to.resolution = from.resolution();
  to.width = from.width();
  to.height = from.height();
  copyPose(from.origin(), to.origin);
  return to;
}

OccupancyGrid fromRos(const nav_msgs::msg::OccupancyGrid & from)
{
  OccupancyGrid to;
  copyHeader(from.header, *to.mutable_header());
  *to.mutable_info() = fromRos(from.info);
  copyOccupancyDataFromRos(from.data, to.mutable_data());
  return to;
}

nav_msgs::msg::OccupancyGrid toRos(const OccupancyGrid & from)
{
  nav_msgs::msg::OccupancyGrid to;
  copyHeader(from.header(), to.header);
  to.info = toRos(from.info());
  copyOccupancyDataToRos(from.data(), to.data);
  return to;
}

OccupancyGridUpdate fromRos(const map_msgs::msg::OccupancyGridUpdate & from)
{
  OccupancyGridUpdate to;
  copyHeader(from.header, *to.mutable_header());
  to.set_x(from.x);
  to.set_y(from.y);
  to.set_width(from.width);
  to.set_height(from.height);
  to.mutable_data()->Reserve(static_cast<int>(from.data.size()));
  for (const auto cell : from.data) {
    to.add_data(static_cast<int32_t>(cell));
  }
  return to;
}

map_msgs::msg::OccupancyGridUpdate toRos(const OccupancyGridUpdate & from)
{
  map_msgs::msg::OccupancyGridUpdate to;
  copyHeader(from.header(), to.header);
  to.x = from.x();
  to.y = from.y();
  to.width = from.width();
  to.height = from.height();
  to.data.resize(static_cast<size_t>(from.data().size()));
  for (int i = 0; i < from.data().size(); ++i) {
    to.data[static_cast<size_t>(i)] = static_cast<int8_t>(from.data().Get(i));
  }
  return to;
}

Octomap fromRos(const octomap_msgs::msg::Octomap & from)
{
  Octomap to;
  copyHeader(from.header, *to.mutable_header());
  to.set_binary(from.binary);
  to.set_id(from.id);
  to.set_resolution(from.resolution);
  to.mutable_data()->Reserve(static_cast<int>(from.data.size()));
  for (const auto byte : from.data) {
    to.add_data(static_cast<int32_t>(byte));
  }
  return to;
}

octomap_msgs::msg::Octomap toRos(const Octomap & from)
{
  octomap_msgs::msg::Octomap to;
  copyHeader(from.header(), to.header);
  to.binary = from.binary();
  to.id = from.id();
  to.resolution = from.resolution();
  to.data.reserve(static_cast<size_t>(from.data_size()));
  for (const auto value : from.data()) {
    to.data.push_back(static_cast<int8_t>(value));
  }
  return to;
}

OctomapWithPose fromRos(const octomap_msgs::msg::OctomapWithPose & from)
{
  OctomapWithPose to;
  copyHeader(from.header, *to.mutable_header());
  copyPose(from.origin, *to.mutable_origin());
  *to.mutable_octomap() = fromRos(from.octomap);
  return to;
}

octomap_msgs::msg::OctomapWithPose toRos(const OctomapWithPose & from)
{
  octomap_msgs::msg::OctomapWithPose to;
  copyHeader(from.header(), to.header);
  copyPose(from.origin(), to.origin);
  to.octomap = toRos(from.octomap());
  return to;
}

GridMapInfo fromRos(const grid_map_msgs::msg::GridMapInfo & from)
{
  GridMapInfo to;
  to.set_resolution(static_cast<float>(from.resolution));
  to.set_length_x(static_cast<float>(from.length_x));
  to.set_length_y(static_cast<float>(from.length_y));
  copyPose(from.pose, *to.mutable_pose());
  return to;
}

grid_map_msgs::msg::GridMapInfo toRos(const GridMapInfo & from)
{
  grid_map_msgs::msg::GridMapInfo to;
  to.resolution = from.resolution();
  to.length_x = from.length_x();
  to.length_y = from.length_y();
  copyPose(from.pose(), to.pose);
  return to;
}

GridMap fromRos(const grid_map_msgs::msg::GridMap & from)
{
  GridMap to;
  *to.mutable_info() = fromRos(from.info);
  copyHeader(from.header, *to.mutable_info()->mutable_header());
  to.mutable_layers()->Assign(from.layers.begin(), from.layers.end());
  to.mutable_basic_layers()->Assign(from.basic_layers.begin(), from.basic_layers.end());
  for (const auto & layer : from.data) {
    *to.add_data() = fromRos(layer);
  }
  to.set_outer_start_index(from.outer_start_index);
  to.set_inner_start_index(from.inner_start_index);
  return to;
}

grid_map_msgs::msg::GridMap toRos(const GridMap & from)
{
  grid_map_msgs::msg::GridMap to;
  copyHeader(from.info().header(), to.header);
  to.info = toRos(from.info());
  to.layers.assign(from.layers().begin(), from.layers().end());
  to.basic_layers.assign(from.basic_layers().begin(), from.basic_layers().end());
  to.data.reserve(static_cast<size_t>(from.data_size()));
  for (const auto & layer : from.data()) {
    to.data.push_back(toRos(layer));
  }
  to.outer_start_index = static_cast<uint16_t>(from.outer_start_index());
  to.inner_start_index = static_cast<uint16_t>(from.inner_start_index());
  return to;
}

}  // namespace autonomy_ros
