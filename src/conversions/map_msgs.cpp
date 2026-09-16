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

#include "autonomy_ros/conversions/map_msgs.hpp"

#include <cmath>

#include <google/protobuf/repeated_field.h>

#include "autonomy_ros/conversions/detail.hpp"

namespace autonomy_ros
{

namespace
{

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

}  // namespace autonomy_ros
