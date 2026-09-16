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

#ifndef AUTONOMY_ROS__CONVERSIONS__MAP_MSGS_HPP_
#define AUTONOMY_ROS__CONVERSIONS__MAP_MSGS_HPP_

#include <automsgs/msgs/map_msgs/map_meta_data.pb.h>
#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace autonomy_ros
{

using MapMetaData = ::automsgs::msgs::map_msgs::MapMetaData;
using OccupancyGrid = ::automsgs::msgs::map_msgs::OccupancyGrid;

MapMetaData fromRos(const nav_msgs::msg::MapMetaData & from);
nav_msgs::msg::MapMetaData toRos(const MapMetaData & from);

OccupancyGrid fromRos(const nav_msgs::msg::OccupancyGrid & from);
nav_msgs::msg::OccupancyGrid toRos(const OccupancyGrid & from);

}  // namespace autonomy_ros

#endif  // AUTONOMY_ROS__CONVERSIONS__MAP_MSGS_HPP_
