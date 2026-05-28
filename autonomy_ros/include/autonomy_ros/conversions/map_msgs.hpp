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

#ifndef AUTONOMY_ROS__CONVERSIONS__MAP_MSGS_HPP_
#define AUTONOMY_ROS__CONVERSIONS__MAP_MSGS_HPP_

/// @file map_msgs.hpp
/// @brief Converts commsgs map types to/from ROS map-related messages.
///
/// Proto schema: autonomy/commsgs/proto/map_msgs.proto
///
/// @par ROS dependencies
/// nav_msgs (occupancy grids), map_msgs (OccupancyGridUpdate), octomap_msgs, grid_map_msgs.
///
/// @par Usage
/// Include this header (or conversions/conversions.hpp) in map bridges and
/// nodes that subscribe to /map or publish map updates:
/// - fromRos() after receiving a ROS map message.
/// - toRos() before calling rclcpp::Publisher::publish().
///
/// @par Example
/// @code
/// #include "autonomy_ros/conversions/map_msgs.hpp"
/// void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
///   auto grid = autonomy_ros::fromRos(*msg);
/// }
/// map_pub_->publish(autonomy_ros::toRos(core_grid));
/// @endcode

#include "autonomy/commsgs/map_msgs.hpp"
#include "nav_msgs/msg/grid_cells.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "grid_map_msgs/msg/grid_map_info.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/msg/octomap_with_pose.hpp"

namespace autonomy_ros
{

using GridCells = ::autonomy::commsgs::map_msgs::GridCells;
using MapMetaData = ::autonomy::commsgs::map_msgs::MapMetaData;
using OccupancyGrid = ::autonomy::commsgs::map_msgs::OccupancyGrid;
using OccupancyGridUpdate = ::autonomy::commsgs::map_msgs::OccupancyGridUpdate;
using Octomap = ::autonomy::commsgs::map_msgs::Octomap;
using OctomapWithPose = ::autonomy::commsgs::map_msgs::OctomapWithPose;
using GridMapInfo = ::autonomy::commsgs::map_msgs::GridMapInfo;
using GridMap = ::autonomy::commsgs::map_msgs::GridMap;

/**
 * @brief Bidirectional conversion between ROS nav_msgs::msg::GridCells and commsgs GridCells.
 *
 * @par fromRos
 * @param from Input ROS message (nav_msgs::msg::GridCells). Fields are copied without coordinate transforms.
 * @return commsgs GridCells for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs GridCells from planners, bridges, or drivers.
 * @return ROS nav_msgs::msg::GridCells ready for rclcpp publish() or subscribe() adapters.
 */
GridCells fromRos(const nav_msgs::msg::GridCells & from);
nav_msgs::msg::GridCells toRos(const GridCells & from);

/**
 * @brief Bidirectional conversion between ROS nav_msgs::msg::MapMetaData and commsgs MapMetaData.
 *
 * @par fromRos
 * @param from Input ROS message (nav_msgs::msg::MapMetaData). Fields are copied without coordinate transforms.
 * @return commsgs MapMetaData for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs MapMetaData from planners, bridges, or drivers.
 * @return ROS nav_msgs::msg::MapMetaData ready for rclcpp publish() or subscribe() adapters.
 */
MapMetaData fromRos(const nav_msgs::msg::MapMetaData & from);
nav_msgs::msg::MapMetaData toRos(const MapMetaData & from);

/**
 * @brief Bidirectional conversion between ROS nav_msgs::msg::OccupancyGrid and commsgs OccupancyGrid.
 *
 * @par fromRos
 * @param from Input ROS message (nav_msgs::msg::OccupancyGrid). Fields are copied without coordinate transforms.
 * @return commsgs OccupancyGrid for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs OccupancyGrid from planners, bridges, or drivers.
 * @return ROS nav_msgs::msg::OccupancyGrid ready for rclcpp publish() or subscribe() adapters.
 */
OccupancyGrid fromRos(const nav_msgs::msg::OccupancyGrid & from);
nav_msgs::msg::OccupancyGrid toRos(const OccupancyGrid & from);

/**
 * @brief Bidirectional conversion between ROS map_msgs::msg::OccupancyGridUpdate and commsgs OccupancyGridUpdate.
 *
 * @par fromRos
 * @param from Input ROS message (map_msgs::msg::OccupancyGridUpdate). Fields are copied without coordinate transforms.
 * @return commsgs OccupancyGridUpdate for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs OccupancyGridUpdate from planners, bridges, or drivers.
 * @return ROS map_msgs::msg::OccupancyGridUpdate ready for rclcpp publish() or subscribe() adapters.
 */
OccupancyGridUpdate fromRos(const map_msgs::msg::OccupancyGridUpdate & from);
map_msgs::msg::OccupancyGridUpdate toRos(const OccupancyGridUpdate & from);

/**
 * @brief Bidirectional conversion between ROS octomap_msgs::msg::Octomap and commsgs Octomap.
 *
 * @par fromRos
 * @param from Input ROS message (octomap_msgs::msg::Octomap). Fields are copied without coordinate transforms.
 * @return commsgs Octomap for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Octomap from planners, bridges, or drivers.
 * @return ROS octomap_msgs::msg::Octomap ready for rclcpp publish() or subscribe() adapters.
 */
Octomap fromRos(const octomap_msgs::msg::Octomap & from);
octomap_msgs::msg::Octomap toRos(const Octomap & from);

/**
 * @brief Bidirectional conversion between ROS octomap_msgs::msg::OctomapWithPose and commsgs OctomapWithPose.
 *
 * @par fromRos
 * @param from Input ROS message (octomap_msgs::msg::OctomapWithPose). Fields are copied without coordinate transforms.
 * @return commsgs OctomapWithPose for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs OctomapWithPose from planners, bridges, or drivers.
 * @return ROS octomap_msgs::msg::OctomapWithPose ready for rclcpp publish() or subscribe() adapters.
 */
OctomapWithPose fromRos(const octomap_msgs::msg::OctomapWithPose & from);
octomap_msgs::msg::OctomapWithPose toRos(const OctomapWithPose & from);

/**
 * @brief Bidirectional conversion between ROS grid_map_msgs::msg::GridMapInfo and commsgs GridMapInfo.
 *
 * @par fromRos
 * @param from Input ROS message (grid_map_msgs::msg::GridMapInfo). Fields are copied without coordinate transforms.
 * @return commsgs GridMapInfo for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs GridMapInfo from planners, bridges, or drivers.
 * @return ROS grid_map_msgs::msg::GridMapInfo ready for rclcpp publish() or subscribe() adapters.
 */
GridMapInfo fromRos(const grid_map_msgs::msg::GridMapInfo & from);
grid_map_msgs::msg::GridMapInfo toRos(const GridMapInfo & from);

/**
 * @brief Bidirectional conversion between ROS grid_map_msgs::msg::GridMap and commsgs GridMap.
 *
 * @par fromRos
 * @param from Input ROS message (grid_map_msgs::msg::GridMap). Fields are copied without coordinate transforms.
 * @return commsgs GridMap for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs GridMap from planners, bridges, or drivers.
 * @return ROS grid_map_msgs::msg::GridMap ready for rclcpp publish() or subscribe() adapters.
 */
GridMap fromRos(const grid_map_msgs::msg::GridMap & from);
grid_map_msgs::msg::GridMap toRos(const GridMap & from);

}  // namespace autonomy_ros

#endif  // AUTONOMY_ROS__CONVERSIONS__MAP_MSGS_HPP_
