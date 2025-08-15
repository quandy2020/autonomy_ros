/*
 * Copyright 2024 The OpenRobotic Beginner Authors
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

 #pragma once

 #include <string>
 #include <tuple>
 
 
 #include "autonomy/commsgs/map_msgs.hpp"
 
 // map_msgs
 #include "nav_msgs/msg/grid_cells.hpp"
 #include "nav_msgs/msg/map_meta_data.hpp"
 #include "nav_msgs/msg/occupancy_grid.hpp"
 
 namespace autonomy_ros {
 
// GridCells
nav_msgs::msg::GridCells ToRos(const ::autonomy::commsgs::map_msgs::GridCells& proto);
::autonomy::commsgs::map_msgs::GridCells FromRos(const nav_msgs::msg::GridCells& ros);

// MapMetaData
nav_msgs::msg::MapMetaData ToRos(const ::autonomy::commsgs::map_msgs::MapMetaData& proto);
::autonomy::commsgs::map_msgs::MapMetaData FromRos(const nav_msgs::msg::MapMetaData& ros);

// OccupancyGrid
nav_msgs::msg::OccupancyGrid ToRos(const ::autonomy::commsgs::map_msgs::OccupancyGrid& proto);
::autonomy::commsgs::map_msgs::OccupancyGrid FromRos(const nav_msgs::msg::OccupancyGrid& ros);
 
 
 }  // namespace autonomy_ros
 
