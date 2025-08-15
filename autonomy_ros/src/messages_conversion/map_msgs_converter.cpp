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

#include "autonomy_ros/messages_conversion/map_msgs_converter.hpp"
#include "autonomy_ros/messages_conversion/std_msgs_converter.hpp"
#include "autonomy_ros/messages_conversion/geometry_msgs_converter.hpp"
#include "autonomy_ros/messages_conversion/nav_msgs_converter.hpp"
#include "autonomy_ros/messages_conversion/builtin_interfaces_converter.hpp"


 namespace autonomy_ros {
 
// GridCells
nav_msgs::msg::GridCells ToRos(const ::autonomy::commsgs::map_msgs::GridCells& proto)
{
    nav_msgs::msg::GridCells data;
    return data;
}

::autonomy::commsgs::map_msgs::GridCells FromRos(const nav_msgs::msg::GridCells& ros)
{
    ::autonomy::commsgs::map_msgs::GridCells  data;
    return data;
}

nav_msgs::msg::MapMetaData ToRos(const ::autonomy::commsgs::map_msgs::MapMetaData& proto)
{
    nav_msgs::msg::MapMetaData data;
    return data;
}

::autonomy::commsgs::map_msgs::MapMetaData FromRos(const nav_msgs::msg::MapMetaData& ros)
{
    ::autonomy::commsgs::map_msgs::MapMetaData  data;
    return data;
}

nav_msgs::msg::OccupancyGrid ToRos(const ::autonomy::commsgs::map_msgs::OccupancyGrid& proto)
{
    nav_msgs::msg::OccupancyGrid data;
    return data;
}

::autonomy::commsgs::map_msgs::OccupancyGrid FromRos(const nav_msgs::msg::OccupancyGrid& ros)
{
    ::autonomy::commsgs::map_msgs::OccupancyGrid data;
    return data;
}

}  // namespace autonomy_ros