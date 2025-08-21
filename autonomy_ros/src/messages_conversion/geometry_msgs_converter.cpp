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

#include "autonomy_ros/messages_conversion/geometry_msgs_converter.hpp"
#include "autonomy_ros/messages_conversion/std_msgs_converter.hpp"
#include "autonomy_ros/messages_conversion/builtin_interfaces_converter.hpp"

namespace autonomy_ros {

geometry_msgs::msg::Point ToRos(const ::autonomy::commsgs::geometry_msgs::Point& data)
{
    geometry_msgs::msg::Point ros;
    ros.x = data.x;
    ros.y = data.y;
    ros.z = data.z;
    return ros;
    
}

::autonomy::commsgs::geometry_msgs::Point FromRos(const geometry_msgs::msg::Point& ros)
{
    ::autonomy::commsgs::geometry_msgs::Point data;
    data.x = ros.x;
    data.y = ros.y;
    data.z = ros.z;
    return data;
}

geometry_msgs::msg::Point32 ToRos(const ::autonomy::commsgs::geometry_msgs::Point32& data)
{
    geometry_msgs::msg::Point32 ros;
    ros.x = data.x;
    ros.y = data.y;
    ros.z = data.z;
    return ros;
    
}

::autonomy::commsgs::geometry_msgs::Point32 FromRos(const geometry_msgs::msg::Point32& ros)
{
    ::autonomy::commsgs::geometry_msgs::Point32 data;
    data.x = ros.x;
    data.y = ros.y;
    data.z = ros.z;
    return data;
}

geometry_msgs::msg::PointStamped ToRos(const ::autonomy::commsgs::geometry_msgs::PointStamped& data)
{
    geometry_msgs::msg::PointStamped ros;
    ros.header = ToRos(data.header);
    ros.point = ToRos(data.point);
    return ros;
    
}

::autonomy::commsgs::geometry_msgs::PointStamped FromRos(const geometry_msgs::msg::PointStamped& ros)
{
    ::autonomy::commsgs::geometry_msgs::PointStamped data;
    data.header = FromRos(ros.header);
    data.point = FromRos(ros.point);
    return data;
}

}  // namespace autonomy_ros