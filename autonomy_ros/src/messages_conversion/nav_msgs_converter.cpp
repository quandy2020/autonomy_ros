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

#include "autonomy_ros/messages_conversion/std_msgs_converter.hpp"
#include "autonomy_ros/messages_conversion/geometry_msgs_converter.hpp"
#include "autonomy_ros/messages_conversion/nav_msgs_converter.hpp"
#include "autonomy_ros/messages_conversion/builtin_interfaces_converter.hpp"


namespace openbot_ros {


nav_msgs::msg::Odometry ToRos(const ::autonomy::commsgs::planning_msgs::Odometry& odom)
{
    nav_msgs::msg::Odometry data;
    return data;
}

::autonomy::commsgs::planning_msgs::Odometry FromRos(const nav_msgs::msg::Odometry& ros)
{
    return {

    };
}

// Path
nav_msgs::msg::Path ToRos(const ::autonomy::commsgs::planning_msgs::Path& data)
{
    nav_msgs::msg::Path ros;
    // ros.header.frame_id = data.header.frame_id;
    // for (auto pose : data.poses) {
    //     ros.poses.push_back(ToRos(pose));
    // }
    return ros;
}

::autonomy::commsgs::planning_msgs::Path FromRos(const nav_msgs::msg::Path& ros)
{
    ::autonomy::commsgs::planning_msgs::Path data;
    return data;
}


}  // namespace openbot_ros