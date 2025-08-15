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

#include "autonomy_ros/messages_conversion/builtin_interfaces_converter.hpp"

namespace autonomy_ros {

rclcpp::Time ToRos(const ::autonomy::commsgs::builtin_interfaces::Time& data)
{
    return rclcpp::Time {
        data.sec,
        data.nanosec
    };
}

::autonomy::commsgs::builtin_interfaces::Time FromRos(const rclcpp::Time& ros)
{
    return ::autonomy::commsgs::builtin_interfaces::Time {
        ros.seconds(),   
        ros.nanoseconds() % 1'000'000'000  
    };
}

// rclcpp::Duration ToRos(const ::autonomy::commsgs::builtin_interfaces::Duration& data)
// {
//     return rclcpp::Duration {
//         ToRos(data.stamp),
//         data.frame_id
//     };
// }

// ::autonomy::commsgs::builtin_interfaces::Duration FromRos(const rclcpp::Duration& ros)
// {
//     return ::autonomy::commsgs::builtin_interfaces::Time {
//         FromRos(ros.stamp),
//         ros.frame_id
//     };
// }

}  // namespace autonomy_ros