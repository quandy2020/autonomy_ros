/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autonomy_ros/time_conversion.hpp"

namespace autonomy_ros {

rclcpp::Time ToRos(::autonomy::commsgs::builtin_interfaces::Time time)
{
    return rclcpp::Time{time.sec, time.nanosec};
}

::autonomy::commsgs::builtin_interfaces::Time FromRos(const rclcpp::Time& time)
{
    return ::autonomy::commsgs::builtin_interfaces::Time {
        time.seconds(),   
        time.nanoseconds() % 1'000'000'000  
    };
}
    
}  // namespace autonomy_ros