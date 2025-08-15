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

#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "rclcpp/rclcpp.hpp"


namespace autonomy_ros {

// Time
rclcpp::Time ToRos(const ::autonomy::commsgs::builtin_interfaces::Time& data);
::autonomy::commsgs::builtin_interfaces::Time FromRos(const rclcpp::Time& ros);

// // Duration
// rclcpp::Duration ToRos(const ::autonomy::commsgs::builtin_interfaces::Duration& data);
// ::autonomy::commsgs::builtin_interfaces::Duration FromRos(const rclcpp::Duration& ros);


}  // namespace autonomy_ros
