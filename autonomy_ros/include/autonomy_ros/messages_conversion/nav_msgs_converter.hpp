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


#include "autonomy/commsgs/planning_msgs.hpp"

// nav_msgs
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

namespace autonomy_ros {

// Odometry
nav_msgs::msg::Odometry ToRos(const ::autonomy::commsgs::planning_msgs::Odometry& proto);
::autonomy::commsgs::planning_msgs::Odometry FromRos(const nav_msgs::msg::Odometry& ros);

// Path
nav_msgs::msg::Path ToRos(const ::autonomy::commsgs::planning_msgs::Path& proto);
::autonomy::commsgs::planning_msgs::Path FromRos(const nav_msgs::msg::Path& ros);


}  // namespace autonomy_ros
