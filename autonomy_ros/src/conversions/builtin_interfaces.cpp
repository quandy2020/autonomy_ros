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

#include "autonomy_ros/conversions/builtin_interfaces.hpp"

#include "autonomy_ros/conversions/detail.hpp"

namespace autonomy_ros
{

Time fromRos(const builtin_interfaces::msg::Time & from)
{
  Time to;
  copyTime(from, to);
  return to;
}

builtin_interfaces::msg::Time toRos(const Time & from)
{
  return toRosTime(from);
}

Duration fromRos(const builtin_interfaces::msg::Duration & from)
{
  Duration to;
  copyDuration(from, to);
  return to;
}

builtin_interfaces::msg::Duration toRos(const Duration & from)
{
  builtin_interfaces::msg::Duration to;
  copyDuration(from, to);
  return to;
}

}  // namespace autonomy_ros
