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

#include "autonomy_ros/conversions/tf2_msgs.hpp"

#include "autonomy_ros/conversions/geometry_msgs.hpp"

namespace autonomy_ros
{

TransformStampeds fromRos(const tf2_msgs::msg::TFMessage & from)
{
  TransformStampeds to;
  for (const auto & tf : from.transforms) {
    *to.add_transforms() = fromRos(tf);
  }
  return to;
}

tf2_msgs::msg::TFMessage toRos(const TransformStampeds & from)
{
  tf2_msgs::msg::TFMessage to;
  to.transforms.reserve(static_cast<size_t>(from.transforms_size()));
  for (const auto & tf : from.transforms()) {
    to.transforms.push_back(toRos(tf));
  }
  return to;
}

}  // namespace autonomy_ros
