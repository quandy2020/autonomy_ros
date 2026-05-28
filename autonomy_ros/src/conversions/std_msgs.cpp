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

#include "autonomy_ros/conversions/std_msgs.hpp"

#include "autonomy_ros/conversions/detail.hpp"

namespace autonomy_ros
{
namespace
{

void copyMultiArrayDimension(
  const std_msgs::msg::MultiArrayDimension & from,
  ::autonomy::commsgs::std_msgs::MultiArrayDimension & to)
{
  to.label = from.label;
  to.size = from.size;
  to.stride = from.stride;
}

void copyMultiArrayDimension(
  const ::autonomy::commsgs::std_msgs::MultiArrayDimension & from,
  std_msgs::msg::MultiArrayDimension & to)
{
  to.label = from.label;
  to.size = from.size;
  to.stride = from.stride;
}

void copyMultiArrayLayout(
  const std_msgs::msg::MultiArrayLayout & from,
  ::autonomy::commsgs::std_msgs::MultiArrayLayout & to)
{
  to.dim.reserve(from.dim.size());
  for (const auto & dim : from.dim) {
    ::autonomy::commsgs::std_msgs::MultiArrayDimension to_dim;
    copyMultiArrayDimension(dim, to_dim);
    to.dim.push_back(to_dim);
  }
  to.data_offset = from.data_offset;
}

void copyMultiArrayLayout(
  const ::autonomy::commsgs::std_msgs::MultiArrayLayout & from,
  std_msgs::msg::MultiArrayLayout & to)
{
  to.dim.reserve(from.dim.size());
  for (const auto & dim : from.dim) {
    std_msgs::msg::MultiArrayDimension to_dim;
    copyMultiArrayDimension(dim, to_dim);
    to.dim.push_back(to_dim);
  }
  to.data_offset = from.data_offset;
}

}  // namespace

Header fromRos(const std_msgs::msg::Header & from)
{
  Header to;
  copyHeader(from, to);
  return to;
}

std_msgs::msg::Header toRos(const Header & from)
{
  std_msgs::msg::Header to;
  copyHeader(from, to);
  return to;
}

ColorRGBA fromRos(const std_msgs::msg::ColorRGBA & from)
{
  ColorRGBA to;
  copyColorRGBA(from, to);
  return to;
}

std_msgs::msg::ColorRGBA toRos(const ColorRGBA & from)
{
  std_msgs::msg::ColorRGBA to;
  copyColorRGBA(from, to);
  return to;
}

MultiArrayDimension fromRos(const std_msgs::msg::MultiArrayDimension & from)
{
  MultiArrayDimension to;
  copyMultiArrayDimension(from, to);
  return to;
}

std_msgs::msg::MultiArrayDimension toRos(const MultiArrayDimension & from)
{
  std_msgs::msg::MultiArrayDimension to;
  copyMultiArrayDimension(from, to);
  return to;
}

MultiArrayLayout fromRos(const std_msgs::msg::MultiArrayLayout & from)
{
  MultiArrayLayout to;
  copyMultiArrayLayout(from, to);
  return to;
}

std_msgs::msg::MultiArrayLayout toRos(const MultiArrayLayout & from)
{
  std_msgs::msg::MultiArrayLayout to;
  copyMultiArrayLayout(from, to);
  return to;
}

Float32MultiArray fromRos(const std_msgs::msg::Float32MultiArray & from)
{
  Float32MultiArray to;
  copyMultiArrayLayout(from.layout, to.layout);
  to.data.assign(from.data.begin(), from.data.end());
  return to;
}

std_msgs::msg::Float32MultiArray toRos(const Float32MultiArray & from)
{
  std_msgs::msg::Float32MultiArray to;
  copyMultiArrayLayout(from.layout, to.layout);
  to.data.assign(from.data.begin(), from.data.end());
  return to;
}

String fromRos(const std_msgs::msg::String & from)
{
  String to;
  to.data = from.data;
  return to;
}

std_msgs::msg::String toRos(const String & from)
{
  std_msgs::msg::String to;
  to.data = from.data;
  return to;
}

}  // namespace autonomy_ros
