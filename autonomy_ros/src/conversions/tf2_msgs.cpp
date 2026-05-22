// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autonomy_ros/conversions/tf2_msgs.hpp"

#include "autonomy_ros/conversions/geometry_msgs.hpp"

namespace autonomy_ros::conversions
{

TransformStampeds fromRos(const tf2_msgs::msg::TFMessage & from)
{
  TransformStampeds to;
  to.transforms.reserve(from.transforms.size());
  for (const auto & tf : from.transforms) {
    to.transforms.push_back(fromRos(tf));
  }
  return to;
}

tf2_msgs::msg::TFMessage toRos(const TransformStampeds & from)
{
  tf2_msgs::msg::TFMessage to;
  to.transforms.reserve(from.transforms.size());
  for (const auto & tf : from.transforms) {
    to.transforms.push_back(toRos(tf));
  }
  return to;
}

}  // namespace autonomy_ros::conversions
