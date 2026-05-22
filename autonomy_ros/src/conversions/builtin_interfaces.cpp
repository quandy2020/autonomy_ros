// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autonomy_ros/conversions/builtin_interfaces.hpp"

#include "autonomy_ros/conversions/detail.hpp"

namespace autonomy_ros::conversions
{

Time fromRos(const builtin_interfaces::msg::Time & from)
{
  Time to;
  detail::copyTime(from, to);
  return to;
}

builtin_interfaces::msg::Time toRos(const Time & from)
{
  return detail::toRosTime(from);
}

Duration fromRos(const builtin_interfaces::msg::Duration & from)
{
  Duration to;
  detail::copyDuration(from, to);
  return to;
}

builtin_interfaces::msg::Duration toRos(const Duration & from)
{
  builtin_interfaces::msg::Duration to;
  detail::copyDuration(from, to);
  return to;
}

}  // namespace autonomy_ros::conversions
