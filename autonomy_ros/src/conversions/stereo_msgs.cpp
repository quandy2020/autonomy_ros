// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autonomy_ros/conversions/stereo_msgs.hpp"

#include "autonomy_ros/conversions/detail.hpp"
#include "autonomy_ros/conversions/sensor_msgs.hpp"

namespace autonomy_ros::conversions
{

DisparityImage fromRos(const stereo_msgs::msg::DisparityImage & from)
{
  DisparityImage to;
  detail::copyHeader(from.header, to.header);
  to.image = fromRos(from.image);
  to.f = from.f;
  to.t = from.t;
  to.valid_window = fromRos(from.valid_window);
  to.min_disparity = from.min_disparity;
  to.max_disparity = from.max_disparity;
  to.delta_d = from.delta_d;
  return to;
}

stereo_msgs::msg::DisparityImage toRos(const DisparityImage & from)
{
  stereo_msgs::msg::DisparityImage to;
  detail::copyHeader(from.header, to.header);
  to.image = toRos(from.image);
  to.f = from.f;
  to.t = from.t;
  to.valid_window = toRos(from.valid_window);
  to.min_disparity = from.min_disparity;
  to.max_disparity = from.max_disparity;
  to.delta_d = from.delta_d;
  return to;
}

}  // namespace autonomy_ros::conversions
