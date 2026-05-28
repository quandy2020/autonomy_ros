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

#include "autonomy_ros/conversions/sensor_msgs.hpp"

#include <algorithm>
#include <array>

#include "autonomy_ros/conversions/detail.hpp"

namespace autonomy_ros
{
namespace
{

void copyCovariance9(
  const std::array<double, 9> & from,
  std::vector<double> & to)
{
  to.assign(from.begin(), from.end());
}

void copyCovariance9(
  const std::vector<double> & from,
  std::array<double, 9> & to)
{
  if (from.size() >= to.size()) {
    std::copy_n(from.begin(), to.size(), to.begin());
  }
}

template<typename ArrayT>
void copyFixedArray(
  const ArrayT & from,
  std::vector<double> & to)
{
  to.assign(from.begin(), from.end());
}

template<size_t N>
void copyFixedArray(
  const std::vector<double> & from,
  std::array<double, N> & to)
{
  if (from.size() >= to.size()) {
    std::copy_n(from.begin(), to.size(), to.begin());
  }
}

}  // namespace

RegionOfInterest fromRos(const sensor_msgs::msg::RegionOfInterest & from)
{
  RegionOfInterest to;
  to.x_offset = from.x_offset;
  to.y_offset = from.y_offset;
  to.height = from.height;
  to.width = from.width;
  to.do_rectify = from.do_rectify;
  return to;
}

sensor_msgs::msg::RegionOfInterest toRos(const RegionOfInterest & from)
{
  sensor_msgs::msg::RegionOfInterest to;
  to.x_offset = from.x_offset;
  to.y_offset = from.y_offset;
  to.height = from.height;
  to.width = from.width;
  to.do_rectify = from.do_rectify;
  return to;
}

CameraInfo fromRos(const sensor_msgs::msg::CameraInfo & from)
{
  CameraInfo to;
  copyHeader(from.header, to.header);
  to.height = from.height;
  to.width = from.width;
  to.distortion_model = from.distortion_model;
  to.d.assign(from.d.begin(), from.d.end());
  copyFixedArray(from.k, to.k);
  copyFixedArray(from.r, to.r);
  copyFixedArray(from.p, to.p);
  to.binning_x0 = from.binning_x;
  to.binning_y1 = from.binning_y;
  to.roi2 = fromRos(from.roi);
  return to;
}

sensor_msgs::msg::CameraInfo toRos(const CameraInfo & from)
{
  sensor_msgs::msg::CameraInfo to;
  copyHeader(from.header, to.header);
  to.height = from.height;
  to.width = from.width;
  to.distortion_model = from.distortion_model;
  to.d.assign(from.d.begin(), from.d.end());
  copyFixedArray(from.k, to.k);
  copyFixedArray(from.r, to.r);
  copyFixedArray(from.p, to.p);
  to.binning_x = from.binning_x0;
  to.binning_y = from.binning_y1;
  to.roi = toRos(from.roi2);
  return to;
}

ChannelFloat32 fromRos(const sensor_msgs::msg::ChannelFloat32 & from)
{
  ChannelFloat32 to;
  to.name = from.name;
  to.values.assign(from.values.begin(), from.values.end());
  return to;
}

sensor_msgs::msg::ChannelFloat32 toRos(const ChannelFloat32 & from)
{
  sensor_msgs::msg::ChannelFloat32 to;
  to.name = from.name;
  to.values.assign(from.values.begin(), from.values.end());
  return to;
}

CompressedImage fromRos(const sensor_msgs::msg::CompressedImage & from)
{
  CompressedImage to;
  copyHeader(from.header, to.header);
  to.format = from.format;
  to.data.reserve(from.data.size());
  for (const auto byte : from.data) {
    to.data.push_back(static_cast<uint32_t>(byte));
  }
  return to;
}

sensor_msgs::msg::CompressedImage toRos(const CompressedImage & from)
{
  sensor_msgs::msg::CompressedImage to;
  copyHeader(from.header, to.header);
  to.format = from.format;
  to.data.reserve(from.data.size());
  for (const auto value : from.data) {
    to.data.push_back(static_cast<uint8_t>(value));
  }
  return to;
}

Illuminance fromRos(const sensor_msgs::msg::Illuminance & from)
{
  Illuminance to;
  copyHeader(from.header, to.header);
  to.illuminance = static_cast<float>(from.illuminance);
  to.variance = static_cast<float>(from.variance);
  return to;
}

sensor_msgs::msg::Illuminance toRos(const Illuminance & from)
{
  sensor_msgs::msg::Illuminance to;
  copyHeader(from.header, to.header);
  to.illuminance = from.illuminance;
  to.variance = from.variance;
  return to;
}

Image fromRos(const sensor_msgs::msg::Image & from)
{
  Image to;
  copyHeader(from.header, to.header);
  to.height = from.height;
  to.width = from.width;
  to.encoding = from.encoding;
  to.is_bigendian = from.is_bigendian;
  to.step = from.step;
  to.data.assign(from.data.begin(), from.data.end());
  return to;
}

sensor_msgs::msg::Image toRos(const Image & from)
{
  sensor_msgs::msg::Image to;
  copyHeader(from.header, to.header);
  to.height = from.height;
  to.width = from.width;
  to.encoding = from.encoding;
  to.is_bigendian = from.is_bigendian;
  to.step = from.step;
  to.data.assign(from.data.begin(), from.data.end());
  return to;
}

Imu fromRos(const sensor_msgs::msg::Imu & from)
{
  Imu to;
  copyHeader(from.header, to.header);
  copyQuaternion(from.orientation, to.orientation);
  copyCovariance9(from.orientation_covariance, to.orientation_covariance);
  copyVector3(from.angular_velocity, to.angular_velocity);
  copyCovariance9(from.angular_velocity_covariance, to.angular_velocity_covariance);
  copyVector3(from.linear_acceleration, to.linear_acceleration);
  copyCovariance9(from.linear_acceleration_covariance, to.linear_acceleration_covariance);
  return to;
}

sensor_msgs::msg::Imu toRos(const Imu & from)
{
  sensor_msgs::msg::Imu to;
  copyHeader(from.header, to.header);
  copyQuaternion(from.orientation, to.orientation);
  copyCovariance9(from.orientation_covariance, to.orientation_covariance);
  copyVector3(from.angular_velocity, to.angular_velocity);
  copyCovariance9(from.angular_velocity_covariance, to.angular_velocity_covariance);
  copyVector3(from.linear_acceleration, to.linear_acceleration);
  copyCovariance9(from.linear_acceleration_covariance, to.linear_acceleration_covariance);
  return to;
}

LaserScan fromRos(const sensor_msgs::msg::LaserScan & from)
{
  LaserScan to;
  copyHeader(from.header, to.header);
  to.angle_min = static_cast<float>(from.angle_min);
  to.angle_max = static_cast<float>(from.angle_max);
  to.angle_increment = static_cast<float>(from.angle_increment);
  to.time_increment = static_cast<float>(from.time_increment);
  to.scan_time = static_cast<float>(from.scan_time);
  to.range_min = static_cast<float>(from.range_min);
  to.range_max = static_cast<float>(from.range_max);
  to.ranges.assign(from.ranges.begin(), from.ranges.end());
  to.intensities.assign(from.intensities.begin(), from.intensities.end());
  return to;
}

sensor_msgs::msg::LaserScan toRos(const LaserScan & from)
{
  sensor_msgs::msg::LaserScan to;
  copyHeader(from.header, to.header);
  to.angle_min = from.angle_min;
  to.angle_max = from.angle_max;
  to.angle_increment = from.angle_increment;
  to.time_increment = from.time_increment;
  to.scan_time = from.scan_time;
  to.range_min = from.range_min;
  to.range_max = from.range_max;
  to.ranges.assign(from.ranges.begin(), from.ranges.end());
  to.intensities.assign(from.intensities.begin(), from.intensities.end());
  return to;
}

PointCloud fromRos(const sensor_msgs::msg::PointCloud & from)
{
  PointCloud to;
  copyHeader(from.header, to.header);
  to.points.reserve(from.points.size());
  for (const auto & point : from.points) {
    ::autonomy::commsgs::geometry_msgs::Point32 commsg_point;
    copyPoint32(point, commsg_point);
    to.points.push_back(commsg_point);
  }
  to.channels.reserve(from.channels.size());
  for (const auto & channel : from.channels) {
    to.channels.push_back(fromRos(channel));
  }
  return to;
}

sensor_msgs::msg::PointCloud toRos(const PointCloud & from)
{
  sensor_msgs::msg::PointCloud to;
  copyHeader(from.header, to.header);
  to.points.reserve(from.points.size());
  for (const auto & point : from.points) {
    geometry_msgs::msg::Point32 ros_point;
    copyPoint32(point, ros_point);
    to.points.push_back(ros_point);
  }
  to.channels.reserve(from.channels.size());
  for (const auto & channel : from.channels) {
    to.channels.push_back(toRos(channel));
  }
  return to;
}

PointField fromRos(const sensor_msgs::msg::PointField & from)
{
  PointField to;
  to.name = from.name;
  to.offset = from.offset;
  to.datatype = from.datatype;
  to.count = from.count;
  return to;
}

sensor_msgs::msg::PointField toRos(const PointField & from)
{
  sensor_msgs::msg::PointField to;
  to.name = from.name;
  to.offset = from.offset;
  to.datatype = from.datatype;
  to.count = from.count;
  return to;
}

PointCloud2 fromRos(const sensor_msgs::msg::PointCloud2 & from)
{
  PointCloud2 to;
  copyHeader(from.header, to.header);
  to.height = from.height;
  to.width = from.width;
  to.fields.reserve(from.fields.size());
  for (const auto & field : from.fields) {
    to.fields.push_back(fromRos(field));
  }
  to.is_bigendian = from.is_bigendian;
  to.point_step = from.point_step;
  to.row_step = from.row_step;
  to.data.assign(from.data.begin(), from.data.end());
  to.is_dense = from.is_dense;
  return to;
}

sensor_msgs::msg::PointCloud2 toRos(const PointCloud2 & from)
{
  sensor_msgs::msg::PointCloud2 to;
  copyHeader(from.header, to.header);
  to.height = from.height;
  to.width = from.width;
  to.fields.reserve(from.fields.size());
  for (const auto & field : from.fields) {
    to.fields.push_back(toRos(field));
  }
  to.is_bigendian = from.is_bigendian;
  to.point_step = from.point_step;
  to.row_step = from.row_step;
  to.data.assign(from.data.begin(), from.data.end());
  to.is_dense = from.is_dense;
  return to;
}

Range fromRos(const sensor_msgs::msg::Range & from)
{
  Range to;
  copyHeader(from.header, to.header);
  to.radiation_type = from.radiation_type;
  to.field_of_view = static_cast<float>(from.field_of_view);
  to.min_range = static_cast<float>(from.min_range);
  to.max_range = static_cast<float>(from.max_range);
  to.range = static_cast<float>(from.range);
  return to;
}

sensor_msgs::msg::Range toRos(const Range & from)
{
  sensor_msgs::msg::Range to;
  copyHeader(from.header, to.header);
  to.radiation_type = from.radiation_type;
  to.field_of_view = from.field_of_view;
  to.min_range = from.min_range;
  to.max_range = from.max_range;
  to.range = from.range;
  return to;
}

Joy fromRos(const sensor_msgs::msg::Joy & from)
{
  Joy to;
  copyHeader(from.header, to.header);
  to.axes.assign(from.axes.begin(), from.axes.end());
  to.buttons.assign(from.buttons.begin(), from.buttons.end());
  return to;
}

sensor_msgs::msg::Joy toRos(const Joy & from)
{
  sensor_msgs::msg::Joy to;
  copyHeader(from.header, to.header);
  to.axes.assign(from.axes.begin(), from.axes.end());
  to.buttons.assign(from.buttons.begin(), from.buttons.end());
  return to;
}

}  // namespace autonomy_ros
