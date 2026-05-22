// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autonomy_ros/conversions/pcl_msgs.hpp"

#include "autonomy_ros/conversions/detail.hpp"
#include "autonomy_ros/conversions/sensor_msgs.hpp"

namespace autonomy_ros::conversions
{

Vertices fromRos(const pcl_msgs::msg::Vertices & from)
{
  Vertices to;
  to.vertices.assign(from.vertices.begin(), from.vertices.end());
  return to;
}

pcl_msgs::msg::Vertices toRos(const Vertices & from)
{
  pcl_msgs::msg::Vertices to;
  to.vertices.assign(from.vertices.begin(), from.vertices.end());
  return to;
}

ModelCoefficients fromRos(const pcl_msgs::msg::ModelCoefficients & from)
{
  ModelCoefficients to;
  detail::copyHeader(from.header, to.header);
  to.values.assign(from.values.begin(), from.values.end());
  return to;
}

pcl_msgs::msg::ModelCoefficients toRos(const ModelCoefficients & from)
{
  pcl_msgs::msg::ModelCoefficients to;
  detail::copyHeader(from.header, to.header);
  to.values.assign(from.values.begin(), from.values.end());
  return to;
}

PointIndices fromRos(const pcl_msgs::msg::PointIndices & from)
{
  PointIndices to;
  detail::copyHeader(from.header, to.header);
  to.indices.assign(from.indices.begin(), from.indices.end());
  return to;
}

pcl_msgs::msg::PointIndices toRos(const PointIndices & from)
{
  pcl_msgs::msg::PointIndices to;
  detail::copyHeader(from.header, to.header);
  to.indices.assign(from.indices.begin(), from.indices.end());
  return to;
}

PolygonMesh fromRos(const pcl_msgs::msg::PolygonMesh & from)
{
  PolygonMesh to;
  detail::copyHeader(from.header, to.header);
  to.cloud = fromRos(from.cloud);
  to.polygons.reserve(from.polygons.size());
  for (const auto & poly : from.polygons) {
    to.polygons.push_back(fromRos(poly));
  }
  return to;
}

pcl_msgs::msg::PolygonMesh toRos(const PolygonMesh & from)
{
  pcl_msgs::msg::PolygonMesh to;
  detail::copyHeader(from.header, to.header);
  to.cloud = toRos(from.cloud);
  to.polygons.reserve(from.polygons.size());
  for (const auto & poly : from.polygons) {
    to.polygons.push_back(toRos(poly));
  }
  return to;
}

}  // namespace autonomy_ros::conversions
