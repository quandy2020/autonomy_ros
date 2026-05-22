// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autonomy_ros/conversions/shape_msgs.hpp"

#include <algorithm>

#include "autonomy_ros/conversions/geometry_msgs.hpp"

namespace autonomy_ros::conversions
{

Plane fromRos(const shape_msgs::msg::Plane & from)
{
  Plane to;
  to.coef.assign(from.coef.begin(), from.coef.end());
  return to;
}

shape_msgs::msg::Plane toRos(const Plane & from)
{
  shape_msgs::msg::Plane to;
  const size_t n = std::min(from.coef.size(), to.coef.size());
  for (size_t i = 0; i < n; ++i) {
    to.coef[i] = from.coef[i];
  }
  return to;
}

SolidPrimitive fromRos(const shape_msgs::msg::SolidPrimitive & from)
{
  SolidPrimitive to;
  to.type = from.type;
  to.dimensions.assign(from.dimensions.begin(), from.dimensions.end());
  to.polygon = fromRos(from.polygon);
  return to;
}

shape_msgs::msg::SolidPrimitive toRos(const SolidPrimitive & from)
{
  shape_msgs::msg::SolidPrimitive to;
  to.type = static_cast<uint8_t>(from.type);
  const size_t n = std::min(from.dimensions.size(), to.dimensions.size());
  for (size_t i = 0; i < n; ++i) {
    to.dimensions[i] = from.dimensions[i];
  }
  to.polygon = toRos(from.polygon);
  return to;
}

MeshTriangle fromRos(const shape_msgs::msg::MeshTriangle & from)
{
  MeshTriangle to;
  to.vertex_indices.assign(from.vertex_indices.begin(), from.vertex_indices.end());
  return to;
}

shape_msgs::msg::MeshTriangle toRos(const MeshTriangle & from)
{
  shape_msgs::msg::MeshTriangle to;
  const size_t n = std::min(from.vertex_indices.size(), to.vertex_indices.size());
  for (size_t i = 0; i < n; ++i) {
    to.vertex_indices[i] = static_cast<uint32_t>(from.vertex_indices[i]);
  }
  return to;
}

Mesh fromRos(const shape_msgs::msg::Mesh & from)
{
  Mesh to;
  to.triangles.reserve(from.triangles.size());
  for (const auto & tri : from.triangles) {
    to.triangles.push_back(fromRos(tri));
  }
  to.vertices.reserve(from.vertices.size());
  for (const auto & v : from.vertices) {
    to.vertices.push_back(fromRos(v));
  }
  return to;
}

shape_msgs::msg::Mesh toRos(const Mesh & from)
{
  shape_msgs::msg::Mesh to;
  to.triangles.reserve(from.triangles.size());
  for (const auto & tri : from.triangles) {
    to.triangles.push_back(toRos(tri));
  }
  to.vertices.reserve(from.vertices.size());
  for (const auto & v : from.vertices) {
    to.vertices.push_back(toRos(v));
  }
  return to;
}

}  // namespace autonomy_ros::conversions
