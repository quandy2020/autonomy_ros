// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autonomy_ros/conversions/visualization_msgs.hpp"

#include "autonomy_ros/conversions/detail.hpp"
#include "autonomy_ros/conversions/geometry_msgs.hpp"
#include "autonomy_ros/conversions/sensor_msgs.hpp"
#include "autonomy_ros/conversions/std_msgs.hpp"

namespace autonomy_ros::conversions
{

MenuEntry fromRos(const visualization_msgs::msg::MenuEntry & from)
{
  MenuEntry to;
  to.id = from.id;
  to.parent_id = from.parent_id;
  to.title = from.title;
  to.command = from.command;
  return to;
}

visualization_msgs::msg::MenuEntry toRos(const MenuEntry & from)
{
  visualization_msgs::msg::MenuEntry to;
  to.id = from.id;
  to.parent_id = from.parent_id;
  to.title = from.title;
  to.command = from.command;
  to.command_type = visualization_msgs::msg::MenuEntry::FEEDBACK;
  return to;
}

MeshFile fromRos(const visualization_msgs::msg::MeshFile & from)
{
  MeshFile to;
  to.filename = from.filename;
  to.data.assign(from.data.begin(), from.data.end());
  return to;
}

visualization_msgs::msg::MeshFile toRos(const MeshFile & from)
{
  visualization_msgs::msg::MeshFile to;
  to.filename = from.filename;
  to.data.assign(from.data.begin(), from.data.end());
  return to;
}

UVCoordinate fromRos(const visualization_msgs::msg::UVCoordinate & from)
{
  UVCoordinate to;
  to.u = from.u;
  to.v = from.v;
  return to;
}

visualization_msgs::msg::UVCoordinate toRos(const UVCoordinate & from)
{
  visualization_msgs::msg::UVCoordinate to;
  to.u = from.u;
  to.v = from.v;
  return to;
}

ImageMarker fromRos(const visualization_msgs::msg::ImageMarker & from)
{
  ImageMarker to;
  detail::copyHeader(from.header, to.header);
  to.ns = from.ns;
  to.id = from.id;
  to.type = from.type;
  to.action = from.action;
  to.position = fromRos(from.position);
  to.scale = from.scale;
  to.outline_color = fromRos(from.outline_color);
  to.filled = from.filled;
  to.fill_color = fromRos(from.fill_color);
  detail::copyDuration(from.lifetime, to.lifetime);
  to.points.reserve(from.points.size());
  for (const auto & p : from.points) {
    to.points.push_back(fromRos(p));
  }
  to.outline_colors.reserve(from.outline_colors.size());
  for (const auto & c : from.outline_colors) {
    to.outline_colors.push_back(fromRos(c));
  }
  return to;
}

visualization_msgs::msg::ImageMarker toRos(const ImageMarker & from)
{
  visualization_msgs::msg::ImageMarker to;
  detail::copyHeader(from.header, to.header);
  to.ns = from.ns;
  to.id = from.id;
  to.type = from.type;
  to.action = from.action;
  to.position = toRos(from.position);
  to.scale = from.scale;
  to.outline_color = toRos(from.outline_color);
  to.filled = from.filled;
  to.fill_color = toRos(from.fill_color);
  detail::copyDuration(from.lifetime, to.lifetime);
  to.points.reserve(from.points.size());
  for (const auto & p : from.points) {
    to.points.push_back(toRos(p));
  }
  to.outline_colors.reserve(from.outline_colors.size());
  for (const auto & c : from.outline_colors) {
    to.outline_colors.push_back(toRos(c));
  }
  return to;
}

Marker fromRos(const visualization_msgs::msg::Marker & from)
{
  Marker to;
  detail::copyHeader(from.header, to.header);
  to.ns = from.ns;
  to.id = from.id;
  to.type = from.type;
  to.action = from.action;
  to.pose = fromRos(from.pose);
  to.scale = fromRos(from.scale);
  to.color = fromRos(from.color);
  detail::copyDuration(from.lifetime, to.lifetime);
  to.frame_locked = from.frame_locked;
  to.points.reserve(from.points.size());
  for (const auto & p : from.points) {
    to.points.push_back(fromRos(p));
  }
  to.colors.reserve(from.colors.size());
  for (const auto & c : from.colors) {
    to.colors.push_back(fromRos(c));
  }
  to.texture_resource = from.texture_resource;
  to.texture = fromRos(from.texture);
  to.uv_coordinates.reserve(from.uv_coordinates.size());
  for (const auto & uv : from.uv_coordinates) {
    to.uv_coordinates.push_back(fromRos(uv));
  }
  to.text = from.text;
  to.mesh_resource = from.mesh_resource;
  to.mesh_file = fromRos(from.mesh_file);
  to.mesh_use_embedded_materials = from.mesh_use_embedded_materials;
  return to;
}

visualization_msgs::msg::Marker toRos(const Marker & from)
{
  visualization_msgs::msg::Marker to;
  detail::copyHeader(from.header, to.header);
  to.ns = from.ns;
  to.id = from.id;
  to.type = from.type;
  to.action = from.action;
  to.pose = toRos(from.pose);
  to.scale = toRos(from.scale);
  to.color = toRos(from.color);
  detail::copyDuration(from.lifetime, to.lifetime);
  to.frame_locked = from.frame_locked;
  to.points.reserve(from.points.size());
  for (const auto & p : from.points) {
    to.points.push_back(toRos(p));
  }
  to.colors.reserve(from.colors.size());
  for (const auto & c : from.colors) {
    to.colors.push_back(toRos(c));
  }
  to.texture_resource = from.texture_resource;
  to.texture = toRos(from.texture);
  to.uv_coordinates.reserve(from.uv_coordinates.size());
  for (const auto & uv : from.uv_coordinates) {
    to.uv_coordinates.push_back(toRos(uv));
  }
  to.text = from.text;
  to.mesh_resource = from.mesh_resource;
  to.mesh_file = toRos(from.mesh_file);
  to.mesh_use_embedded_materials = from.mesh_use_embedded_materials;
  return to;
}

MarkerArray fromRos(const visualization_msgs::msg::MarkerArray & from)
{
  MarkerArray to;
  to.markers.reserve(from.markers.size());
  for (const auto & m : from.markers) {
    to.markers.push_back(fromRos(m));
  }
  return to;
}

visualization_msgs::msg::MarkerArray toRos(const MarkerArray & from)
{
  visualization_msgs::msg::MarkerArray to;
  to.markers.reserve(from.markers.size());
  for (const auto & m : from.markers) {
    to.markers.push_back(toRos(m));
  }
  return to;
}

InteractiveMarkerControl fromRos(
  const visualization_msgs::msg::InteractiveMarkerControl & from)
{
  InteractiveMarkerControl to;
  to.name = from.name;
  to.orientation = fromRos(from.orientation);
  to.orientation_mode = from.orientation_mode;
  to.interaction_mode = from.interaction_mode;
  to.always_visible = from.always_visible;
  to.markers.reserve(from.markers.size());
  for (const auto & m : from.markers) {
    to.markers.push_back(fromRos(m));
  }
  to.independent_marker_orientation = from.independent_marker_orientation;
  to.description = from.description;
  return to;
}

visualization_msgs::msg::InteractiveMarkerControl toRos(
  const InteractiveMarkerControl & from)
{
  visualization_msgs::msg::InteractiveMarkerControl to;
  to.name = from.name;
  to.orientation = toRos(from.orientation);
  to.orientation_mode = from.orientation_mode;
  to.interaction_mode = from.interaction_mode;
  to.always_visible = from.always_visible;
  to.markers.reserve(from.markers.size());
  for (const auto & m : from.markers) {
    to.markers.push_back(toRos(m));
  }
  to.independent_marker_orientation = from.independent_marker_orientation;
  to.description = from.description;
  return to;
}

InteractiveMarkerFeedback fromRos(
  const visualization_msgs::msg::InteractiveMarkerFeedback & from)
{
  InteractiveMarkerFeedback to;
  detail::copyHeader(from.header, to.header);
  to.client_id = from.client_id;
  to.marker_name = from.marker_name;
  to.control_name = from.control_name;
  to.event_type = from.event_type;
  to.pose = fromRos(from.pose);
  to.menu_entry_id = from.menu_entry_id;
  to.mouse_point = fromRos(from.mouse_point);
  to.mouse_point_valid = from.mouse_point_valid;
  return to;
}

visualization_msgs::msg::InteractiveMarkerFeedback toRos(
  const InteractiveMarkerFeedback & from)
{
  visualization_msgs::msg::InteractiveMarkerFeedback to;
  detail::copyHeader(from.header, to.header);
  to.client_id = from.client_id;
  to.marker_name = from.marker_name;
  to.control_name = from.control_name;
  to.event_type = from.event_type;
  to.pose = toRos(from.pose);
  to.menu_entry_id = from.menu_entry_id;
  to.mouse_point = toRos(from.mouse_point);
  to.mouse_point_valid = from.mouse_point_valid;
  return to;
}

InteractiveMarker fromRos(const visualization_msgs::msg::InteractiveMarker & from)
{
  InteractiveMarker to;
  detail::copyHeader(from.header, to.header);
  to.pose = fromRos(from.pose);
  to.name = from.name;
  to.description = from.description;
  to.scale = from.scale;
  to.menu_entries.reserve(from.menu_entries.size());
  for (const auto & entry : from.menu_entries) {
    to.menu_entries.push_back(fromRos(entry));
  }
  to.controls.reserve(from.controls.size());
  for (const auto & ctrl : from.controls) {
    to.controls.push_back(fromRos(ctrl));
  }
  return to;
}

visualization_msgs::msg::InteractiveMarker toRos(const InteractiveMarker & from)
{
  visualization_msgs::msg::InteractiveMarker to;
  detail::copyHeader(from.header, to.header);
  to.pose = toRos(from.pose);
  to.name = from.name;
  to.description = from.description;
  to.scale = from.scale;
  to.menu_entries.reserve(from.menu_entries.size());
  for (const auto & entry : from.menu_entries) {
    to.menu_entries.push_back(toRos(entry));
  }
  to.controls.reserve(from.controls.size());
  for (const auto & ctrl : from.controls) {
    to.controls.push_back(toRos(ctrl));
  }
  return to;
}

InteractiveMarkerInit fromRos(
  const visualization_msgs::msg::InteractiveMarkerInit & from)
{
  InteractiveMarkerInit to;
  to.server_id = from.server_id;
  to.seq_num = from.seq_num;
  to.markers.reserve(from.markers.size());
  for (const auto & m : from.markers) {
    to.markers.push_back(fromRos(m));
  }
  return to;
}

visualization_msgs::msg::InteractiveMarkerInit toRos(
  const InteractiveMarkerInit & from)
{
  visualization_msgs::msg::InteractiveMarkerInit to;
  to.server_id = from.server_id;
  to.seq_num = from.seq_num;
  to.markers.reserve(from.markers.size());
  for (const auto & m : from.markers) {
    to.markers.push_back(toRos(m));
  }
  return to;
}

InteractiveMarkerPose fromRos(
  const visualization_msgs::msg::InteractiveMarkerPose & from)
{
  InteractiveMarkerPose to;
  detail::copyHeader(from.header, to.header);
  to.pose = fromRos(from.pose);
  to.name = from.name;
  return to;
}

visualization_msgs::msg::InteractiveMarkerPose toRos(
  const InteractiveMarkerPose & from)
{
  visualization_msgs::msg::InteractiveMarkerPose to;
  detail::copyHeader(from.header, to.header);
  to.pose = toRos(from.pose);
  to.name = from.name;
  return to;
}

InteractiveMarkerUpdate fromRos(
  const visualization_msgs::msg::InteractiveMarkerUpdate & from)
{
  InteractiveMarkerUpdate to;
  to.server_id = from.server_id;
  to.seq_num = from.seq_num;
  to.type = from.type;
  to.markers.reserve(from.markers.size());
  for (const auto & m : from.markers) {
    to.markers.push_back(fromRos(m));
  }
  to.poses.reserve(from.poses.size());
  for (const auto & p : from.poses) {
    to.poses.push_back(fromRos(p));
  }
  to.erases = from.erases;
  return to;
}

visualization_msgs::msg::InteractiveMarkerUpdate toRos(
  const InteractiveMarkerUpdate & from)
{
  visualization_msgs::msg::InteractiveMarkerUpdate to;
  to.server_id = from.server_id;
  to.seq_num = from.seq_num;
  to.type = from.type;
  to.markers.reserve(from.markers.size());
  for (const auto & m : from.markers) {
    to.markers.push_back(toRos(m));
  }
  to.poses.reserve(from.poses.size());
  for (const auto & p : from.poses) {
    to.poses.push_back(toRos(p));
  }
  to.erases = from.erases;
  return to;
}

}  // namespace autonomy_ros::conversions
