// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autonomy_ros/conversions/diagnostic_msgs.hpp"

#include "autonomy_ros/conversions/detail.hpp"

namespace autonomy_ros::conversions
{

KeyValue fromRos(const diagnostic_msgs::msg::KeyValue & from)
{
  KeyValue to;
  to.key = from.key;
  to.value = from.value;
  return to;
}

diagnostic_msgs::msg::KeyValue toRos(const KeyValue & from)
{
  diagnostic_msgs::msg::KeyValue to;
  to.key = from.key;
  to.value = from.value;
  return to;
}

DiagnosticStatus fromRos(const diagnostic_msgs::msg::DiagnosticStatus & from)
{
  DiagnosticStatus to;
  to.level = from.level;
  to.name = from.name;
  to.message = from.message;
  to.hardware_id = from.hardware_id;
  to.values.reserve(from.values.size());
  for (const auto & kv : from.values) {
    to.values.push_back(fromRos(kv));
  }
  return to;
}

diagnostic_msgs::msg::DiagnosticStatus toRos(const DiagnosticStatus & from)
{
  diagnostic_msgs::msg::DiagnosticStatus to;
  to.level = from.level;
  to.name = from.name;
  to.message = from.message;
  to.hardware_id = from.hardware_id;
  to.values.reserve(from.values.size());
  for (const auto & kv : from.values) {
    to.values.push_back(toRos(kv));
  }
  return to;
}

DiagnosticArray fromRos(const diagnostic_msgs::msg::DiagnosticArray & from)
{
  DiagnosticArray to;
  detail::copyHeader(from.header, to.header);
  to.status.reserve(from.status.size());
  for (const auto & st : from.status) {
    to.status.push_back(fromRos(st));
  }
  return to;
}

diagnostic_msgs::msg::DiagnosticArray toRos(const DiagnosticArray & from)
{
  diagnostic_msgs::msg::DiagnosticArray to;
  detail::copyHeader(from.header, to.header);
  to.status.reserve(from.status.size());
  for (const auto & st : from.status) {
    to.status.push_back(toRos(st));
  }
  return to;
}

}  // namespace autonomy_ros::conversions
