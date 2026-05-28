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

#include "autonomy_ros/logger.hpp"

#include <chrono>
#include <string>
#include <thread>

#include "glog/log_severity.h"

namespace autonomy_ros
{

ScopedRosLogSink::ScopedRosLogSink()
{
  ::google::AddLogSink(this);
}

ScopedRosLogSink::~ScopedRosLogSink()
{
  ::google::RemoveLogSink(this);
}

void ScopedRosLogSink::send(
  const ::google::LogSeverity severity,
  const char * const filename,
  const char * const base_filename,
  const int line,
  const std::tm * const tm_time,
  const char * const message,
  const std::size_t message_len)
{
  (void)filename;
  (void)base_filename;
  (void)line;
  (void)tm_time;

  std::string message_string(message, message_len);
  while (!message_string.empty() && message_string.back() == '\n') {
    message_string.pop_back();
  }

  switch (severity) {
    case ::google::GLOG_INFO:
      RCLCPP_INFO_STREAM(logger_, message_string);
      break;
    case ::google::GLOG_WARNING:
      RCLCPP_WARN_STREAM(logger_, message_string);
      break;
    case ::google::GLOG_ERROR:
      RCLCPP_ERROR_STREAM(logger_, message_string);
      break;
    case ::google::GLOG_FATAL:
      RCLCPP_FATAL_STREAM(logger_, message_string);
      will_die_ = true;
      break;
    default:
      RCLCPP_INFO_STREAM(logger_, message_string);
      break;
  }
}

void ScopedRosLogSink::WaitTillSent()
{
  if (will_die_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

}  // namespace autonomy_ros
