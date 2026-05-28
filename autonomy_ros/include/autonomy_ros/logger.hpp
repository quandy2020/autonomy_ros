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

#ifndef AUTONOMY_ROS__LOGGER_HPP_
#define AUTONOMY_ROS__LOGGER_HPP_

#include <cstddef>
#include <ctime>

#include "glog/logging.h"
#include "rclcpp/rclcpp.hpp"

namespace autonomy_ros
{

// Routes Google glog output to ROS logging while alive.
class ScopedRosLogSink : public ::google::LogSink
{
public:
  ScopedRosLogSink();
  ~ScopedRosLogSink() override;

  void send(
    ::google::LogSeverity severity,
    const char * filename,
    const char * base_filename,
    int line,
    const std::tm * tm_time,
    const char * message,
    std::size_t message_len) override;

  void WaitTillSent() override;

private:
  bool will_die_{false};
  rclcpp::Logger logger_{rclcpp::get_logger("autonomy_ros")};
};

}  // namespace autonomy_ros

#endif  // AUTONOMY_ROS__LOGGER_HPP_
