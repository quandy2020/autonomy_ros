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

#include <memory>

#include "autolink/init.hpp"
#include "autonomy_ros/options.hpp"
#include "autonomy_ros/logger.hpp"
#include "autonomy_ros/node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy_ros
{

void Run()
{
  auto node = std::make_shared<rclcpp::Node>("autonomy_node");
  auto system = std::make_unique<RosAutonomySystem>(*node, CreateOptions(*node));

  RCLCPP_INFO(
    node->get_logger(),
    "autonomy_ros: core=%s",
    system->IsRunning() ? "running" : "failed");

  rclcpp::spin(node);
}

}  // namespace autonomy_ros

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  if (!autolink::Init(argv[0], "autonomy_ros")) {
    RCLCPP_ERROR(
      rclcpp::get_logger("autonomy_node"),
      "autolink::Init failed; BT navigation will not start.");
    rclcpp::shutdown();
    return 1;
  }

  autonomy_ros::ScopedRosLogSink ros_log_sink;
  autonomy_ros::Run();
  autolink::WaitForShutdown();
  rclcpp::shutdown();
  return 0;
}
