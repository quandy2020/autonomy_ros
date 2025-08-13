/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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


#include "autonomy/system/system.hpp"
#include "autonomy_ros/autonomy_node.hpp"
#include "autonomy_ros/node_options.hpp"

#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");

DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

DEFINE_bool(startup_with_default_topics, true,
  "Enable to immediately start with default topics.");

namespace autonomy_ros {
namespace {

void Run() 
{
  rclcpp::Node::SharedPtr ros_node = rclcpp::Node::make_shared("autonomy_node");
  constexpr double kTfBufferCacheTimeInSeconds = 10.;

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(ros_node->get_clock(),
    tf2::durationFromSec(kTfBufferCacheTimeInSeconds), ros_node);

  // std::shared_ptr<tf2_ros::TransformListener> tf_listener =
  //     std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  auto node_options = LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);
  auto autonomy_builder = ::autonomy::system::CreateAutonomyBuilder(node_options.autonomy_options);
  auto node = std::make_shared<autonomy_ros::Node>(node_options, std::move(autonomy_builder), 
      tf_buffer, ros_node, FLAGS_collect_metrics);

  if (FLAGS_startup_with_default_topics) {
      node->StartupWithDefaultTopics();
  }
  rclcpp::spin(ros_node);
}

}  // namespace
}  // namespace autonomy_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  google::AllowCommandLineReparsing();
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  CHECK(!FLAGS_configuration_directory.empty())
    << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
    << "-configuration_basename is missing.";

  autonomy_ros::Run();
  ::rclcpp::shutdown();
  google::ShutdownGoogleLogging();  
  return EXIT_SUCCESS;
}