/*
 * Copyright 2026 autonomy_ros contributors
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

/**
 * @file
 * @brief ROS 2 node wiring SensorHub to topics or mock/hardware drivers.
 */

#include <memory>
#include <string>
#include <vector>

#include "autodriver/app/sensor_manager.hpp"
#include "autodriver/bridge/autonomy/hub_config_loader.hpp"
#include "autodriver/config/hub_config.hpp"
#include "autodriver/types/sensor_type.hpp"
#include "autonomy_driver/ros_sensor_drivers.hpp"
#include "autonomy_driver/ros_snapshot_publisher.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

class DriverHubNode : public rclcpp::Node
{
public:
  DriverHubNode()
  : Node("driver_hub_node")
  {
    declare_parameter<std::string>("configuration_directory", "");
    declare_parameter<std::string>(
      "configuration_file", "driver/autodriver.lua");
    declare_parameter<bool>("use_mock", false);
    declare_parameter<int>("alignment_window_ms", 100);
    declare_parameter<int>("publish_period_ms", 20);
    declare_parameter<int>("buffer_capacity", 32);
    declare_parameter<bool>("publish_ros_topics", false);
    declare_parameter<std::string>("ros_topics.imu", "imu/data");
    declare_parameter<std::string>("ros_topics.gps", "gps/fix");
    declare_parameter<std::string>("ros_topics.odom", "odom");
    declare_parameter<std::string>("ros_topics.laser", "scan");
    declare_parameter<std::string>("ros_topics.range", "range/front");
    declare_parameter<std::string>("ros_topics.camera_color", "camera/color/image_raw");
    declare_parameter<std::string>("ros_topics.camera_depth", "camera/depth/image_raw");
    declare_parameter<std::vector<std::string>>(
      "ros_drivers.types", std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>(
      "ros_drivers.topics", std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>(
      "ros_drivers.sensor_ids", std::vector<std::string>{});

    Setup();
  }

private:
  void Setup()
  {
    const bool use_mock = get_parameter("use_mock").as_bool();
    autodriver::HubConfig config;

    if (use_mock) {
      config = autodriver::DefaultSimulationHubConfig();
    } else {
      const std::string config_dir =
        get_parameter("configuration_directory").as_string();
      const std::string config_file =
        get_parameter("configuration_file").as_string();
      try {
        config = autodriver::bridge::LoadHubConfigFromDirectory(
          config_dir, config_file);
      } catch (const std::exception & ex) {
        RCLCPP_WARN(
          get_logger(),
          "Lua config failed (%s), falling back to ros_drivers parameters",
          ex.what());
        config.register_builtin_mocks = false;
      }
    }

    config.hub_options.alignment_window = std::chrono::milliseconds(
      get_parameter("alignment_window_ms").as_int());
    config.hub_options.publish_period = std::chrono::milliseconds(
      get_parameter("publish_period_ms").as_int());
    config.hub_options.buffer_capacity = static_cast<std::size_t>(
      get_parameter("buffer_capacity").as_int());

    manager_ = std::make_unique<autodriver::SensorManager>(config);

    if (!use_mock && config.drivers.empty()) {
      RegisterRosDriversFromParameters();
    }

    if (get_parameter("publish_ros_topics").as_bool()) {
      autonomy_driver::RosSnapshotTopicNames topics;
      topics.imu = get_parameter("ros_topics.imu").as_string();
      topics.gps = get_parameter("ros_topics.gps").as_string();
      topics.odom = get_parameter("ros_topics.odom").as_string();
      topics.laser = get_parameter("ros_topics.laser").as_string();
      topics.range = get_parameter("ros_topics.range").as_string();
      topics.camera_color = get_parameter("ros_topics.camera_color").as_string();
      topics.camera_depth = get_parameter("ros_topics.camera_depth").as_string();
      snapshot_publisher_ =
        std::make_unique<autonomy_driver::RosSnapshotPublisher>(
          this, std::move(topics));
    }

    manager_->SetRawSampleCallback(
      [this](const autodriver::SensorSample & sample) {
        if (snapshot_publisher_) {
          snapshot_publisher_->PublishRaw(sample);
        }
      });

    manager_->SetAlignedCallback(
      [this](const autodriver::AlignedSnapshot & snapshot) {
        if (snapshot_publisher_) {
          snapshot_publisher_->Publish(snapshot);
        }
        RCLCPP_DEBUG(
          get_logger(), "Aligned snapshot sensors=%zu",
          snapshot.samples.size());
      });

    if (!manager_->Initialize()) {
      for (const auto & err : manager_->init_errors()) {
        RCLCPP_ERROR(get_logger(), "Driver init failed: %s", err.c_str());
      }
      throw std::runtime_error("SensorManager initialize failed");
    }

    if (!manager_->Start()) {
      throw std::runtime_error("SensorManager start failed");
    }

    RCLCPP_INFO(
      get_logger(),
      "driver_hub_node started (mock=%s drivers=%zu publish=%s)",
      use_mock ? "true" : "false",
      config.drivers.size(),
      snapshot_publisher_ ? "true" : "false");
  }

  void RegisterRosDriversFromParameters()
  {
    const auto types = get_parameter("ros_drivers.types").as_string_array();
    const auto topics = get_parameter("ros_drivers.topics").as_string_array();
    const auto sensor_ids =
      get_parameter("ros_drivers.sensor_ids").as_string_array();

    if (types.size() != topics.size()) {
      throw std::runtime_error(
        "ros_drivers.types and ros_drivers.topics length mismatch");
    }

    for (std::size_t i = 0; i < types.size(); ++i) {
      autodriver::SensorId sensor_id =
        (i < sensor_ids.size()) ? sensor_ids[i] : types[i];
      auto driver = autonomy_driver::CreateRosDriver(
        this, types[i], topics[i], sensor_id);
      manager_->hub().RegisterDriver(std::move(driver));
      RCLCPP_INFO(
        get_logger(), "ROS driver type=%s topic=%s id=%s",
        types[i].c_str(), topics[i].c_str(), sensor_id.c_str());
    }
  }

  std::unique_ptr<autodriver::SensorManager> manager_;
  std::unique_ptr<autonomy_driver::RosSnapshotPublisher> snapshot_publisher_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<DriverHubNode>();
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    fprintf(stderr, "driver_hub_node failed: %s\n", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
