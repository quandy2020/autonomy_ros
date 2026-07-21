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
 * @brief ROS 2 topic subscriptions as autodriver SensorDriver backends.
 */

#ifndef AUTONOMY_DRIVER_ROS_SENSOR_DRIVERS_HPP_
#define AUTONOMY_DRIVER_ROS_SENSOR_DRIVERS_HPP_

#include <memory>
#include <string>

#include "autodriver/common/sensor_id.hpp"
#include "autodriver/hal/sensor_driver.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy_driver {

/**
 * @brief Creates a ROS topic driver for the given modality.
 * @param type One of imu, laser, odom, gps, range, camera.
 * @param topic ROS subscription topic (relative or absolute).
 * @param sensor_id HAL instance id stamped on outgoing samples.
 */
std::shared_ptr<autodriver::SensorDriver> CreateRosDriver(
  rclcpp::Node * node,
  const std::string & type,
  const std::string & topic,
  const autodriver::SensorId & sensor_id);

}  // namespace autonomy_driver

#endif  // AUTONOMY_DRIVER_ROS_SENSOR_DRIVERS_HPP_
