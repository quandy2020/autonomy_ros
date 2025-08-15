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


#include "autonomy_ros/sensor_bridge.hpp"

namespace autonomy_ros {


std::unique_ptr<::autonomy::sensor::OdometryData> SensorBridge::ToOdometryData(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) 
{
    // const carto::common::Time time = FromRos(msg->header.stamp);
    // const auto sensor_to_tracking = tf_bridge_.LookupToTracking(time, CheckNoLeadingSlash(msg->child_frame_id));
    // if (sensor_to_tracking == nullptr) {
    //     return nullptr;
    // }

    return std::make_unique<::autonomy::sensor::OdometryData>(
        ::autonomy::sensor::OdometryData{
            // time, ToRigid3d(msg->pose.pose) * sensor_to_tracking->inverse()
    });
}

void SensorBridge::HandleOdometryMessage(const std::string& sensor_id, const nav_msgs::msg::Odometry::ConstSharedPtr& msg) 
{
    std::unique_ptr<::autonomy::sensor::OdometryData> odometry_data = ToOdometryData(msg);
    if (odometry_data != nullptr) {
        // trajectory_builder_->AddSensorData(sensor_id,
        //     ::autonomy::sensor::OdometryData{odometry_data->time, odometry_data->pose});
    }
}

const TfBridge& SensorBridge::tf_bridge() const { return tf_bridge_; }




}  // namespace autonomy_ros