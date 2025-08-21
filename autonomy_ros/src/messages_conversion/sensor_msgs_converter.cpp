/*
 * Copyright 2024 The OpenRobotic Beginner Authors
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


#include "autonomy_ros/messages_conversion/std_msgs_converter.hpp"
#include "autonomy_ros/messages_conversion/geometry_msgs_converter.hpp"
#include "autonomy_ros/messages_conversion/sensor_msgs_converter.hpp"

namespace autonomy_ros {

sensor_msgs::msg::ChannelFloat32 ToRos(const ::autonomy::commsgs::sensor_msgs::ChannelFloat32& data)
{
    sensor_msgs::msg::ChannelFloat32 ros;
    ros.name = data.name;
    for (auto const& value : data.values) {
        ros.values.push_back(value);
    }
    return ros;
}

::autonomy::commsgs::sensor_msgs::ChannelFloat32 FromRos(const sensor_msgs::msg::ChannelFloat32& ros)
{
    ::autonomy::commsgs::sensor_msgs::ChannelFloat32 data;
    return data;
}

sensor_msgs::msg::PointField ToRos(const ::autonomy::commsgs::sensor_msgs::PointField& data)
{
    sensor_msgs::msg::PointField ros;
    ros.name = data.name;
    ros.offset = data.offset;
    ros.datatype = data.datatype;
    ros.count = data.count;
    return ros;
}

::autonomy::commsgs::sensor_msgs::PointField FromRos(const sensor_msgs::msg::PointField& ros)
{
    ::autonomy::commsgs::sensor_msgs::PointField data;
    return data;
}

sensor_msgs::msg::PointCloud ToRos(const ::autonomy::commsgs::sensor_msgs::PointCloud& data)
{
    sensor_msgs::msg::PointCloud ros;
    ros.header = ToRos(data.header);
    for (const auto& point : data.points) {
        ros.points.push_back(ToRos(point));
    }

    for (const auto& channel : data.channels) {
        ros.channels.push_back(ToRos(channel));
    }
    return ros;
}

::autonomy::commsgs::sensor_msgs::PointCloud FromRos(const sensor_msgs::msg::PointCloud& ros)
{
    ::autonomy::commsgs::sensor_msgs::PointCloud  data;
    return data;
}

sensor_msgs::msg::PointCloud2 ToRos(const ::autonomy::commsgs::sensor_msgs::PointCloud2& data)
{
    sensor_msgs::msg::PointCloud2 ros;
    ros.header = ToRos(data.header);
    ros.height = data.height;
    ros.width = data.width;
    for (auto field : data.fields) {
        ros.fields.push_back(ToRos(field));
    }
    ros.is_bigendian = data.is_bigendian;
    ros.point_step = data.point_step;
    ros.row_step = data.row_step;
    for (auto data : data.data) {
        ros.data.push_back(data);
    }
    ros.is_dense = data.is_dense;
    return ros;
}

::autonomy::commsgs::sensor_msgs::PointCloud2 FromRos(const sensor_msgs::msg::PointCloud2& ros)
{
    ::autonomy::commsgs::sensor_msgs::PointCloud2 data;
    return data;
}


}  // namespace autonomy_ros