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


#pragma once 

#include <string>
#include <vector>

namespace autonomy_ros {

// Default topic names; expected to be remapped as needed.
constexpr char kLaserScanTopic[] = "scan";
constexpr char kMultiEchoLaserScanTopic[] = "echoes";
constexpr char kPointCloud2Topic[] = "points2";
constexpr char kImuTopic[] = "imu";
constexpr char kOdometryTopic[] = "odom";
constexpr char kNavSatFixTopic[] = "fix";
constexpr char kLandmarkTopic[] = "landmark";
constexpr char kOccupancyGridTopic[] = "map";
constexpr char kGlobalPlanTopic[] = "plan";
constexpr char kLocalPlanTopic[] = "local_plan";

// constexpr char kGetTrajectoryStatesServiceName[] = "get_trajectory_states";
constexpr double kOccupancyGridPublishPeriodSec = 1.0;
constexpr double kGlobalTrajectoryPublishPeriodSec = 1.0;
constexpr double kLocalTrajectoryPublishPeriodSec = 20.0;

constexpr double kTopicMismatchCheckDelaySec = 3.0;
constexpr int kInfiniteSubscriberQueueSize = 0;
constexpr int kLatestOnlyPublisherQueueSize = 1;

// For multiple topics adds numbers to the topic name and returns the list.
std::vector<std::string> ComputeRepeatedTopicNames(const std::string& topic, int num_topics);

}  // namespace autonomy_ros
