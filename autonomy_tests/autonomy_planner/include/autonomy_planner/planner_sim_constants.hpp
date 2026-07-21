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
 * @brief Shared constants for planner simulation.
 */

#ifndef AUTONOMY_PLANNER_PLANNER_SIM_CONSTANTS_HPP_
#define AUTONOMY_PLANNER_PLANNER_SIM_CONSTANTS_HPP_

namespace autonomy_planner {

/** Default global planner plugin id. */
constexpr char kDefaultPlannerId[] = "navfn_planner";

/** Default static map YAML basename under autonomy config/data. */
constexpr char kDefaultMapFile[] = "map.yaml";

/** Default OccupancyGrid topic for RViz Map display. */
constexpr char kDefaultMapTopic[] = "map";

/** Default RViz 2D Pose Estimate topic. */
constexpr char kDefaultInitialPoseTopic[] = "initialpose";

/** Default RViz navigation goal topic. */
constexpr char kDefaultGoalPoseTopic[] = "goal_pose";

/** Odom is considered synced when within this distance of set_pose (m). */
constexpr double kPoseConvergeDistanceM = 0.15;

}  // namespace autonomy_planner

#endif  // AUTONOMY_PLANNER_PLANNER_SIM_CONSTANTS_HPP_
