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
 * @brief Shared constants for controller simulation.
 */

#ifndef AUTONOMY_CONTROLLER_CONTROLLER_SIM_CONSTANTS_HPP_
#define AUTONOMY_CONTROLLER_CONTROLLER_SIM_CONSTANTS_HPP_

#include <cstddef>

namespace autonomy_controller
{

/** Default controller plugin id (MPPI). */
constexpr char kDefaultControllerId[] = "mppi";

/** Default RViz navigation goal topic. */
constexpr char kDefaultGoalPoseTopic[] = "goal_pose";

/** Max poses retained in the executed-path trail. */
constexpr std::size_t kExecutedPathMaxPoses = 5000;

/** Trim batch size when executed path exceeds kExecutedPathMaxPoses. */
constexpr std::size_t kExecutedPathTrimBatch = 1000;

}  // namespace autonomy_controller

#endif  // AUTONOMY_CONTROLLER_CONTROLLER_SIM_CONSTANTS_HPP_
