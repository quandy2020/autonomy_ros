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
 * @brief Free-space spawn pose search on a 2-D costmap grid.
 */

#ifndef AUTONOMY_PLANNER_SPAWN_POSE_FINDER_HPP_
#define AUTONOMY_PLANNER_SPAWN_POSE_FINDER_HPP_

#include <random>

#include "autonomy/map/costmap_2d/costmap_2d.hpp"

namespace autonomy_planner {

/**
 * @class SpawnPoseFinder
 * @brief Finds robot spawn positions on free, non-inflated costmap cells.
 *
 * Search keeps a margin from map borders because NavFn treats outer cells as
 * lethal obstacles.
 */
class SpawnPoseFinder
{
public:
  /**
   * @brief Construct finder bound to a costmap grid.
   * @param grid Non-owning pointer; must outlive this object.
   */
  explicit SpawnPoseFinder(const autonomy::map::costmap_2d::Costmap2D * grid);

  /**
   * @brief Find a free cell near the map center (spiral outward).
   * @param x Output world x (m).
   * @param y Output world y (m).
   * @param margin_cells Inset from each border in cells.
   * @return True when a valid cell is found.
   */
  bool FindNearCenter(
    double * x, double * y,
    unsigned int margin_cells) const;

  /**
   * @brief Sample a random free cell inside the margin.
   * @param x Output world x (m).
   * @param y Output world y (m).
   * @param margin_cells Inset from each border in cells.
   * @param rng Random engine for sampling.
   * @param max_attempts Maximum draw count.
   * @param avoid_x Previous spawn x for distance filtering (optional).
   * @param avoid_y Previous spawn y for distance filtering (optional).
   * @param avoid_valid When true, prefer cells far from (avoid_x, avoid_y).
   * @param min_relocate_dist Minimum distance from previous pose (m).
   */
  bool FindRandom(
    double * x, double * y,
    unsigned int margin_cells,
    std::mt19937 * rng,
    unsigned int max_attempts,
    double avoid_x, double avoid_y,
    bool avoid_valid,
    double min_relocate_dist) const;

private:
  bool IsSpawnCellOk(
    unsigned int mx, unsigned int my,
    unsigned int margin_cells) const;

  const autonomy::map::costmap_2d::Costmap2D * grid_;
};

}  // namespace autonomy_planner

#endif  // AUTONOMY_PLANNER_SPAWN_POSE_FINDER_HPP_
