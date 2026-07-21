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
 * @brief Implements costmap-based spawn pose search.
 */

#include "autonomy_planner/spawn_pose_finder.hpp"

#include <algorithm>
#include <cmath>

#include "autonomy/map/costmap_2d/cost_values.hpp"

namespace autonomy_planner {

SpawnPoseFinder::SpawnPoseFinder(
  const autonomy::map::costmap_2d::Costmap2D * grid)
: grid_(grid)
{
}

bool SpawnPoseFinder::IsSpawnCellOk(
  unsigned int mx, unsigned int my,
  unsigned int margin_cells) const
{
  if (!grid_) {
    return false;
  }

  const unsigned int size_x = grid_->getSizeInCellsX();
  const unsigned int size_y = grid_->getSizeInCellsY();
  if (mx < margin_cells || my < margin_cells ||
    mx + margin_cells >= size_x || my + margin_cells >= size_y)
  {
    return false;
  }

  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      const auto cost = grid_->getCost(mx + dx, my + dy);
      if (cost >= autonomy::map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return false;
      }
    }
  }
  return grid_->getCost(mx, my) == autonomy::map::costmap_2d::FREE_SPACE;
}

bool SpawnPoseFinder::FindNearCenter(
  double * x, double * y,
  unsigned int margin_cells) const
{
  if (!grid_ || !x || !y) {
    return false;
  }

  const unsigned int size_x = grid_->getSizeInCellsX();
  const unsigned int size_y = grid_->getSizeInCellsY();
  if (size_x <= 2 * margin_cells + 1 || size_y <= 2 * margin_cells + 1) {
    return false;
  }

  const unsigned int cx = size_x / 2;
  const unsigned int cy = size_y / 2;
  const unsigned int max_r =
    std::max(cx, size_x - 1 - cx) + std::max(cy, size_y - 1 - cy);

  for (unsigned int r = 0; r <= max_r; ++r) {
    const int x0 = static_cast<int>(cx) - static_cast<int>(r);
    const int x1 = static_cast<int>(cx) + static_cast<int>(r);
    const int y0 = static_cast<int>(cy) - static_cast<int>(r);
    const int y1 = static_cast<int>(cy) + static_cast<int>(r);

    for (int ix = x0; ix <= x1; ++ix) {
      for (int iy : {y0, y1}) {
        if (ix < 0 || iy < 0 ||
          ix >= static_cast<int>(size_x) || iy >= static_cast<int>(size_y))
        {
          continue;
        }
        const auto mx = static_cast<unsigned int>(ix);
        const auto my = static_cast<unsigned int>(iy);
        if (IsSpawnCellOk(mx, my, margin_cells)) {
          grid_->mapToWorld(mx, my, *x, *y);
          return true;
        }
      }
    }

    for (int iy = y0 + 1; iy <= y1 - 1; ++iy) {
      for (int ix : {x0, x1}) {
        if (ix < 0 || iy < 0 ||
          ix >= static_cast<int>(size_x) || iy >= static_cast<int>(size_y))
        {
          continue;
        }
        const auto mx = static_cast<unsigned int>(ix);
        const auto my = static_cast<unsigned int>(iy);
        if (IsSpawnCellOk(mx, my, margin_cells)) {
          grid_->mapToWorld(mx, my, *x, *y);
          return true;
        }
      }
    }
  }
  return false;
}

bool SpawnPoseFinder::FindRandom(
  double * x, double * y,
  unsigned int margin_cells,
  std::mt19937 * rng,
  unsigned int max_attempts,
  double avoid_x, double avoid_y,
  bool avoid_valid,
  double min_relocate_dist) const
{
  if (!grid_ || !x || !y || !rng || max_attempts == 0) {
    return false;
  }

  const unsigned int size_x = grid_->getSizeInCellsX();
  const unsigned int size_y = grid_->getSizeInCellsY();
  if (size_x <= 2 * margin_cells + 1 || size_y <= 2 * margin_cells + 1) {
    return false;
  }

  const unsigned int min_x = margin_cells;
  const unsigned int max_x = size_x - margin_cells - 1;
  const unsigned int min_y = margin_cells;
  const unsigned int max_y = size_y - margin_cells - 1;

  std::uniform_int_distribution<unsigned int> dist_x(min_x, max_x);
  std::uniform_int_distribution<unsigned int> dist_y(min_y, max_y);

  auto try_sample = [&](bool enforce_relocate_dist) -> bool {
    for (unsigned int attempt = 0; attempt < max_attempts; ++attempt) {
      const auto mx = dist_x(*rng);
      const auto my = dist_y(*rng);
      if (!IsSpawnCellOk(mx, my, margin_cells)) {
        continue;
      }

      double wx = 0.0;
      double wy = 0.0;
      grid_->mapToWorld(mx, my, wx, wy);
      if (enforce_relocate_dist && avoid_valid &&
        std::hypot(wx - avoid_x, wy - avoid_y) < min_relocate_dist)
      {
        continue;
      }

      *x = wx;
      *y = wy;
      return true;
    }
    return false;
  };

  if (try_sample(true)) {
    return true;
  }
  return try_sample(false);
}

}  // namespace autonomy_planner
