/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

#include "awm/safe_map.h"

#include <cmath>
#include <cstddef>
#include <vector>

#include "awm/grid.h"

namespace awm {

void makeSafeMap(const nav_msgs::OccupancyGrid& map, int occupied_threshold, double margin_m,
                 nav_msgs::OccupancyGrid* safe_map) {
  *safe_map = map;
  for (std::size_t k = 0; k < map.data.size(); k++) {
    if (map.data[k] < 0)
      safe_map->data[k] = kSafeUnknown;
    else if (map.data[k] >= occupied_threshold)
      safe_map->data[k] = kSafeOccupied;
    else
      safe_map->data[k] = kSafeFree;
  }
  if (!hasFullData(map) || map.info.width == 0 || margin_m <= 0) return;

  // Offsets of the cells closer than margin_m, computed once. The small tolerance keeps a cell
  // exactly margin_m away free although resolution is a float (0.05 is stored as 0.0500000007).
  const double resolution = map.info.resolution;
  const int reach = static_cast<int>(std::ceil(margin_m / resolution));
  std::vector<GridCell> offsets;
  for (int dy = -reach; dy <= reach; dy++) {
    for (int dx = -reach; dx <= reach; dx++) {
      if (std::hypot(dx, dy) * resolution < margin_m - 1e-6) offsets.push_back({dx, dy});
    }
  }

  // Only free cells change, so caution cells never widen the band.
  for (std::size_t k = 0; k < safe_map->data.size(); k++) {
    if (safe_map->data[k] != kSafeOccupied) continue;
    const int x = k % map.info.width;
    const int y = k / map.info.width;
    for (const GridCell& offset : offsets) {
      std::int8_t* c = cell(safe_map, x + offset.x, y + offset.y);
      if (c != nullptr && *c == kSafeFree) *c = kSafeCaution;
    }
  }
}

}  // namespace awm
