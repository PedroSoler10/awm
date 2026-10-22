/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

#include "awm/safe_map.h"

#include <cstddef>

#include "awm/grid.h"

namespace awm {

void makeSafeMap(const nav_msgs::OccupancyGrid& map, int occupied_threshold, int caution_threshold,
                 int inflate_radius_cells, nav_msgs::OccupancyGrid* safe_map) {
  *safe_map = map;
  for (std::size_t k = 0; k < map.data.size(); k++) {
    if (map.data[k] >= occupied_threshold)
      safe_map->data[k] = kSafeOccupied;
    else if (map.data[k] >= caution_threshold)
      safe_map->data[k] = kSafeCaution;
    else if (map.data[k] >= 0)
      safe_map->data[k] = kSafeFree;
    else
      safe_map->data[k] = kSafeUnknown;
  }

  for (std::size_t k = 0; k < map.data.size(); k++) {
    if (map.data[k] < occupied_threshold) continue;
    const int x = k % map.info.width;
    const int y = k / map.info.width;
    for (int ny = y - inflate_radius_cells; ny <= y + inflate_radius_cells; ny++) {
      for (int nx = x - inflate_radius_cells; nx <= x + inflate_radius_cells; nx++) {
        const std::int8_t* source = cell(map, nx, ny);
        if (source != nullptr && *source >= 0 && *source < caution_threshold)
          *cell(safe_map, nx, ny) = kSafeCaution;
      }
    }
  }
}

}  // namespace awm
