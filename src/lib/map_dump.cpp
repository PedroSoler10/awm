/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

#include "awm/map_dump.h"

#include <cstddef>
#include <fstream>

namespace awm {

bool writeMapText(const nav_msgs::OccupancyGrid& grid, const std::string& path) {
  std::ofstream file(path);
  if (!file) return false;
  for (int y = static_cast<int>(grid.info.height) - 1; y >= 0; y--) {
    for (std::size_t x = 0; x < grid.info.width; x++) {
      const int value = grid.data[y * grid.info.width + x];
      if (value >= 90)
        file << "*\t";
      else if (value >= 50)
        file << "+\t";
      else if (value < 0)
        file << "·\t";
      else
        file << " \t";
    }
    file << "\n";
  }
  return true;
}

}  // namespace awm
