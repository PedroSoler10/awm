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
#include <unordered_set>

namespace awm {

bool writeMapText(const nav_msgs::OccupancyGrid& grid, const std::string& path) {
  return writeMapText(grid, path, {});
}

bool writeMapText(const nav_msgs::OccupancyGrid& grid, const std::string& path,
                  const std::vector<GridCell>& marks) {
  // data must hold every cell, or the loop below would read past its end.
  if (grid.data.size() != static_cast<std::size_t>(grid.info.width) * grid.info.height)
    return false;
  std::ofstream file(path);
  if (!file) return false;
  std::unordered_set<std::size_t> marked;
  for (const GridCell& m : marks) {
    if (cell(grid, m.x, m.y) != nullptr) marked.insert(m.y * grid.info.width + m.x);
  }
  for (int y = static_cast<int>(grid.info.height) - 1; y >= 0; y--) {
    for (std::size_t x = 0; x < grid.info.width; x++) {
      const int value = grid.data[y * grid.info.width + x];
      if (marked.count(y * grid.info.width + x) > 0)
        file << "8\t";
      else if (value >= 90)
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
