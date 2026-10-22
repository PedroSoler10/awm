/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

#include "awm/frontiers.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <unordered_set>

#include "awm/safe_map.h"

namespace awm {

namespace {

constexpr std::int8_t kFree = 0;

bool hasUnknownNeighbour(const nav_msgs::OccupancyGrid& map, int x, int y) {
  for (int ny = y - 1; ny <= y + 1; ny++) {
    for (int nx = x - 1; nx <= x + 1; nx++) {
      const std::int8_t* c = cell(map, nx, ny);
      if ((nx != x || ny != y) && c != nullptr && *c < 0) return true;
    }
  }
  return false;
}

double squaredDistance(double x0, double y0, double x1, double y1) {
  return (x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1);
}

}  // namespace

bool findFrontierCells(const nav_msgs::OccupancyGrid& map, const nav_msgs::OccupancyGrid& safe_map,
                       std::vector<GridCell>* cells) {
  cells->clear();
  if (map.info.width != safe_map.info.width || map.info.height != safe_map.info.height ||
      !hasFullData(map) || !hasFullData(safe_map))
    return false;

  for (int y = 0; y < static_cast<int>(map.info.height); y++) {
    for (int x = 0; x < static_cast<int>(map.info.width); x++) {
      if (*cell(map, x, y) == kFree && *cell(safe_map, x, y) == kSafeFree &&
          hasUnknownNeighbour(map, x, y))
        cells->push_back({x, y});
    }
  }
  return true;
}

void clusterFrontiers(const std::vector<GridCell>& cells, int width, int min_cluster_size,
                      std::vector<std::vector<GridCell>>* clusters) {
  clusters->clear();
  // One key per cell (y * width + x) lets us find the neighbours of a cell without a full grid.
  const auto key = [width](int x, int y) { return static_cast<std::int64_t>(y) * width + x; };
  std::unordered_set<std::int64_t> unvisited;
  for (const GridCell& c : cells) unvisited.insert(key(c.x, c.y));

  for (const GridCell& start : cells) {
    if (unvisited.erase(key(start.x, start.y)) == 0) continue;

    // Breadth-first search: `cluster` doubles as the queue, `next` is the read position.
    std::vector<GridCell> cluster = {start};
    for (std::size_t next = 0; next < cluster.size(); next++) {
      const GridCell current = cluster[next];
      for (int ny = current.y - 1; ny <= current.y + 1; ny++) {
        for (int nx = current.x - 1; nx <= current.x + 1; nx++) {
          // Without the column test, x = -1 would alias the last column of the row above.
          if (nx < 0 || nx >= width) continue;
          if (unvisited.erase(key(nx, ny)) > 0) cluster.push_back({nx, ny});
        }
      }
    }
    if (static_cast<int>(cluster.size()) >= min_cluster_size) clusters->push_back(cluster);
  }
}

void frontierGoals(const std::vector<std::vector<GridCell>>& clusters, int robot_x, int robot_y,
                   std::vector<GridCell>* goals) {
  goals->clear();
  for (const std::vector<GridCell>& cluster : clusters) {
    if (cluster.empty()) continue;
    double centroid_x = 0;
    double centroid_y = 0;
    for (const GridCell& c : cluster) {
      centroid_x += c.x;
      centroid_y += c.y;
    }
    centroid_x /= cluster.size();
    centroid_y /= cluster.size();

    const GridCell* best = &cluster[0];
    for (const GridCell& c : cluster) {
      if (squaredDistance(c.x, c.y, centroid_x, centroid_y) <
          squaredDistance(best->x, best->y, centroid_x, centroid_y))
        best = &c;
    }
    goals->push_back(*best);
  }

  // Stable, so clusters at the same distance keep their order and the result is deterministic.
  std::stable_sort(goals->begin(), goals->end(),
                   [robot_x, robot_y](const GridCell& a, const GridCell& b) {
                     return squaredDistance(a.x, a.y, robot_x, robot_y) <
                            squaredDistance(b.x, b.y, robot_x, robot_y);
                   });
}

}  // namespace awm
