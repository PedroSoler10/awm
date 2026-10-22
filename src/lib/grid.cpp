/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

#include "awm/grid.h"

#include <cmath>
#include <cstddef>
#include <set>
#include <utility>

namespace awm {

void worldToGrid(const nav_msgs::OccupancyGrid& grid, double world_x, double world_y, int* x,
                 int* y) {
  *x = static_cast<int>(std::round((world_x - grid.info.origin.position.x) / grid.info.resolution));
  *y = static_cast<int>(std::round((world_y - grid.info.origin.position.y) / grid.info.resolution));
}

void worldToGrid(const nav_msgs::OccupancyGrid& grid, const geometry_msgs::Pose& pose, int* x,
                 int* y) {
  worldToGrid(grid, pose.position.x, pose.position.y, x, y);
}

void gridToWorld(const nav_msgs::OccupancyGrid& grid, int x, int y, double* world_x,
                 double* world_y) {
  *world_x = x * grid.info.resolution + grid.info.origin.position.x;
  *world_y = y * grid.info.resolution + grid.info.origin.position.y;
}

bool hasFullData(const nav_msgs::OccupancyGrid& grid) {
  return grid.data.size() == static_cast<std::size_t>(grid.info.width) * grid.info.height;
}

namespace {

// Index of cell (x, y) in grid.data, or false if the cell is outside the grid. data is checked
// too, so a message whose data is shorter than width * height is never read past its end.
bool cellIndex(const nav_msgs::OccupancyGrid& grid, int x, int y, std::size_t* index) {
  if (x < 0 || y < 0 || x >= static_cast<int>(grid.info.width) ||
      y >= static_cast<int>(grid.info.height))
    return false;
  *index = static_cast<std::size_t>(y) * grid.info.width + x;
  return *index < grid.data.size();
}

}  // namespace

std::int8_t* cell(nav_msgs::OccupancyGrid* grid, int x, int y) {
  std::size_t index;
  return cellIndex(*grid, x, y, &index) ? &grid->data[index] : nullptr;
}

const std::int8_t* cell(const nav_msgs::OccupancyGrid& grid, int x, int y) {
  std::size_t index;
  return cellIndex(grid, x, y, &index) ? &grid.data[index] : nullptr;
}

void markDisk(nav_msgs::OccupancyGrid* grid, int cx, int cy, double radius_m, std::int8_t value,
              std::int8_t keep_value) {
  const int radius = static_cast<int>(std::ceil(radius_m / grid->info.resolution));
  for (int y = cy - radius; y <= cy + radius; y++) {
    for (int x = cx - radius; x <= cx + radius; x++) {
      std::int8_t* c = cell(grid, x, y);
      const double distance = std::hypot(x - cx, y - cy) * grid->info.resolution;
      if (c != nullptr && distance <= radius_m && *c != keep_value) *c = value;
    }
  }
}

void inflate(const nav_msgs::OccupancyGrid& source, int threshold, int radius_cells,
             nav_msgs::OccupancyGrid* target, std::int8_t keep_value) {
  for (std::size_t k = 0; k < source.data.size(); k++) {
    if (source.data[k] <= threshold) continue;
    const int x = k % source.info.width;
    const int y = k / source.info.width;
    for (int ny = y - radius_cells; ny <= y + radius_cells; ny++) {
      for (int nx = x - radius_cells; nx <= x + radius_cells; nx++) {
        std::int8_t* c = cell(target, nx, ny);
        if (c != nullptr && *c != keep_value) *c = 100;
      }
    }
  }
}

bool findNearestCell(const nav_msgs::OccupancyGrid& grid, int x0, int y0, std::int8_t wanted_value,
                     int max_radius_cells, int* x, int* y, int* angle_deg) {
  std::vector<GridCell> found;
  std::vector<int> angles;
  if (!findNearestCells(grid, x0, y0, wanted_value, max_radius_cells, 1, &found, &angles))
    return false;
  *x = found[0].x;
  *y = found[0].y;
  *angle_deg = angles[0];
  return true;
}

bool findNearestCells(const nav_msgs::OccupancyGrid& grid, int x0, int y0, std::int8_t wanted_value,
                      int max_radius_cells, int n, std::vector<GridCell>* out,
                      std::vector<int>* angles_deg) {
  out->clear();
  if (angles_deg != nullptr) angles_deg->clear();
  // Rounding makes small circles hit the same cell at several angles, and circles of different
  // radius can share a cell too.
  std::set<std::pair<int, int>> seen;
  for (int d = 0; d <= max_radius_cells; d++) {
    for (int a = 0; a < 360; a++) {
      const double angle = a * M_PI / 180;
      const int cx = x0 + static_cast<int>(std::round(d * std::cos(angle)));
      const int cy = y0 + static_cast<int>(std::round(d * std::sin(angle)));
      const std::int8_t* c = cell(grid, cx, cy);
      if (c == nullptr || *c != wanted_value || !seen.insert({cx, cy}).second) continue;
      out->push_back({cx, cy});
      if (angles_deg != nullptr) angles_deg->push_back(a);
      if (n > 0 && static_cast<int>(out->size()) == n) return true;
    }
  }
  return !out->empty();
}

}  // namespace awm
