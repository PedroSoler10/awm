/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

/*
 * Helpers on nav_msgs/OccupancyGrid. Cell (x, y) is column x, row y, stored at
 * data[y * width + x]. None of them needs a running ROS master, so they can be unit tested.
 */

#ifndef AWM_GRID_H_
#define AWM_GRID_H_

#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>

#include <cstdint>
#include <vector>

namespace awm {

// A cell of a grid: x = column, y = row.
struct GridCell {
  int x;
  int y;
};

// World position (metres) -> grid cell (x = column, y = row), rounded to the nearest cell.
void worldToGrid(const nav_msgs::OccupancyGrid& grid, double world_x, double world_y, int* x,
                 int* y);
void worldToGrid(const nav_msgs::OccupancyGrid& grid, const geometry_msgs::Pose& pose, int* x,
                 int* y);

// Grid cell -> world position (metres). The inverse of worldToGrid.
void gridToWorld(const nav_msgs::OccupancyGrid& grid, int x, int y, double* world_x,
                 double* world_y);

// Cell (x, y) of the grid, or nullptr if it is outside the grid (or beyond the end of data).
std::int8_t* cell(nav_msgs::OccupancyGrid* grid, int x, int y);
const std::int8_t* cell(const nav_msgs::OccupancyGrid& grid, int x, int y);

// Sets to `value` every cell within radius_m metres of (cx, cy), except cells equal to keep_value.
void markDisk(nav_msgs::OccupancyGrid* grid, int cx, int cy, double radius_m, std::int8_t value,
              std::int8_t keep_value);

// For every source cell above threshold, sets to 100 the target cells in the square of
// radius_cells around it, except cells equal to keep_value. source and target must have the same
// size: source decides where the obstacles are, target receives the margin.
void inflate(const nav_msgs::OccupancyGrid& source, int threshold, int radius_cells,
             nav_msgs::OccupancyGrid* target, std::int8_t keep_value);

// Looks for a cell equal to wanted_value on circles of growing radius d = 0..max_radius_cells
// around (x0, y0), sweeping the angle from 0 to 359 degrees. The first match is the closest cell:
// writes it to (x, y), the angle it was found at to angle_deg, and returns true. Returns false if
// there is no such cell within max_radius_cells.
bool findNearestCell(const nav_msgs::OccupancyGrid& grid, int x0, int y0, std::int8_t wanted_value,
                     int max_radius_cells, int* x, int* y, int* angle_deg);

// The same ring search, collecting up to n different cells equal to wanted_value, closest first
// (n = 0 collects them all). *out is cleared first; angles_deg, if not null, receives the angle
// each cell was found at. Returns false if no cell was found. The sweep samples 360 angles per
// circle, so on circles larger than about 57 cells it can skip cells between two samples.
bool findNearestCells(const nav_msgs::OccupancyGrid& grid, int x0, int y0, std::int8_t wanted_value,
                      int max_radius_cells, int n, std::vector<GridCell>* out,
                      std::vector<int>* angles_deg = nullptr);

}  // namespace awm

#endif  // AWM_GRID_H_
