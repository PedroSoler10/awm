/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

/*
 * The measure map: a nav_msgs/OccupancyGrid the size of the SLAM map, with our own meaning:
 *   100  obstacle, unknown, or too close to an obstacle (cannot be a goal)
 *     0  free cell already measured
 *    -1  free cell still to be measured  <- candidates for the next goal
 */

#ifndef AWM_MEASURE_MAP_H_
#define AWM_MEASURE_MAP_H_

#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>

#include <cstdint>
#include <string>
#include <vector>

#include "awm/grid.h"

namespace awm {

constexpr std::int8_t kBlocked = 100;
constexpr std::int8_t kMeasured = 0;
constexpr std::int8_t kToMeasure = -1;

struct MeasureMapParams {
  double measure_radius_m;   // Cells this close to the robot are measured.
  int inflate_radius_cells;  // Safety margin around obstacles.
  int occupied_threshold;    // Occupancy above this is an obstacle.
};

// Rebuilds *measure_map from the SLAM map and the robot position (metres, "map" frame). Cells
// measured before are kept, unless the SLAM map changed size. Does not touch the header stamp.
void updateMeasureMap(const nav_msgs::OccupancyGrid& map, double robot_x, double robot_y,
                      const MeasureMapParams& params, nav_msgs::OccupancyGrid* measure_map);

// Removes from *cells the cells that measure_map marks as measured, keeping the order of the
// others. Used to stop offering goals the robot has already been to. Cells outside the measure map
// are kept.
void removeMeasuredCells(const nav_msgs::OccupancyGrid& measure_map, std::vector<GridCell>* cells);

// Placeholder for the real measurement: creates an empty file in dir named after the position and
// the local time, e.g. "1.200000_-0.500000_2026-10-21T10-30-05.txt" (std::to_string keeps six
// decimals). Writes the name (without dir) to *file_name and returns false if the file could not
// be created.
bool writeMeasurementFile(const std::string& dir, const geometry_msgs::Pose& pose,
                          std::string* file_name);

}  // namespace awm

#endif  // AWM_MEASURE_MAP_H_
