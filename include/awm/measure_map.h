/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

/*
 * The measure map: a nav_msgs/OccupancyGrid built on the safe map (awm/safe_map.h), with our own
 * meaning. It says what has been measured; where goals may go is the safe map's job.
 *   100  unknown or occupied: never measured
 *     0  measured
 *    -1  still to be measured: free or caution cells (next to walls too)
 */

#ifndef AWM_MEASURE_MAP_H_
#define AWM_MEASURE_MAP_H_

#include <geometry_msgs/Point.h>
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

// Rebuilds *measure_map from the safe map and the robot position (metres, "map" frame): cells
// within measure_radius_m of the robot are now measured. Cells measured before stay measured,
// unless the safe map now says unknown or occupied. When the safe map changes geometry (SLAM grew
// the map: new size and origin), they are carried over by world position.
void updateMeasureMap(const nav_msgs::OccupancyGrid& safe_map, double robot_x, double robot_y,
                      double measure_radius_m, nav_msgs::OccupancyGrid* measure_map);

// Goals that measure the cells still to be measured nearest to the robot (ring search on
// measure_map from the robot, up to max_radius_cells). A target that is free in safe_map is its
// own goal. A target too close to an obstacle (caution) is measured from the free cell of
// safe_map nearest to it within measure_radius_m. Targets with no such cell are skipped. Collects
// up to n different goals (n = 0: all), nearest target first, as world positions of safe_map
// cell centres. *goals is cleared first; returns false if it stays empty.
bool findMeasurementGoals(const nav_msgs::OccupancyGrid& measure_map,
                          const nav_msgs::OccupancyGrid& safe_map, double robot_x, double robot_y,
                          int max_radius_cells, double measure_radius_m, int n,
                          std::vector<geometry_msgs::Point>* goals);

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
