/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

/*
 * Text dump of a grid, one character per cell, for looking at a map in a terminal or a file.
 *   *  occupied (>= 90)    +  caution (>= 50)    ·  unknown    (space)  free
 * The top row of the file is the highest y, so the picture has the same orientation as in RViz.
 */

#ifndef AWM_MAP_DUMP_H_
#define AWM_MAP_DUMP_H_

#include <nav_msgs/OccupancyGrid.h>

#include <string>

namespace awm {

// Writes the grid to path, replacing the file. Returns false if the file cannot be opened.
bool writeMapText(const nav_msgs::OccupancyGrid& grid, const std::string& path);

}  // namespace awm

#endif  // AWM_MAP_DUMP_H_
