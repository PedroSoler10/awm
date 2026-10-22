/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

/*
 * Text dump of a grid, one character per cell, for looking at a map in a terminal or a file.
 *   *  occupied (>= 90)    +  caution (>= 50)    ·  unknown    (space)  free    8  marked cell
 * The top row of the file is the highest y, so the picture has the same orientation as in RViz.
 */

#ifndef AWM_MAP_DUMP_H_
#define AWM_MAP_DUMP_H_

#include <nav_msgs/OccupancyGrid.h>

#include <string>
#include <vector>

#include "awm/grid.h"

namespace awm {

// Writes the grid to path, replacing the file. Returns false if data does not hold width * height
// cells or the file cannot be opened.
bool writeMapText(const nav_msgs::OccupancyGrid& grid, const std::string& path);

// The same, drawing an 8 over each cell in marks (for example the goals just computed). Marks
// outside the grid are ignored.
bool writeMapText(const nav_msgs::OccupancyGrid& grid, const std::string& path,
                  const std::vector<GridCell>& marks);

}  // namespace awm

#endif  // AWM_MAP_DUMP_H_
