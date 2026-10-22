/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

/*
 * Frontier detection: a frontier is a free cell at the edge of the known map, so driving there
 * reveals new space. Cells are returned in grid coordinates (see awm/grid.h); the caller converts
 * them to world positions with gridToWorld.
 */

#ifndef AWM_FRONTIERS_H_
#define AWM_FRONTIERS_H_

#include <nav_msgs/OccupancyGrid.h>

#include <vector>

#include "awm/grid.h"

namespace awm {

// Cells of the safe map (awm/safe_map.h) that are free (so not in the caution band of a wall) and
// have at least one unknown cell among their 8 neighbours. *cells is cleared first and filled in
// row-major order. Returns false, with *cells empty, if data does not hold width * height cells.
bool findFrontierCells(const nav_msgs::OccupancyGrid& safe_map, std::vector<GridCell>* cells);

// Groups cells that touch (8-connectivity) and drops the groups with fewer than min_cluster_size
// cells, which are usually noise. width is the width of the grid the cells come from. *clusters is
// cleared first; clusters come in the order of their first cell.
void clusterFrontiers(const std::vector<GridCell>& cells, int width, int min_cluster_size,
                      std::vector<std::vector<GridCell>>* clusters);

// One goal per cluster: the cell of the cluster nearest to its centroid (the centroid itself can
// lie off the frontier), sorted by distance to the robot cell, closest first. *goals is cleared
// first.
void frontierGoals(const std::vector<std::vector<GridCell>>& clusters, int robot_x, int robot_y,
                   std::vector<GridCell>* goals);

}  // namespace awm

#endif  // AWM_FRONTIERS_H_
