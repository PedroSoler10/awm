/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

/*
 * The safe map: the SLAM map reduced to four values, with a caution band around obstacles. It
 * says where the robot may go: goals are only placed on free cells.
 *   -1  unknown
 *    0  free
 *   50  caution: free, but closer than the margin to an obstacle
 *  100  occupied
 */

#ifndef AWM_SAFE_MAP_H_
#define AWM_SAFE_MAP_H_

#include <nav_msgs/OccupancyGrid.h>

#include <cstdint>

namespace awm {

constexpr std::int8_t kSafeUnknown = -1;
constexpr std::int8_t kSafeFree = 0;
constexpr std::int8_t kSafeCaution = 50;
constexpr std::int8_t kSafeOccupied = 100;

// Builds *safe_map from map (header and metadata are copied). Negative occupancy is unknown,
// occupancy >= occupied_threshold is occupied, anything else is free. Free cells closer than
// margin_m to an occupied cell (distance between cell centres) become caution; a cell exactly
// margin_m away stays free. Unknown and occupied cells are never turned into caution.
void makeSafeMap(const nav_msgs::OccupancyGrid& map, int occupied_threshold, double margin_m,
                 nav_msgs::OccupancyGrid* safe_map);

}  // namespace awm

#endif  // AWM_SAFE_MAP_H_
