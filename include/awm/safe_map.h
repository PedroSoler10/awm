/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

/*
 * The safe map: the SLAM map reduced to four values, with a caution band around obstacles.
 *   -1  unknown
 *    0  free
 *   50  caution: uncertain occupancy, or within the inflate radius of an obstacle
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

// Builds *safe_map from map (header and metadata are copied). Occupancy >= occupied_threshold is
// occupied, >= caution_threshold is caution, >= 0 is free, negative is unknown. Free cells inside
// the square of inflate_radius_cells around an occupied cell become caution. The band is computed
// from the source map, so it never turns an occupied or unknown cell into caution.
void makeSafeMap(const nav_msgs::OccupancyGrid& map, int occupied_threshold, int caution_threshold,
                 int inflate_radius_cells, nav_msgs::OccupancyGrid* safe_map);

}  // namespace awm

#endif  // AWM_SAFE_MAP_H_
