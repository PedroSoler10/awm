/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

#include "awm/measure_map.h"

#include <algorithm>
#include <cstddef>
#include <ctime>
#include <fstream>
#include <utility>

namespace awm {

namespace {

// Same cells at the same world positions.
bool sameGeometry(const nav_msgs::MapMetaData& a, const nav_msgs::MapMetaData& b) {
  return a.width == b.width && a.height == b.height && a.resolution == b.resolution &&
         a.origin.position.x == b.origin.position.x && a.origin.position.y == b.origin.position.y;
}

// Marks as measured, in *to, the cells at the world positions of the measured cells of from.
// Cells that are blocked in *to, or outside it, are left alone.
void carryOverMeasuredCells(const nav_msgs::OccupancyGrid& from, nav_msgs::OccupancyGrid* to) {
  for (int y = 0; y < static_cast<int>(from.info.height); y++) {
    for (int x = 0; x < static_cast<int>(from.info.width); x++) {
      const std::int8_t* old_value = cell(from, x, y);
      if (old_value == nullptr || *old_value != kMeasured) continue;
      double world_x;
      double world_y;
      gridToWorld(from, x, y, &world_x, &world_y);
      int new_x;
      int new_y;
      worldToGrid(*to, world_x, world_y, &new_x, &new_y);
      std::int8_t* new_value = cell(to, new_x, new_y);
      if (new_value != nullptr && *new_value != kBlocked) *new_value = kMeasured;
    }
  }
}

}  // namespace

void updateMeasureMap(const nav_msgs::OccupancyGrid& map, double robot_x, double robot_y,
                      const MeasureMapParams& params, nav_msgs::OccupancyGrid* measure_map) {
  // When the SLAM map grows its size and origin change, and the same index no longer is the same
  // place: rebuild from the new map and carry the measured cells over by world position.
  const bool new_geometry = !sameGeometry(measure_map->info, map.info);
  nav_msgs::OccupancyGrid old_measure_map;
  if (new_geometry) {
    old_measure_map = std::move(*measure_map);
    *measure_map = map;
  }

  // 1) Obstacles and unknown cells -> blocked. Other cells -> to measure, keeping those already
  //    measured. After a geometry change the copied values come from the SLAM map, where 0 means
  //    "free", not "measured", so everything is reset and step 2 restores the measured cells.
  for (std::size_t k = 0; k < map.data.size(); k++) {
    if (map.data[k] == -1 || map.data[k] > params.occupied_threshold)
      measure_map->data[k] = kBlocked;
    else if (new_geometry || measure_map->data[k] != kMeasured)
      measure_map->data[k] = kToMeasure;
  }

  // 2) Measured cells of the old geometry.
  if (new_geometry) carryOverMeasuredCells(old_measure_map, measure_map);

  // 3) Cells close to the robot are now measured.
  int x;
  int y;
  worldToGrid(*measure_map, robot_x, robot_y, &x, &y);
  markDisk(measure_map, x, y, params.measure_radius_m, kMeasured, kBlocked);

  // 4) Safety margin: cells next to an obstacle cannot be goals unless measured.
  inflate(map, params.occupied_threshold, params.inflate_radius_cells, measure_map, kMeasured);
}

void removeMeasuredCells(const nav_msgs::OccupancyGrid& measure_map, std::vector<GridCell>* cells) {
  const auto measured = [&measure_map](const GridCell& c) {
    const std::int8_t* value = cell(measure_map, c.x, c.y);
    return value != nullptr && *value == kMeasured;
  };
  cells->erase(std::remove_if(cells->begin(), cells->end(), measured), cells->end());
}

bool writeMeasurementFile(const std::string& dir, const geometry_msgs::Pose& pose,
                          std::string* file_name) {
  // No colons in the time, so the name is also valid on Windows.
  const std::time_t now = std::time(nullptr);
  std::tm local_time;
  char time_text[32] = "";
  if (localtime_r(&now, &local_time) != nullptr)
    std::strftime(time_text, sizeof(time_text), "%Y-%m-%dT%H-%M-%S", &local_time);
  *file_name = std::to_string(pose.position.x) + "_" + std::to_string(pose.position.y) + "_" +
               time_text + ".txt";
  std::ofstream measure_file(dir + "/" + *file_name);
  return measure_file.is_open();
}

}  // namespace awm
