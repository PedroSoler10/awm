/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

#include "awm/measure_map.h"

#include <cstddef>
#include <ctime>
#include <fstream>

#include "awm/grid.h"

namespace awm {

void updateMeasureMap(const nav_msgs::OccupancyGrid& map, double robot_x, double robot_y,
                      const MeasureMapParams& params, nav_msgs::OccupancyGrid* measure_map) {
  // When the SLAM map grows its size changes and the old measure map no longer fits: start again
  // from the new map (cells measured so far are forgotten).
  const bool resized =
      measure_map->info.width != map.info.width || measure_map->info.height != map.info.height;
  if (resized) *measure_map = map;

  // 1) Obstacles and unknown cells -> blocked. Other cells -> to measure, keeping those already
  //    measured. After a resize the copied values come from the SLAM map, where 0 means "free",
  //    not "measured", so everything is reset.
  for (std::size_t k = 0; k < map.data.size(); k++) {
    if (map.data[k] == -1 || map.data[k] > params.occupied_threshold)
      measure_map->data[k] = kBlocked;
    else if (resized || measure_map->data[k] != kMeasured)
      measure_map->data[k] = kToMeasure;
  }

  // 2) Cells close to the robot are now measured.
  geometry_msgs::Pose robot;
  robot.position.x = robot_x;
  robot.position.y = robot_y;
  int x;
  int y;
  worldToGrid(*measure_map, robot, &x, &y);
  markDisk(measure_map, x, y, params.measure_radius_m, kMeasured, kBlocked);

  // 3) Safety margin: cells next to an obstacle cannot be goals unless measured.
  inflate(map, params.occupied_threshold, params.inflate_radius_cells, measure_map, kMeasured);
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
