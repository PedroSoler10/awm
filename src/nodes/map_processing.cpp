/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

/*
 * map_processing: turns the SLAM map into the safe map.
 *
 * Subscribes: /map       (nav_msgs/OccupancyGrid)  SLAM map
 * Publishes:  /safe_map  (nav_msgs/OccupancyGrid, latched)  -1 unknown, 0 free, 50 caution, 100
 *                                                          occupied (see awm/safe_map.h)
 * Private params (defaults in brackets):
 *   ~occupied_threshold [90], ~caution_threshold [50], ~inflate_radius_cells [6],
 *   ~dump_maps [false] write map.txt and safe_map.txt on every map, ~dump_dir [.]
 */

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <cstdio>
#include <string>

#include "awm/map_dump.h"
#include "awm/safe_map.h"

class MapProcessing {
 public:
  // The constructor only sets up. The work happens in mapCallback().
  MapProcessing() {
    ros::NodeHandle pnh("~");
    pnh.param("occupied_threshold", occupied_threshold_, 90);
    pnh.param("caution_threshold", caution_threshold_, 50);
    pnh.param("inflate_radius_cells", inflate_radius_cells_, 6);
    pnh.param("dump_maps", dump_maps_, false);
    pnh.param("dump_dir", dump_dir_, std::string("."));

    // Only with dumps on: the files are ours then, and stale ones would mislead.
    if (dump_maps_) {
      std::remove(dumpPath("map.txt").c_str());
      std::remove(dumpPath("safe_map.txt").c_str());
    }

    pub_safe_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("/safe_map", 10, true);
    sub_map_ = nh_.subscribe("/map", 10, &MapProcessing::mapCallback, this);
  }

 private:
  void mapCallback(const nav_msgs::OccupancyGrid& map) {
    nav_msgs::OccupancyGrid safe_map;
    awm::makeSafeMap(map, occupied_threshold_, caution_threshold_, inflate_radius_cells_,
                     &safe_map);
    pub_safe_map_.publish(safe_map);

    if (dump_maps_) {
      awm::writeMapText(map, dumpPath("map.txt"));
      awm::writeMapText(safe_map, dumpPath("safe_map.txt"));
    }
  }

  std::string dumpPath(const std::string& name) const { return dump_dir_ + "/" + name; }

  ros::NodeHandle nh_;
  ros::Subscriber sub_map_;
  ros::Publisher pub_safe_map_;
  int occupied_threshold_;
  int caution_threshold_;
  int inflate_radius_cells_;
  bool dump_maps_;
  std::string dump_dir_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_processing");
  MapProcessing map_processing;
  ros::spin();
  return 0;
}
