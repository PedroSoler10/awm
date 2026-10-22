/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

#include <gtest/gtest.h>
#include <nav_msgs/OccupancyGrid.h>

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include "awm/grid.h"
#include "awm/map_dump.h"
#include "awm/safe_map.h"

namespace {

constexpr int kOccupied = 90;
constexpr int kCaution = 50;

// A 10x10 grid of free cells, 5 cm each, with one obstacle at (5, 5).
nav_msgs::OccupancyGrid makeMap() {
  nav_msgs::OccupancyGrid map;
  map.info.width = 10;
  map.info.height = 10;
  map.info.resolution = 0.05;
  map.data.assign(100, 0);
  *awm::cell(&map, 5, 5) = 100;
  return map;
}

std::int8_t at(const nav_msgs::OccupancyGrid& grid, int x, int y) { return *awm::cell(grid, x, y); }

}  // namespace

TEST(SafeMap, DiscretisesValues) {
  nav_msgs::OccupancyGrid map = makeMap();
  *awm::cell(&map, 0, 0) = 89;  // free but not an obstacle
  *awm::cell(&map, 9, 0) = 60;  // uncertain
  *awm::cell(&map, 0, 9) = -1;  // unknown
  *awm::cell(&map, 9, 9) = 90;  // exactly the threshold is occupied
  nav_msgs::OccupancyGrid safe;
  awm::makeSafeMap(map, kOccupied, kCaution, 0, &safe);
  EXPECT_EQ(50, at(safe, 0, 0));
  EXPECT_EQ(50, at(safe, 9, 0));
  EXPECT_EQ(-1, at(safe, 0, 9));
  EXPECT_EQ(100, at(safe, 9, 9));
  EXPECT_EQ(100, at(safe, 5, 5));
  EXPECT_EQ(0, at(safe, 4, 4));
}

TEST(SafeMap, KeepsHeaderAndSize) {
  nav_msgs::OccupancyGrid map = makeMap();
  map.header.frame_id = "map";
  nav_msgs::OccupancyGrid safe;
  awm::makeSafeMap(map, kOccupied, kCaution, 2, &safe);
  EXPECT_EQ("map", safe.header.frame_id);
  EXPECT_EQ(map.info.width, safe.info.width);
  EXPECT_EQ(map.data.size(), safe.data.size());
}

TEST(SafeMap, CautionBandAroundObstacle) {
  nav_msgs::OccupancyGrid safe;
  awm::makeSafeMap(makeMap(), kOccupied, kCaution, 2, &safe);
  for (int y = 3; y <= 7; y++) {
    for (int x = 3; x <= 7; x++) {
      if (x == 5 && y == 5) continue;
      EXPECT_EQ(50, at(safe, x, y)) << x << "," << y;
    }
  }
  EXPECT_EQ(100, at(safe, 5, 5));  // the obstacle itself is not turned into caution
  EXPECT_EQ(0, at(safe, 2, 5));
  EXPECT_EQ(0, at(safe, 8, 5));
  EXPECT_EQ(0, at(safe, 5, 2));
  EXPECT_EQ(0, at(safe, 5, 8));
}

TEST(SafeMap, BandReachesTheBorders) {
  nav_msgs::OccupancyGrid map = makeMap();
  *awm::cell(&map, 0, 0) = 100;
  *awm::cell(&map, 9, 9) = 100;
  nav_msgs::OccupancyGrid safe;
  awm::makeSafeMap(map, kOccupied, kCaution, 1, &safe);
  // Row 0 and column 0 are inside the grid: the old "> 0" test skipped them.
  EXPECT_EQ(50, at(safe, 1, 0));
  EXPECT_EQ(50, at(safe, 0, 1));
  EXPECT_EQ(50, at(safe, 1, 1));
  EXPECT_EQ(50, at(safe, 8, 9));
  EXPECT_EQ(50, at(safe, 9, 8));
}

TEST(SafeMap, UnknownStaysUnknownNextToObstacle) {
  nav_msgs::OccupancyGrid map = makeMap();
  *awm::cell(&map, 6, 5) = -1;
  nav_msgs::OccupancyGrid safe;
  awm::makeSafeMap(map, kOccupied, kCaution, 2, &safe);
  EXPECT_EQ(-1, at(safe, 6, 5));
  EXPECT_EQ(50, at(safe, 6, 6));
}

TEST(SafeMap, BandIsNotChainedFromCautionCells) {
  nav_msgs::OccupancyGrid safe;
  awm::makeSafeMap(makeMap(), kOccupied, kCaution, 1, &safe);
  EXPECT_EQ(50, at(safe, 4, 4));
  EXPECT_EQ(0, at(safe, 3, 3));  // one step further would mean the band grew from itself
}

TEST(SafeMap, EmptyMapIsFine) {
  nav_msgs::OccupancyGrid map;
  nav_msgs::OccupancyGrid safe;
  awm::makeSafeMap(map, kOccupied, kCaution, 6, &safe);
  EXPECT_TRUE(safe.data.empty());
}

TEST(MapDump, WritesSymbolsTopRowFirst) {
  nav_msgs::OccupancyGrid map;
  map.info.width = 3;
  map.info.height = 2;
  map.data = {0,  50, 100,  // y = 0 (bottom row of the picture)
              -1, 0,  0};   // y = 1 (top row)
  const std::string path = ::testing::TempDir() + "awm_map_dump_test.txt";
  ASSERT_TRUE(awm::writeMapText(map, path));
  std::ifstream file(path);
  std::stringstream text;
  text << file.rdbuf();
  EXPECT_EQ("·\t \t \t\n \t+\t*\t\n", text.str());
  std::remove(path.c_str());
}

TEST(MapDump, DrawsMarksOverCells) {
  nav_msgs::OccupancyGrid map;
  map.info.width = 3;
  map.info.height = 2;
  map.data = {0, 50, 100, -1, 0, 0};
  const std::string path = ::testing::TempDir() + "awm_map_dump_marks_test.txt";
  // (2, 0) is occupied and (0, 1) unknown: the mark wins. (7, 7) is outside and ignored.
  ASSERT_TRUE(awm::writeMapText(map, path, {{2, 0}, {0, 1}, {7, 7}}));
  std::ifstream file(path);
  std::stringstream text;
  text << file.rdbuf();
  EXPECT_EQ("8\t \t \t\n \t+\t8\t\n", text.str());
  std::remove(path.c_str());
}

TEST(MapDump, FailsWhenDataDoesNotMatchSize) {
  nav_msgs::OccupancyGrid map;
  map.info.width = 3;
  map.info.height = 2;
  map.data = {0, 50, 100};
  EXPECT_FALSE(awm::writeMapText(map, ::testing::TempDir() + "awm_map_dump_short_test.txt"));
}

TEST(MapDump, FailsOnBadPath) {
  EXPECT_FALSE(awm::writeMapText(nav_msgs::OccupancyGrid(), "/nonexistent_dir/x.txt"));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
