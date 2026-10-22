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

constexpr int kOccupied = 80;
constexpr double kMargin = 0.40;  // 8 cells of 5 cm.

// A 21x21 grid of free cells, 5 cm each, with one obstacle at the centre (10, 10).
nav_msgs::OccupancyGrid makeMap() {
  nav_msgs::OccupancyGrid map;
  map.info.width = 21;
  map.info.height = 21;
  map.info.resolution = 0.05;
  map.data.assign(21 * 21, 0);
  *awm::cell(&map, 10, 10) = 100;
  return map;
}

std::int8_t at(const nav_msgs::OccupancyGrid& grid, int x, int y) { return *awm::cell(grid, x, y); }

}  // namespace

TEST(SafeMap, DiscretisesValues) {
  nav_msgs::OccupancyGrid map = makeMap();
  *awm::cell(&map, 0, 0) = 79;    // Below the threshold: free, so it will be measured.
  *awm::cell(&map, 20, 0) = 80;   // Exactly the threshold: occupied.
  *awm::cell(&map, 0, 20) = -1;   // Unknown.
  *awm::cell(&map, 20, 20) = 55;  // Uncertain, still free.
  nav_msgs::OccupancyGrid safe;
  awm::makeSafeMap(map, kOccupied, 0.0, &safe);  // No margin.
  EXPECT_EQ(awm::kSafeFree, at(safe, 0, 0));
  EXPECT_EQ(awm::kSafeOccupied, at(safe, 20, 0));
  EXPECT_EQ(awm::kSafeUnknown, at(safe, 0, 20));
  EXPECT_EQ(awm::kSafeFree, at(safe, 20, 20));
  EXPECT_EQ(awm::kSafeOccupied, at(safe, 10, 10));
  EXPECT_EQ(awm::kSafeFree, at(safe, 9, 10));
}

TEST(SafeMap, KeepsHeaderAndSize) {
  nav_msgs::OccupancyGrid map = makeMap();
  map.header.frame_id = "map";
  nav_msgs::OccupancyGrid safe;
  awm::makeSafeMap(map, kOccupied, kMargin, &safe);
  EXPECT_EQ("map", safe.header.frame_id);
  EXPECT_EQ(map.info.width, safe.info.width);
  EXPECT_EQ(map.data.size(), safe.data.size());
}

TEST(SafeMap, MarginIsARoundDistance) {
  nav_msgs::OccupancyGrid safe;
  awm::makeSafeMap(makeMap(), kOccupied, kMargin, &safe);
  EXPECT_EQ(awm::kSafeCaution, at(safe, 17, 10));  // 0.35 m.
  EXPECT_EQ(awm::kSafeFree, at(safe, 18, 10));     // Exactly 0.40 m: allowed.
  EXPECT_EQ(awm::kSafeFree, at(safe, 10, 2));
  EXPECT_EQ(awm::kSafeCaution, at(safe, 15, 15));  // Diagonal 0.354 m.
  EXPECT_EQ(awm::kSafeCaution, at(safe, 15, 16));  // 0.391 m.
  EXPECT_EQ(awm::kSafeFree, at(safe, 16, 16));     // 0.424 m: a square margin would block it.
  EXPECT_EQ(awm::kSafeOccupied, at(safe, 10, 10));
}

TEST(SafeMap, BandReachesTheBorders) {
  nav_msgs::OccupancyGrid map = makeMap();
  *awm::cell(&map, 0, 0) = 100;
  nav_msgs::OccupancyGrid safe;
  awm::makeSafeMap(map, kOccupied, 0.10, &safe);
  EXPECT_EQ(awm::kSafeCaution, at(safe, 1, 0));
  EXPECT_EQ(awm::kSafeCaution, at(safe, 0, 1));
  EXPECT_EQ(awm::kSafeCaution, at(safe, 1, 1));
  EXPECT_EQ(awm::kSafeFree, at(safe, 2, 0));  // Exactly 0.10 m.
}

TEST(SafeMap, UnknownStaysUnknownNextToObstacle) {
  nav_msgs::OccupancyGrid map = makeMap();
  *awm::cell(&map, 11, 10) = -1;
  nav_msgs::OccupancyGrid safe;
  awm::makeSafeMap(map, kOccupied, kMargin, &safe);
  EXPECT_EQ(awm::kSafeUnknown, at(safe, 11, 10));
  EXPECT_EQ(awm::kSafeCaution, at(safe, 11, 11));
}

TEST(SafeMap, BandIsNotChainedFromCautionCells) {
  nav_msgs::OccupancyGrid safe;
  awm::makeSafeMap(makeMap(), kOccupied, 0.06, &safe);
  EXPECT_EQ(awm::kSafeCaution, at(safe, 11, 10));
  EXPECT_EQ(awm::kSafeFree, at(safe, 12, 10));  // One step further would mean the band grew.
}

TEST(SafeMap, EmptyMapIsFine) {
  nav_msgs::OccupancyGrid map;
  nav_msgs::OccupancyGrid safe;
  awm::makeSafeMap(map, kOccupied, kMargin, &safe);
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
