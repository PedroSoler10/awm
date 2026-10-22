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
#include <vector>

#include "awm/grid.h"
#include "awm/measure_map.h"

namespace {

// A free width x height SLAM map, 5 cm cells, origin at (0, 0).
nav_msgs::OccupancyGrid makeMap(int width, int height) {
  nav_msgs::OccupancyGrid map;
  map.info.width = width;
  map.info.height = height;
  map.info.resolution = 0.05;
  map.data.assign(width * height, 0);
  return map;
}

// Small values to keep the counts easy: a 0.11 m measure radius (the 13 cells within 2 cells; see
// test_grid.cpp for why not 0.1) and a 1-cell margin.
awm::MeasureMapParams testParams() {
  awm::MeasureMapParams params;
  params.measure_radius_m = 0.11;
  params.inflate_radius_cells = 1;
  params.occupied_threshold = 90;
  return params;
}

int countValue(const nav_msgs::OccupancyGrid& grid, std::int8_t value) {
  int n = 0;
  for (std::int8_t v : grid.data) n += (v == value);
  return n;
}

TEST(MeasureMap, FirstUpdateClassifiesCells) {
  nav_msgs::OccupancyGrid map = makeMap(20, 20);
  *awm::cell(&map, 15, 15) = 100;  // Obstacle.
  *awm::cell(&map, 0, 19) = -1;    // Unknown.
  nav_msgs::OccupancyGrid measure_map;

  // Robot at cell (5, 5).
  awm::updateMeasureMap(map, 0.25, 0.25, testParams(), &measure_map);

  ASSERT_EQ(measure_map.data.size(), map.data.size());
  EXPECT_EQ(countValue(measure_map, awm::kMeasured), 13);  // Disk of radius 2 cells.
  EXPECT_EQ(*awm::cell(measure_map, 5, 5), awm::kMeasured);
  EXPECT_EQ(*awm::cell(measure_map, 0, 19), awm::kBlocked);
  EXPECT_EQ(*awm::cell(measure_map, 15, 15), awm::kBlocked);
  EXPECT_EQ(*awm::cell(measure_map, 16, 16), awm::kBlocked);  // Margin.
  EXPECT_EQ(*awm::cell(measure_map, 17, 15), awm::kToMeasure);
  EXPECT_EQ(countValue(measure_map, awm::kBlocked), 1 + 9);
}

TEST(MeasureMap, KeepsMeasuredCellsWhenSizeIsSame) {
  const nav_msgs::OccupancyGrid map = makeMap(20, 20);
  nav_msgs::OccupancyGrid measure_map;
  awm::updateMeasureMap(map, 0.25, 0.25, testParams(), &measure_map);
  awm::updateMeasureMap(map, 0.75, 0.75, testParams(), &measure_map);  // Cell (15, 15).

  EXPECT_EQ(*awm::cell(measure_map, 5, 5), awm::kMeasured);
  EXPECT_EQ(*awm::cell(measure_map, 15, 15), awm::kMeasured);
  EXPECT_EQ(countValue(measure_map, awm::kMeasured), 26);
}

TEST(MeasureMap, ResetsAfterResize) {
  nav_msgs::OccupancyGrid measure_map;
  awm::updateMeasureMap(makeMap(20, 20), 0.25, 0.25, testParams(), &measure_map);
  ASSERT_EQ(*awm::cell(measure_map, 5, 5), awm::kMeasured);

  // The SLAM map grew. Its free cells are 0, which must not be read as "measured".
  nav_msgs::OccupancyGrid bigger = makeMap(30, 25);
  *awm::cell(&bigger, 20, 20) = 100;
  awm::updateMeasureMap(bigger, 0.75, 0.75, testParams(), &measure_map);  // Cell (15, 15).

  EXPECT_EQ(measure_map.info.width, 30u);
  EXPECT_EQ(measure_map.info.height, 25u);
  ASSERT_EQ(measure_map.data.size(), bigger.data.size());
  EXPECT_EQ(*awm::cell(measure_map, 5, 5), awm::kToMeasure);  // Forgotten.
  EXPECT_EQ(*awm::cell(measure_map, 15, 15), awm::kMeasured);
  EXPECT_EQ(countValue(measure_map, awm::kMeasured), 13);
  EXPECT_EQ(countValue(measure_map, awm::kBlocked), 9);
  EXPECT_EQ(countValue(measure_map, awm::kToMeasure), 30 * 25 - 13 - 9);
}

TEST(MeasureMap, MarginDoesNotCoverMeasuredCells) {
  nav_msgs::OccupancyGrid map = makeMap(20, 20);
  nav_msgs::OccupancyGrid measure_map;
  awm::updateMeasureMap(map, 0.25, 0.25, testParams(), &measure_map);

  // An obstacle appears next to the measured disk.
  *awm::cell(&map, 8, 5) = 100;
  awm::updateMeasureMap(map, 0.75, 0.75, testParams(), &measure_map);
  EXPECT_EQ(*awm::cell(measure_map, 7, 5), awm::kMeasured);
  EXPECT_EQ(*awm::cell(measure_map, 8, 5), awm::kBlocked);
  EXPECT_EQ(*awm::cell(measure_map, 9, 5), awm::kBlocked);
}

TEST(MeasureMap, RemoveMeasuredCellsKeepsOrderOfTheOthers) {
  nav_msgs::OccupancyGrid measure_map = makeMap(4, 4);  // All 0, i.e. measured.
  *awm::cell(&measure_map, 1, 1) = awm::kToMeasure;
  *awm::cell(&measure_map, 3, 0) = awm::kBlocked;
  *awm::cell(&measure_map, 2, 2) = awm::kToMeasure;
  // (0, 0) is measured and goes; (9, 9) is outside the map and stays.
  std::vector<awm::GridCell> cells = {{2, 2}, {0, 0}, {3, 0}, {9, 9}, {1, 1}};
  awm::removeMeasuredCells(measure_map, &cells);
  ASSERT_EQ(cells.size(), 4u);
  EXPECT_EQ(cells[0].x, 2);
  EXPECT_EQ(cells[1].x, 3);
  EXPECT_EQ(cells[2].x, 9);
  EXPECT_EQ(cells[3].x, 1);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
