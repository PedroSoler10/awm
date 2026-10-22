/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

#include <geometry_msgs/Point.h>
#include <gtest/gtest.h>
#include <nav_msgs/OccupancyGrid.h>

#include <cstdint>
#include <vector>

#include "awm/grid.h"
#include "awm/measure_map.h"
#include "awm/safe_map.h"

namespace {

// A free width x height safe map, 5 cm cells, origin at (0, 0).
nav_msgs::OccupancyGrid makeSafeMap(int width, int height) {
  nav_msgs::OccupancyGrid map;
  map.info.width = width;
  map.info.height = height;
  map.info.resolution = 0.05;
  map.data.assign(width * height, awm::kSafeFree);
  return map;
}

// 0.11 m: the 13 cells within 2 cells (see test_grid.cpp for why not 0.1).
constexpr double kRadius = 0.11;

int countValue(const nav_msgs::OccupancyGrid& grid, std::int8_t value) {
  int n = 0;
  for (std::int8_t v : grid.data) n += (v == value);
  return n;
}

TEST(MeasureMap, FirstUpdateClassifiesCells) {
  nav_msgs::OccupancyGrid safe = makeSafeMap(20, 20);
  *awm::cell(&safe, 15, 15) = awm::kSafeOccupied;
  *awm::cell(&safe, 16, 16) = awm::kSafeCaution;
  *awm::cell(&safe, 0, 19) = awm::kSafeUnknown;
  nav_msgs::OccupancyGrid measure_map;

  // Robot at cell (5, 5).
  awm::updateMeasureMap(safe, 0.25, 0.25, kRadius, &measure_map);

  ASSERT_EQ(measure_map.data.size(), safe.data.size());
  EXPECT_EQ(countValue(measure_map, awm::kMeasured), 13);  // Disk of radius 2 cells.
  EXPECT_EQ(*awm::cell(measure_map, 5, 5), awm::kMeasured);
  EXPECT_EQ(*awm::cell(measure_map, 0, 19), awm::kBlocked);
  EXPECT_EQ(*awm::cell(measure_map, 15, 15), awm::kBlocked);
  EXPECT_EQ(*awm::cell(measure_map, 16, 16), awm::kToMeasure);  // Caution: measured too.
  EXPECT_EQ(countValue(measure_map, awm::kBlocked), 2);
}

TEST(MeasureMap, CautionCellsGetMeasured) {
  nav_msgs::OccupancyGrid safe = makeSafeMap(20, 20);
  *awm::cell(&safe, 7, 5) = awm::kSafeOccupied;
  *awm::cell(&safe, 6, 5) = awm::kSafeCaution;
  nav_msgs::OccupancyGrid measure_map;
  awm::updateMeasureMap(safe, 0.25, 0.25, kRadius, &measure_map);
  EXPECT_EQ(*awm::cell(measure_map, 6, 5), awm::kMeasured);
  EXPECT_EQ(*awm::cell(measure_map, 7, 5), awm::kBlocked);  // Occupied cells never are.
}

TEST(MeasureMap, KeepsMeasuredCellsWhenGeometryIsSame) {
  const nav_msgs::OccupancyGrid safe = makeSafeMap(20, 20);
  nav_msgs::OccupancyGrid measure_map;
  awm::updateMeasureMap(safe, 0.25, 0.25, kRadius, &measure_map);
  awm::updateMeasureMap(safe, 0.75, 0.75, kRadius, &measure_map);  // Cell (15, 15).

  EXPECT_EQ(*awm::cell(measure_map, 5, 5), awm::kMeasured);
  EXPECT_EQ(*awm::cell(measure_map, 15, 15), awm::kMeasured);
  EXPECT_EQ(countValue(measure_map, awm::kMeasured), 26);
}

TEST(MeasureMap, KeepsMeasuredCellsWhenMapGrows) {
  nav_msgs::OccupancyGrid measure_map;
  awm::updateMeasureMap(makeSafeMap(20, 20), 0.25, 0.25, kRadius, &measure_map);
  ASSERT_EQ(*awm::cell(measure_map, 5, 5), awm::kMeasured);

  // The map grew by 5 cells to the left and 2 below, so the world point (0.25, 0.25) is now
  // cell (10, 7). Its free cells are 0, which must not be read as "measured".
  nav_msgs::OccupancyGrid bigger = makeSafeMap(30, 25);
  bigger.info.origin.position.x = -0.25;
  bigger.info.origin.position.y = -0.10;
  *awm::cell(&bigger, 25, 20) = awm::kSafeOccupied;
  awm::updateMeasureMap(bigger, 0.75, 0.75, kRadius, &measure_map);  // Cell (20, 17).

  EXPECT_EQ(measure_map.info.width, 30u);
  EXPECT_EQ(measure_map.info.height, 25u);
  ASSERT_EQ(measure_map.data.size(), bigger.data.size());
  EXPECT_EQ(*awm::cell(measure_map, 10, 7), awm::kMeasured);   // Carried over.
  EXPECT_EQ(*awm::cell(measure_map, 5, 5), awm::kToMeasure);   // Same index, other place.
  EXPECT_EQ(*awm::cell(measure_map, 20, 17), awm::kMeasured);  // Robot.
  EXPECT_EQ(countValue(measure_map, awm::kMeasured), 26);
  EXPECT_EQ(countValue(measure_map, awm::kBlocked), 1);
  EXPECT_EQ(countValue(measure_map, awm::kToMeasure), 30 * 25 - 26 - 1);
}

TEST(MeasureMap, OriginShiftMovesMeasuredCellsUnlessNowBlocked) {
  nav_msgs::OccupancyGrid measure_map;
  awm::updateMeasureMap(makeSafeMap(20, 20), 0.25, 0.25, kRadius, &measure_map);

  // Same size, origin 2 cells to the right: the disk around (5, 5) is now around (3, 5), whose
  // centre has become an obstacle.
  nav_msgs::OccupancyGrid shifted = makeSafeMap(20, 20);
  shifted.info.origin.position.x = 0.10;
  *awm::cell(&shifted, 3, 5) = awm::kSafeOccupied;
  awm::updateMeasureMap(shifted, 0.75, 0.75, kRadius, &measure_map);  // Cell (13, 15).

  EXPECT_EQ(*awm::cell(measure_map, 3, 5), awm::kBlocked);
  EXPECT_EQ(*awm::cell(measure_map, 4, 5), awm::kMeasured);
  EXPECT_EQ(*awm::cell(measure_map, 6, 5), awm::kToMeasure);  // Same index, other place.
  EXPECT_EQ(*awm::cell(measure_map, 13, 15), awm::kMeasured);
  EXPECT_EQ(countValue(measure_map, awm::kMeasured), 12 + 13);
  EXPECT_EQ(countValue(measure_map, awm::kBlocked), 1);
}

TEST(MeasureMap, RemoveMeasuredCellsKeepsOrderOfTheOthers) {
  nav_msgs::OccupancyGrid measure_map = makeSafeMap(4, 4);  // All 0, i.e. measured.
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

// A 30x30 room for the goal search: a wall at x = 20 with its caution band on both sides
// (x = 17..19 and 21..23), free space elsewhere. The measure map is all measured except the cells
// listed by each test.
struct Room {
  nav_msgs::OccupancyGrid safe = makeSafeMap(30, 30);
  nav_msgs::OccupancyGrid measure = makeSafeMap(30, 30);  // All 0: measured.
  Room() {
    for (int y = 0; y < 30; y++) {
      *awm::cell(&safe, 20, y) = awm::kSafeOccupied;
      *awm::cell(&measure, 20, y) = awm::kBlocked;
      for (int x = 17; x <= 23; x++) {
        if (x != 20) *awm::cell(&safe, x, y) = awm::kSafeCaution;
      }
    }
  }
};

// Robot at cell (5, 10).
constexpr double kRobotX = 0.25;
constexpr double kRobotY = 0.50;

TEST(MeasurementGoals, FreeTargetIsItsOwnGoal) {
  Room room;
  *awm::cell(&room.measure, 12, 10) = awm::kToMeasure;
  std::vector<geometry_msgs::Point> goals;
  ASSERT_TRUE(
      awm::findMeasurementGoals(room.measure, room.safe, kRobotX, kRobotY, 200, 0.5, 1, &goals));
  ASSERT_EQ(goals.size(), 1u);
  EXPECT_NEAR(goals[0].x, 0.60, 1e-6);
  EXPECT_NEAR(goals[0].y, 0.50, 1e-6);
}

TEST(MeasurementGoals, CautionTargetIsMeasuredFromNearestFreeCell) {
  Room room;
  *awm::cell(&room.measure, 18, 10) = awm::kToMeasure;  // 2 cells into the band.
  std::vector<geometry_msgs::Point> goals;
  ASSERT_TRUE(
      awm::findMeasurementGoals(room.measure, room.safe, kRobotX, kRobotY, 200, 0.5, 1, &goals));
  ASSERT_EQ(goals.size(), 1u);
  EXPECT_NEAR(goals[0].x, 0.80, 1e-6);  // Cell (16, 10): the free cell nearest to the target.
  EXPECT_NEAR(goals[0].y, 0.50, 1e-6);
}

TEST(MeasurementGoals, TargetWithoutFreeCellInReachIsSkipped) {
  Room room;
  *awm::cell(&room.measure, 19, 10) = awm::kToMeasure;  // Free space is 3 cells away.
  std::vector<geometry_msgs::Point> goals;
  EXPECT_FALSE(
      awm::findMeasurementGoals(room.measure, room.safe, kRobotX, kRobotY, 200, 0.10, 0, &goals));
  EXPECT_TRUE(goals.empty());

  // A farther target that can be measured is offered instead.
  *awm::cell(&room.measure, 5, 25) = awm::kToMeasure;
  ASSERT_TRUE(
      awm::findMeasurementGoals(room.measure, room.safe, kRobotX, kRobotY, 200, 0.10, 0, &goals));
  ASSERT_EQ(goals.size(), 1u);
  EXPECT_NEAR(goals[0].y, 1.25, 1e-6);
}

TEST(MeasurementGoals, TargetsSharingAGoalGiveItOnce) {
  Room room;
  *awm::cell(&room.measure, 18, 10) = awm::kToMeasure;
  *awm::cell(&room.measure, 19, 10) = awm::kToMeasure;  // Also measured best from (16, 10).
  *awm::cell(&room.measure, 10, 20) = awm::kToMeasure;
  std::vector<geometry_msgs::Point> goals;
  ASSERT_TRUE(
      awm::findMeasurementGoals(room.measure, room.safe, kRobotX, kRobotY, 200, 0.5, 0, &goals));
  ASSERT_EQ(goals.size(), 2u);
  EXPECT_NEAR(goals[0].x, 0.50, 1e-6);  // (10, 20) is the nearest target to the robot.
  EXPECT_NEAR(goals[1].x, 0.80, 1e-6);

  ASSERT_TRUE(
      awm::findMeasurementGoals(room.measure, room.safe, kRobotX, kRobotY, 200, 0.5, 1, &goals));
  EXPECT_EQ(goals.size(), 1u);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
