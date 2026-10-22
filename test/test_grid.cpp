/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

#include <geometry_msgs/Pose.h>
#include <gtest/gtest.h>
#include <nav_msgs/OccupancyGrid.h>

#include <cstdint>
#include <vector>

#include "awm/grid.h"

namespace {

// A width x height grid with 5 cm cells and its origin at (-1, -2), filled with value.
nav_msgs::OccupancyGrid makeGrid(int width, int height, std::int8_t value) {
  nav_msgs::OccupancyGrid grid;
  grid.info.width = width;
  grid.info.height = height;
  grid.info.resolution = 0.05;
  grid.info.origin.position.x = -1.0;
  grid.info.origin.position.y = -2.0;
  grid.data.assign(width * height, value);
  return grid;
}

int countValue(const nav_msgs::OccupancyGrid& grid, std::int8_t value) {
  int n = 0;
  for (std::int8_t v : grid.data) n += (v == value);
  return n;
}

TEST(Grid, WorldToGridRoundTrip) {
  const nav_msgs::OccupancyGrid grid = makeGrid(40, 30, 0);
  for (int y = 0; y < 30; y += 7) {
    for (int x = 0; x < 40; x += 9) {
      geometry_msgs::Pose pose;
      awm::gridToWorld(grid, x, y, &pose.position.x, &pose.position.y);
      int gx;
      int gy;
      awm::worldToGrid(grid, pose, &gx, &gy);
      EXPECT_EQ(gx, x);
      EXPECT_EQ(gy, y);
    }
  }
}

TEST(Grid, WorldToGridRoundsToNearestCell) {
  const nav_msgs::OccupancyGrid grid = makeGrid(40, 30, 0);
  geometry_msgs::Pose pose;
  pose.position.x = -1.0 + 3 * 0.05 + 0.02;  // Closer to cell 3 than to cell 4.
  pose.position.y = -2.0 + 5 * 0.05 + 0.03;  // Closer to cell 6 than to cell 5.
  int x;
  int y;
  awm::worldToGrid(grid, pose, &x, &y);
  EXPECT_EQ(x, 3);
  EXPECT_EQ(y, 6);
}

TEST(Grid, CellOutOfBoundsIsNull) {
  nav_msgs::OccupancyGrid grid = makeGrid(4, 3, 0);
  const nav_msgs::OccupancyGrid& const_grid = grid;
  EXPECT_EQ(awm::cell(&grid, -1, 0), nullptr);
  EXPECT_EQ(awm::cell(&grid, 0, -1), nullptr);
  EXPECT_EQ(awm::cell(&grid, 4, 0), nullptr);
  EXPECT_EQ(awm::cell(&grid, 0, 3), nullptr);
  EXPECT_EQ(awm::cell(const_grid, 4, 2), nullptr);
  EXPECT_EQ(awm::cell(const_grid, 3, 3), nullptr);

  // In bounds: (x, y) is data[y * width + x], in both versions.
  EXPECT_EQ(awm::cell(&grid, 3, 2), &grid.data[2 * 4 + 3]);
  EXPECT_EQ(awm::cell(const_grid, 1, 2), &grid.data[2 * 4 + 1]);
}

TEST(Grid, MarkDiskRadius) {
  nav_msgs::OccupancyGrid grid = makeGrid(41, 41, -1);
  // 0.11 m at 5 cm per cell is 2.2 cells: the disk is the 13 cells with x^2 + y^2 <= 4. Not exactly
  // 0.1 m, because resolution is a float32 slightly above 0.05 and the 2-cell ring would fall out.
  awm::markDisk(&grid, 20, 20, 0.11, 0, 100);
  EXPECT_EQ(countValue(grid, 0), 13);
  EXPECT_EQ(*awm::cell(grid, 20, 20), 0);
  EXPECT_EQ(*awm::cell(grid, 22, 20), 0);
  EXPECT_EQ(*awm::cell(grid, 20, 18), 0);
  EXPECT_EQ(*awm::cell(grid, 21, 21), 0);
  EXPECT_EQ(*awm::cell(grid, 22, 21), -1);  // sqrt(5) cells > 2 cells.
  EXPECT_EQ(*awm::cell(grid, 23, 20), -1);
}

TEST(Grid, MarkDiskKeepsValueAndClipsAtBorder) {
  nav_msgs::OccupancyGrid grid = makeGrid(10, 10, -1);
  *awm::cell(&grid, 1, 0) = 100;
  awm::markDisk(&grid, 0, 0, 0.11, 0, 100);  // Centre in the corner: only a quarter fits.
  EXPECT_EQ(*awm::cell(grid, 1, 0), 100);
  EXPECT_EQ(countValue(grid, 0), 5);  // (0,0) (2,0) (0,1) (1,1) (0,2).
}

TEST(Grid, InflateKeepsMeasuredCells) {
  nav_msgs::OccupancyGrid source = makeGrid(9, 9, 0);
  *awm::cell(&source, 4, 4) = 100;
  nav_msgs::OccupancyGrid target = makeGrid(9, 9, -1);
  *awm::cell(&target, 5, 5) = 0;  // Measured, inside the margin.
  *awm::cell(&target, 7, 7) = 0;  // Measured, outside the margin.

  awm::inflate(source, 90, 2, &target, 0);

  EXPECT_EQ(*awm::cell(target, 5, 5), 0);
  EXPECT_EQ(*awm::cell(target, 7, 7), 0);
  EXPECT_EQ(*awm::cell(target, 2, 2), 100);  // Corner of the 5x5 square.
  EXPECT_EQ(*awm::cell(target, 6, 4), 100);
  EXPECT_EQ(*awm::cell(target, 1, 4), -1);  // Just outside.
  EXPECT_EQ(countValue(target, 100), 24);   // 25 cells minus the measured one.
}

TEST(Grid, InflateIgnoresCellsAtThreshold) {
  nav_msgs::OccupancyGrid source = makeGrid(5, 5, 90);
  nav_msgs::OccupancyGrid target = makeGrid(5, 5, -1);
  awm::inflate(source, 90, 1, &target, 0);
  EXPECT_EQ(countValue(target, 100), 0);
}

TEST(Grid, FindNearestCellReturnsClosest) {
  nav_msgs::OccupancyGrid grid = makeGrid(30, 30, 100);
  *awm::cell(&grid, 20, 10) = -1;  // 10 cells away.
  *awm::cell(&grid, 10, 15) = -1;  // 5 cells away, at 90 degrees.
  int x;
  int y;
  int angle;
  ASSERT_TRUE(awm::findNearestCell(grid, 10, 10, -1, 200, &x, &y, &angle));
  EXPECT_EQ(x, 10);
  EXPECT_EQ(y, 15);
  // Rounding to cells makes a few angles near 90 land on the same cell; the first one wins.
  EXPECT_NEAR(angle, 90, 5);
}

TEST(Grid, FindNearestCellUsesSweepOrder) {
  // Two cells at the same distance: the one at the smaller angle comes first.
  nav_msgs::OccupancyGrid grid = makeGrid(30, 30, 100);
  *awm::cell(&grid, 7, 10) = -1;  // 180 degrees.
  *awm::cell(&grid, 10, 7) = -1;  // 270 degrees.
  int x;
  int y;
  int angle;
  ASSERT_TRUE(awm::findNearestCell(grid, 10, 10, -1, 200, &x, &y, &angle));
  EXPECT_EQ(x, 7);
  EXPECT_EQ(y, 10);
  EXPECT_NEAR(angle, 180, 10);
}

TEST(Grid, FindNearestCellAtDistanceZero) {
  nav_msgs::OccupancyGrid grid = makeGrid(5, 5, -1);
  int x;
  int y;
  int angle;
  ASSERT_TRUE(awm::findNearestCell(grid, 2, 3, -1, 200, &x, &y, &angle));
  EXPECT_EQ(x, 2);
  EXPECT_EQ(y, 3);
  EXPECT_EQ(angle, 0);
}

TEST(Grid, FindNearestCellFalseWhenNone) {
  nav_msgs::OccupancyGrid grid = makeGrid(30, 30, 100);
  int x = -7;
  int y = -7;
  int angle = -7;
  EXPECT_FALSE(awm::findNearestCell(grid, 10, 10, -1, 200, &x, &y, &angle));
  EXPECT_EQ(x, -7);  // Outputs untouched.

  // A match beyond the search limit is not found either.
  *awm::cell(&grid, 25, 10) = -1;
  EXPECT_FALSE(awm::findNearestCell(grid, 10, 10, -1, 14, &x, &y, &angle));
  EXPECT_TRUE(awm::findNearestCell(grid, 10, 10, -1, 15, &x, &y, &angle));
}

TEST(Grid, FindNearestCellsReturnsClosestFirstWithoutDuplicates) {
  nav_msgs::OccupancyGrid grid = makeGrid(30, 30, 100);
  *awm::cell(&grid, 10, 10) = -1;  // The start cell itself: found at d = 0 for every angle.
  *awm::cell(&grid, 10, 15) = -1;  // 5 cells away.
  *awm::cell(&grid, 20, 10) = -1;  // 10 cells away.
  std::vector<awm::GridCell> cells;
  std::vector<int> angles;
  ASSERT_TRUE(awm::findNearestCells(grid, 10, 10, -1, 200, 0, &cells, &angles));
  ASSERT_EQ(cells.size(), 3u);
  ASSERT_EQ(angles.size(), 3u);
  EXPECT_EQ(cells[0].x, 10);
  EXPECT_EQ(cells[0].y, 10);
  EXPECT_EQ(cells[1].x, 10);
  EXPECT_EQ(cells[1].y, 15);
  EXPECT_EQ(cells[2].x, 20);
  EXPECT_EQ(cells[2].y, 10);
}

TEST(Grid, FindNearestCellsStopsAtN) {
  nav_msgs::OccupancyGrid grid = makeGrid(30, 30, -1);
  std::vector<awm::GridCell> cells;
  ASSERT_TRUE(awm::findNearestCells(grid, 15, 15, -1, 200, 5, &cells));
  EXPECT_EQ(cells.size(), 5u);
  ASSERT_TRUE(awm::findNearestCells(grid, 15, 15, -1, 200, 1, &cells));
  EXPECT_EQ(cells.size(), 1u);  // Cleared first.
}

TEST(Grid, FindNearestCellsFalseWhenNone) {
  nav_msgs::OccupancyGrid grid = makeGrid(30, 30, 100);
  std::vector<awm::GridCell> cells = {{1, 1}};
  EXPECT_FALSE(awm::findNearestCells(grid, 10, 10, -1, 200, 3, &cells));
  EXPECT_TRUE(cells.empty());
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
