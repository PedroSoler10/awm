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

// The first cell findNearestCells finds (n = 1), with its angle. Outputs untouched if none.
bool findNearest(const nav_msgs::OccupancyGrid& grid, int x0, int y0, std::int8_t value,
                 int max_radius_cells, int* x, int* y, int* angle_deg) {
  std::vector<awm::GridCell> cells;
  std::vector<int> angles;
  if (!awm::findNearestCells(grid, x0, y0, value, max_radius_cells, 1, &cells, &angles))
    return false;
  *x = cells[0].x;
  *y = cells[0].y;
  *angle_deg = angles[0];
  return true;
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

TEST(Grid, CellBeyondShortDataIsNull) {
  // info says 4 x 3, but data only holds the first row and a half.
  nav_msgs::OccupancyGrid grid = makeGrid(4, 3, 0);
  grid.data.resize(6);
  EXPECT_EQ(awm::cell(&grid, 1, 1), &grid.data[5]);
  EXPECT_EQ(awm::cell(&grid, 2, 1), nullptr);
  EXPECT_EQ(awm::cell(static_cast<const nav_msgs::OccupancyGrid&>(grid), 3, 2), nullptr);
}

TEST(Grid, SameGeometry) {
  nav_msgs::OccupancyGrid a = makeGrid(4, 3, 0);
  nav_msgs::OccupancyGrid b = makeGrid(4, 3, -1);  // Values do not matter.
  EXPECT_TRUE(awm::sameGeometry(a.info, b.info));
  b.info.origin.position.y = 0.05;
  EXPECT_FALSE(awm::sameGeometry(a.info, b.info));
  EXPECT_FALSE(awm::sameGeometry(a.info, makeGrid(5, 3, 0).info));
}

TEST(Grid, HasFullData) {
  nav_msgs::OccupancyGrid grid = makeGrid(4, 3, 0);
  EXPECT_TRUE(awm::hasFullData(grid));
  grid.data.resize(11);
  EXPECT_FALSE(awm::hasFullData(grid));
  EXPECT_TRUE(awm::hasFullData(nav_msgs::OccupancyGrid()));  // 0 x 0, no data.
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

TEST(Grid, FindNearestCellReturnsClosest) {
  nav_msgs::OccupancyGrid grid = makeGrid(30, 30, 100);
  *awm::cell(&grid, 20, 10) = -1;  // 10 cells away.
  *awm::cell(&grid, 10, 15) = -1;  // 5 cells away, at 90 degrees.
  int x;
  int y;
  int angle;
  ASSERT_TRUE(findNearest(grid, 10, 10, -1, 200, &x, &y, &angle));
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
  ASSERT_TRUE(findNearest(grid, 10, 10, -1, 200, &x, &y, &angle));
  EXPECT_EQ(x, 7);
  EXPECT_EQ(y, 10);
  EXPECT_NEAR(angle, 180, 10);
}

TEST(Grid, FindNearestCellAtDistanceZero) {
  nav_msgs::OccupancyGrid grid = makeGrid(5, 5, -1);
  int x;
  int y;
  int angle;
  ASSERT_TRUE(findNearest(grid, 2, 3, -1, 200, &x, &y, &angle));
  EXPECT_EQ(x, 2);
  EXPECT_EQ(y, 3);
  EXPECT_EQ(angle, 0);
}

TEST(Grid, FindNearestCellFalseWhenNone) {
  nav_msgs::OccupancyGrid grid = makeGrid(30, 30, 100);
  int x = -7;
  int y = -7;
  int angle = -7;
  EXPECT_FALSE(findNearest(grid, 10, 10, -1, 200, &x, &y, &angle));
  EXPECT_EQ(x, -7);  // Outputs untouched.

  // A match beyond the search limit is not found either.
  *awm::cell(&grid, 25, 10) = -1;
  EXPECT_FALSE(findNearest(grid, 10, 10, -1, 14, &x, &y, &angle));
  EXPECT_TRUE(findNearest(grid, 10, 10, -1, 15, &x, &y, &angle));
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
