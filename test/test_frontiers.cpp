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

#include "awm/frontiers.h"
#include "awm/grid.h"

namespace {

nav_msgs::OccupancyGrid makeGrid(int width, int height, std::int8_t value) {
  nav_msgs::OccupancyGrid grid;
  grid.info.width = width;
  grid.info.height = height;
  grid.info.resolution = 0.05;
  grid.data.assign(width * height, value);
  return grid;
}

// A safe map with free space for x < 10 and unknown for x >= 10: the frontier is the column x = 9.
nav_msgs::OccupancyGrid makeHalfKnownMap() {
  nav_msgs::OccupancyGrid map = makeGrid(20, 20, 0);
  for (int y = 0; y < 20; y++) {
    for (int x = 10; x < 20; x++) *awm::cell(&map, x, y) = -1;
  }
  return map;
}

bool contains(const std::vector<awm::GridCell>& cells, int x, int y) {
  for (const awm::GridCell& c : cells) {
    if (c.x == x && c.y == y) return true;
  }
  return false;
}

}  // namespace

TEST(Frontiers, KnownUnknownBoundaryIsOneCluster) {
  std::vector<awm::GridCell> cells;
  ASSERT_TRUE(awm::findFrontierCells(makeHalfKnownMap(), &cells));
  EXPECT_EQ(cells.size(), 20u);  // The whole column x = 9, corners included.
  for (const awm::GridCell& c : cells) EXPECT_EQ(c.x, 9);

  std::vector<std::vector<awm::GridCell>> clusters;
  awm::clusterFrontiers(cells, 20, 3, &clusters);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters[0].size(), 20u);
}

TEST(Frontiers, DiagonalUnknownNeighbourCounts) {
  // A free cell that touches unknown only at a corner is a frontier: the check is 3x3.
  nav_msgs::OccupancyGrid safe = makeGrid(5, 5, 0);
  *awm::cell(&safe, 3, 3) = -1;
  std::vector<awm::GridCell> cells;
  ASSERT_TRUE(awm::findFrontierCells(safe, &cells));
  EXPECT_EQ(cells.size(), 8u);  // The 8 cells around (3, 3), diagonals included.
  EXPECT_TRUE(contains(cells, 2, 2));
  EXPECT_TRUE(contains(cells, 4, 4));
  EXPECT_FALSE(contains(cells, 3, 3));  // The unknown cell itself is not free.
}

TEST(Frontiers, OnlyFreeCells) {
  nav_msgs::OccupancyGrid safe = makeHalfKnownMap();
  *awm::cell(&safe, 9, 4) = 100;  // Obstacle.
  *awm::cell(&safe, 9, 5) = 50;   // In the caution band.
  std::vector<awm::GridCell> cells;
  ASSERT_TRUE(awm::findFrontierCells(safe, &cells));
  EXPECT_EQ(cells.size(), 18u);
  EXPECT_FALSE(contains(cells, 9, 4));
  EXPECT_FALSE(contains(cells, 9, 5));
  EXPECT_TRUE(contains(cells, 9, 6));
}

TEST(Frontiers, FullyKnownMapHasNone) {
  std::vector<awm::GridCell> cells = {{1, 1}};
  EXPECT_TRUE(awm::findFrontierCells(makeGrid(8, 8, 0), &cells));
  EXPECT_TRUE(cells.empty());
}

TEST(Frontiers, ShortDataIsRejected) {
  nav_msgs::OccupancyGrid safe = makeGrid(8, 8, 0);
  safe.data.resize(60);
  std::vector<awm::GridCell> cells = {{1, 1}};
  EXPECT_FALSE(awm::findFrontierCells(safe, &cells));
  EXPECT_TRUE(cells.empty());
}

TEST(Frontiers, SmallClusterIsFilteredOut) {
  // A long frontier (10 cells) and a short one (2 cells), far apart.
  std::vector<awm::GridCell> cells;
  for (int y = 0; y < 10; y++) cells.push_back({1, y});
  cells.push_back({10, 3});
  cells.push_back({10, 4});
  std::vector<std::vector<awm::GridCell>> clusters;
  awm::clusterFrontiers(cells, 20, 5, &clusters);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters[0].size(), 10u);
  awm::clusterFrontiers(cells, 20, 2, &clusters);
  EXPECT_EQ(clusters.size(), 2u);
  awm::clusterFrontiers(cells, 20, 11, &clusters);
  EXPECT_TRUE(clusters.empty());
}

TEST(Frontiers, ClustersJoinDiagonallyButNotAcrossTheGridEdge) {
  // (1,4)-(2,5)-(3,6) touch at corners: one cluster. (4,1) is the last column of row 1 and (0,2) is
  // the first column of row 2: next to each other in memory, but 4 columns apart on the grid.
  const std::vector<awm::GridCell> cells = {{1, 4}, {2, 5}, {3, 6}, {4, 1}, {0, 2}};
  std::vector<std::vector<awm::GridCell>> clusters;
  awm::clusterFrontiers(cells, 5, 1, &clusters);
  ASSERT_EQ(clusters.size(), 3u);
  EXPECT_EQ(clusters[0].size(), 3u);
  EXPECT_EQ(clusters[1].size(), 1u);
  EXPECT_EQ(clusters[2].size(), 1u);
}

TEST(Frontiers, NoCellsNoClusters) {
  std::vector<std::vector<awm::GridCell>> clusters(1);
  awm::clusterFrontiers({}, 10, 1, &clusters);
  EXPECT_TRUE(clusters.empty());
}

TEST(Frontiers, GoalIsClusterCellNearestToCentroid) {
  // An L: its centroid lies off the shape, so the goal must be picked among the real cells.
  std::vector<std::vector<awm::GridCell>> clusters(1);
  for (int y = 0; y < 5; y++) clusters[0].push_back({0, y});
  for (int x = 1; x < 5; x++) clusters[0].push_back({x, 0});
  std::vector<awm::GridCell> goals;
  awm::frontierGoals(clusters, 0, 0, &goals);
  ASSERT_EQ(goals.size(), 1u);
  EXPECT_TRUE(contains(clusters[0], goals[0].x, goals[0].y));
  // Centroid = (10/9, 10/9) = (1.11, 1.11): the nearest cells are (1, 0) and (0, 1); the first in
  // the cluster order wins.
  EXPECT_EQ(goals[0].x, 0);
  EXPECT_EQ(goals[0].y, 1);
}

TEST(Frontiers, GoalsAreSortedByDistanceToRobot) {
  std::vector<std::vector<awm::GridCell>> clusters = {
      {{20, 20}},  // far
      {{3, 4}},    // 5 cells from the robot
      {{9, 0}},    // 9 cells
  };
  std::vector<awm::GridCell> goals;
  awm::frontierGoals(clusters, 0, 0, &goals);
  ASSERT_EQ(goals.size(), 3u);
  EXPECT_EQ(goals[0].x, 3);
  EXPECT_EQ(goals[1].x, 9);
  EXPECT_EQ(goals[2].x, 20);

  // From the other side the order flips.
  awm::frontierGoals(clusters, 20, 20, &goals);
  EXPECT_EQ(goals[0].x, 20);
  EXPECT_EQ(goals[1].x, 9);
  EXPECT_EQ(goals[2].x, 3);
}

TEST(Frontiers, NoClustersNoGoals) {
  std::vector<awm::GridCell> goals = {{1, 1}};
  awm::frontierGoals({}, 0, 0, &goals);
  EXPECT_TRUE(goals.empty());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
