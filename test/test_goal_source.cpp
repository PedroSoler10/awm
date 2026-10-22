/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

#include <geometry_msgs/PoseStamped.h>
#include <gtest/gtest.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <cstdint>

#include "awm/goal_source.h"
#include "awm/grid.h"
#include "awm/measure_map.h"
#include "awm/nearest_unmeasured_goal_source.h"

namespace {

// A width x height measure map with 5 cm cells and its origin at (-1, -2), every cell blocked.
nav_msgs::OccupancyGrid makeMeasureMap(int width, int height) {
  nav_msgs::OccupancyGrid grid;
  grid.info.width = width;
  grid.info.height = height;
  grid.info.resolution = 0.05;
  grid.info.origin.position.x = -1.0;
  grid.info.origin.position.y = -2.0;
  grid.data.assign(width * height, awm::kBlocked);
  return grid;
}

// Robot pose in "map" at the centre of cell (x, y).
geometry_msgs::PoseStamped robotAt(const nav_msgs::OccupancyGrid& grid, int x, int y) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.seq = 7;
  pose.pose.orientation.w = 1;
  awm::gridToWorld(grid, x, y, &pose.pose.position.x, &pose.pose.position.y);
  return pose;
}

TEST(NearestUnmeasuredGoalSource, ChoosesClosestCellToMeasure) {
  nav_msgs::OccupancyGrid map = makeMeasureMap(40, 40);
  *awm::cell(&map, 25, 20) = awm::kToMeasure;  // 5 cells east of the robot.
  *awm::cell(&map, 20, 28) = awm::kToMeasure;  // 8 cells north.
  *awm::cell(&map, 20, 22) = awm::kMeasured;   // Closer, but already measured.
  const geometry_msgs::PoseStamped robot = robotAt(map, 20, 20);

  awm::NearestUnmeasuredGoalSource source(200);
  geometry_msgs::PoseStamped goal;
  ASSERT_TRUE(source.next({robot, map}, &goal));

  int x;
  int y;
  awm::worldToGrid(map, goal.pose, &x, &y);
  EXPECT_EQ(x, 25);
  EXPECT_EQ(y, 20);
  EXPECT_NEAR(tf2::getYaw(goal.pose.orientation), 0.0, 1e-9);  // Found at angle 0 (east).
  // The header comes from the robot pose; the caller stamps it.
  EXPECT_EQ(goal.header.frame_id, "map");
  EXPECT_EQ(goal.header.seq, 7u);
}

TEST(NearestUnmeasuredGoalSource, FacesTheDirectionTheCellWasFoundIn) {
  nav_msgs::OccupancyGrid map = makeMeasureMap(40, 40);
  *awm::cell(&map, 20, 26) = awm::kToMeasure;  // North of the robot.
  const geometry_msgs::PoseStamped robot = robotAt(map, 20, 20);

  awm::NearestUnmeasuredGoalSource source(200);
  geometry_msgs::PoseStamped goal;
  ASSERT_TRUE(source.next({robot, map}, &goal));
  // The sweep reaches the cell a few degrees before 90 (cell rounding), and the goal keeps that
  // exact angle, as brain always did.
  int x;
  int y;
  int angle_deg;
  ASSERT_TRUE(awm::findNearestCell(map, 20, 20, awm::kToMeasure, 200, &x, &y, &angle_deg));
  EXPECT_GT(angle_deg, 80);
  EXPECT_LE(angle_deg, 90);
  EXPECT_NEAR(tf2::getYaw(goal.pose.orientation), angle_deg * M_PI / 180, 1e-9);
  EXPECT_NEAR(goal.pose.position.x, robot.pose.position.x, 1e-6);
  EXPECT_NEAR(goal.pose.position.y, robot.pose.position.y + 6 * 0.05, 1e-6);
}

TEST(NearestUnmeasuredGoalSource, NothingToMeasure) {
  nav_msgs::OccupancyGrid map = makeMeasureMap(40, 40);
  *awm::cell(&map, 10, 10) = awm::kMeasured;
  geometry_msgs::PoseStamped goal;
  goal.pose.position.x = 42.0;

  awm::NearestUnmeasuredGoalSource source(200);
  EXPECT_FALSE(source.next({robotAt(map, 20, 20), map}, &goal));
  EXPECT_EQ(goal.pose.position.x, 42.0);  // Untouched.
}

TEST(NearestUnmeasuredGoalSource, RespectsSearchLimit) {
  nav_msgs::OccupancyGrid map = makeMeasureMap(40, 40);
  *awm::cell(&map, 30, 20) = awm::kToMeasure;  // 10 cells away.
  const geometry_msgs::PoseStamped robot = robotAt(map, 20, 20);
  geometry_msgs::PoseStamped goal;

  EXPECT_FALSE(awm::NearestUnmeasuredGoalSource(9).next({robot, map}, &goal));
  EXPECT_TRUE(awm::NearestUnmeasuredGoalSource(10).next({robot, map}, &goal));
}

TEST(NearestUnmeasuredGoalSource, WorksThroughTheInterface) {
  nav_msgs::OccupancyGrid map = makeMeasureMap(10, 10);
  *awm::cell(&map, 3, 3) = awm::kToMeasure;
  const geometry_msgs::PoseStamped robot = robotAt(map, 3, 3);  // Distance 0.

  awm::NearestUnmeasuredGoalSource nearest(200);
  awm::GoalSource& source = nearest;
  geometry_msgs::PoseStamped goal;
  ASSERT_TRUE(source.next({robot, map}, &goal));
  EXPECT_NEAR(goal.pose.position.x, robot.pose.position.x, 1e-6);
  EXPECT_NEAR(goal.pose.position.y, robot.pose.position.y, 1e-6);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
