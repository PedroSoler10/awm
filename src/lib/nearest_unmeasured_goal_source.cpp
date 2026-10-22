/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

#include "awm/nearest_unmeasured_goal_source.h"

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>

#include "awm/grid.h"
#include "awm/measure_map.h"

namespace awm {

bool NearestUnmeasuredGoalSource::next(const Context& ctx, geometry_msgs::PoseStamped* goal) {
  int robot_x;
  int robot_y;
  worldToGrid(ctx.measure_map, ctx.robot_pose.pose, &robot_x, &robot_y);
  int x;
  int y;
  int angle_deg;
  if (!findNearestCell(ctx.measure_map, robot_x, robot_y, kToMeasure, search_limit_cells_, &x, &y,
                       &angle_deg)) {
    ROS_INFO("EVERYTHING MEASURED");
    return false;
  }

  // Cell -> world coordinates, facing the direction the cell was found in.
  *goal = ctx.robot_pose;
  gridToWorld(ctx.measure_map, x, y, &goal->pose.position.x, &goal->pose.position.y);
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, angle_deg * M_PI / 180);
  goal->pose.orientation = tf2::toMsg(orientation);

  ROS_INFO("NEW GOAL: cell x: %d y: %d, angle: %d", x, y, angle_deg);
  return true;
}

}  // namespace awm
