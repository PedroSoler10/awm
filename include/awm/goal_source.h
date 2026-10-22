/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

/*
 * GoalSource: how brain chooses its next goal (Strategy pattern). brain owns one implementation,
 * chosen by its ~goal_source parameter:
 *   internal  NearestUnmeasuredGoalSource  searches the measure map in-process
 *   service   ServiceGoalSource            asks the /goal_points service
 */

#ifndef AWM_GOAL_SOURCE_H_
#define AWM_GOAL_SOURCE_H_

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>

namespace awm {

class GoalSource {
 public:
  // What brain knows when it asks for a goal.
  struct Context {
    const geometry_msgs::PoseStamped& robot_pose;  // Robot pose in the "map" frame.
    const nav_msgs::OccupancyGrid& measure_map;    // Meaning in awm/measure_map.h.
  };

  virtual ~GoalSource() = default;

  // Writes the next goal (in the frame of ctx.robot_pose, header copied from it) to *goal. Returns
  // false when there is nothing left to do. The caller stamps and publishes the goal.
  virtual bool next(const Context& ctx, geometry_msgs::PoseStamped* goal) = 0;
};

}  // namespace awm

#endif  // AWM_GOAL_SOURCE_H_
