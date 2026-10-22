/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

#ifndef AWM_NEAREST_UNMEASURED_GOAL_SOURCE_H_
#define AWM_NEAREST_UNMEASURED_GOAL_SOURCE_H_

#include <geometry_msgs/PoseStamped.h>

#include "awm/goal_source.h"

namespace awm {

// The closest cell of the measure map still to be measured (ring search from the robot cell),
// facing the direction it was found in. Needs no other node.
class NearestUnmeasuredGoalSource : public GoalSource {
 public:
  explicit NearestUnmeasuredGoalSource(int search_limit_cells)
      : search_limit_cells_(search_limit_cells) {}

  bool next(const Context& ctx, geometry_msgs::PoseStamped* goal) override;

 private:
  int search_limit_cells_;  // Farthest goal searched from the robot.
};

}  // namespace awm

#endif  // AWM_NEAREST_UNMEASURED_GOAL_SOURCE_H_
