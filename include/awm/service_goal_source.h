/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

#ifndef AWM_SERVICE_GOAL_SOURCE_H_
#define AWM_SERVICE_GOAL_SOURCE_H_

#include <geometry_msgs/PoseStamped.h>

#include <cstdint>
#include <string>

#include "awm/goal_source.h"

namespace awm {

// True for the modes /goal_points accepts (awm::GoalPoints::Request::EXPLORATION, MEASUREMENT,
// FUSION).
bool isValidGoalMode(int goal_mode);

// Asks the /goal_points service (points_identification) for one goal, in the given mode
// (awm::GoalPoints::Request::EXPLORATION, MEASUREMENT or FUSION). The goal faces the direction
// from the robot to the point, because the service gives positions only.
class ServiceGoalSource : public GoalSource {
 public:
  explicit ServiceGoalSource(std::uint8_t goal_mode, const std::string& service = "/goal_points")
      : goal_mode_(goal_mode), service_(service) {}

  // Blocks until the service answers, so false only means that it has no goal left (or that ROS
  // is shutting down).
  bool next(const Context& ctx, geometry_msgs::PoseStamped* goal) override;

 private:
  std::uint8_t goal_mode_;
  std::string service_;
};

}  // namespace awm

#endif  // AWM_SERVICE_GOAL_SOURCE_H_
