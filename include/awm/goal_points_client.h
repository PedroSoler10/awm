/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

#ifndef AWM_GOAL_POINTS_CLIENT_H_
#define AWM_GOAL_POINTS_CLIENT_H_

#include <geometry_msgs/PoseStamped.h>

#include <cstdint>
#include <string>

namespace awm {

// True for the modes /goal_points accepts (awm::GoalPoints::Request::EXPLORATION, MEASUREMENT,
// FUSION).
bool isValidGoalMode(int goal_mode);

// How brain gets its goals: asks the /goal_points service (points_identification) for one goal
// in goal_mode and writes it to *goal, in the frame of robot_pose (header copied from it), facing
// the direction from the robot to the point, because the service gives positions only. Blocks
// until the service answers, so false only means that it has no goal left (or that ROS is
// shutting down). The caller stamps and sends the goal.
bool requestGoal(std::uint8_t goal_mode, const geometry_msgs::PoseStamped& robot_pose,
                 geometry_msgs::PoseStamped* goal, const std::string& service = "/goal_points");

}  // namespace awm

#endif  // AWM_GOAL_POINTS_CLIENT_H_
