/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

#include "awm/service_goal_source.h"

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>

#include "awm/GoalPoints.h"
#include "awm/grid.h"

namespace awm {

bool ServiceGoalSource::isValidMode(int goal_mode) {
  using Request = GoalPoints::Request;
  return goal_mode == Request::EXPLORATION || goal_mode == Request::MEASUREMENT ||
         goal_mode == Request::FUSION;
}

bool ServiceGoalSource::next(const Context& ctx, geometry_msgs::PoseStamped* goal) {
  GoalPoints srv;
  srv.request.mode = goal_mode_;
  srv.request.num = 1;
  // An unreachable service is not "nothing left": wait for it, so the robot does not stop for
  // good when points_identification starts late or restarts.
  while (true) {
    if (!ros::ok()) return false;
    if (!ros::service::waitForService(service_, ros::Duration(1.0))) {
      ROS_WARN_THROTTLE(5.0, "WAITING FOR SERVICE %s (IS points_identification UP?)",
                        service_.c_str());
      continue;
    }
    if (ros::service::call(service_, srv)) break;
    ROS_WARN_THROTTLE(5.0, "CALL TO %s FAILED", service_.c_str());
    ros::WallDuration(1.0).sleep();
  }
  if (srv.response.points.empty()) {
    ROS_INFO("NO MORE GOAL POINTS");
    return false;
  }

  // Face the direction of travel.
  const geometry_msgs::Point& target = srv.response.points.front();
  const geometry_msgs::Point& robot = ctx.robot_pose.pose.position;
  const double heading = std::atan2(target.y - robot.y, target.x - robot.x);
  *goal = ctx.robot_pose;
  goal->pose.position.x = target.x;
  goal->pose.position.y = target.y;
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, heading);
  goal->pose.orientation = tf2::toMsg(orientation);

  int x;
  int y;
  worldToGrid(ctx.measure_map, goal->pose, &x, &y);
  ROS_INFO("NEW GOAL: cell x: %d y: %d, angle: %d", x, y,
           static_cast<int>(std::lround(heading * 180 / M_PI)));
  return true;
}

}  // namespace awm
