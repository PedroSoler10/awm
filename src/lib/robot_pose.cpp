/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

#include "awm/robot_pose.h"

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace awm {

RobotPose::RobotPose() {
  // The LiDAR pose in its own frame: the origin, no rotation. Transformed to "map" it gives the
  // robot position.
  base_scan_pose_.header.frame_id = "base_scan";
  base_scan_pose_.pose.orientation.w = 1;
}

bool RobotPose::lookup(geometry_msgs::PoseStamped* out) {
  // Stamp 0 means "latest available transform". A fixed old stamp would ask tf for the pose at
  // that old moment, which tf stops storing after about 10 seconds.
  base_scan_pose_.header.stamp = ros::Time(0);
  try {
    buffer_.transform(base_scan_pose_, *out, "map");
  } catch (tf2::TransformException& ex) {
    ROS_WARN_THROTTLE(1.0, "ERROR WITH SCAN POSE: %s", ex.what());
    return false;
  }
  out->header.stamp = ros::Time::now();
  return true;
}

}  // namespace awm
