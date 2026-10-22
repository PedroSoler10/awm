/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

#ifndef AWM_ROBOT_POSE_H_
#define AWM_ROBOT_POSE_H_

#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace awm {

// Robot pose in the "map" frame, read from tf as the origin of the LiDAR frame (base_scan).
// Construct it after ros::init: the listener subscribes to /tf at once.
class RobotPose {
 public:
  RobotPose();

  // Writes the latest pose to *out, stamped with the current time. Returns false (and leaves *out
  // unchanged) if tf cannot provide the transform yet.
  bool lookup(geometry_msgs::PoseStamped* out);

 private:
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_{buffer_};  // Must be declared after buffer_.
  geometry_msgs::PoseStamped base_scan_pose_;     // Constant: origin of the base_scan frame.
};

}  // namespace awm

#endif  // AWM_ROBOT_POSE_H_
