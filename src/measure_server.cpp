/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

#include "ros/ros.h"
#include "awm/Measure.h"

bool measure(awm::Measure::Request  &req,
         awm::Measure::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "measure_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("measure", measure);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
