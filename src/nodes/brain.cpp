/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

/*
 * brain: autonomous exploration node (ROS 1 Noetic, TurtleBot3 + SLAM + move_base).
 *
 * Repeatedly: pick the closest free cell that has not been "measured" yet, send it to move_base as
 * a goal, wait until move_base finishes, stand still for a few seconds taking a "measurement", and
 * start over. brain does NOT drive the motors: move_base plans the path and drives.
 *
 * Subscribes: /map                (nav_msgs/OccupancyGrid)         SLAM map
 *             /move_base/status   (actionlib_msgs/GoalStatusArray) state of the goals
 * Publishes:  /move_base_simple/goal (geometry_msgs/PoseStamped)   next goal for move_base
 *             /scan_pose          (geometry_msgs/PoseStamped)      robot pose in the "map" frame
 *             /measure_map        (nav_msgs/OccupancyGrid)         what has been measured so far
 * Private params (defaults in brackets):
 *   ~measure_duration_s [3.0], ~measure_radius_m [0.5], ~inflate_radius_cells [6],
 *   ~search_limit_cells [200], ~occupied_threshold [90], ~output_dir [.] for the measurement files
 *
 * Grid values (nav_msgs/OccupancyGrid): -1 unknown, 0 free, 100 occupied. Cell (x, y) is stored at
 * data[y * width + x]. In measure_map (same message type, our own meaning):
 *   100  obstacle, unknown, or too close to an obstacle (cannot be a goal)
 *     0  free cell already measured
 *    -1  free cell still to be measured  <- candidates for the next goal
 */

// C system headers (.h): ROS core, ROS messages, TF2
#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// C++ standard library (no .h suffix)
#include <cmath>
#include <string>

// This package's library
#include "awm/grid.h"
#include "awm/measure_map.h"
#include "awm/robot_pose.h"

class Brain {
 public:
  // The constructor only sets up. The program itself runs in run().
  Brain() {
    ros::NodeHandle pnh("~");
    pnh.param("measure_duration_s", measure_duration_s_, 3.0);
    pnh.param("measure_radius_m", measure_params_.measure_radius_m, 0.5);
    pnh.param("inflate_radius_cells", measure_params_.inflate_radius_cells, 6);
    pnh.param("search_limit_cells", search_limit_cells_, 200);
    pnh.param("occupied_threshold", measure_params_.occupied_threshold, 90);
    pnh.param("output_dir", output_dir_, std::string("."));

    pub_scan_pose_ = n_.advertise<geometry_msgs::PoseStamped>("/scan_pose", 100);
    pub_measure_map_ = n_.advertise<nav_msgs::OccupancyGrid>("/measure_map", 100);
    pub_goal_ = n_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    sub_map_ = n_.subscribe("/map", 100, &Brain::mapCallback, this);
    sub_status_ = n_.subscribe("/move_base/status", 10, &Brain::statusCallback, this);
    ROS_INFO("INITIALIZATION DONE");
  }

  void run() {
    // Each pass processes pending callbacks, then runs one step of the state machine.
    // 10 Hz: fast enough for the controller, slow enough to keep CPU load low.
    ros::Rate loop_rate(10);
    while (ros::ok()) {
      ros::spinOnce();
      control();
      loop_rate.sleep();
    }
  }

 private:
  // WAITING (once) -> MEASURING -> REACHING_GOAL -> MEASURING -> ... -> DONE
  enum class State { WAITING, MEASURING, REACHING_GOAL, DONE };

  // CALLBACKS //
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) { map_ = *msg; }

  // move_base publishes the state of all recent goals. We want the state of OUR latest goal.
  void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& st) {
    const actionlib_msgs::GoalStatus* latest = nullptr;
    for (const actionlib_msgs::GoalStatus& s : st->status_list) {
      // Goals older than the one we sent are history (e.g. the previous goal, still listed for a
      // few seconds as SUCCEEDED). Ignoring them avoids a false "goal reached". This assumes
      // move_base stamps the goal with the time it received it.
      if (s.goal_id.stamp < goal_sent_time_) continue;
      if (latest == nullptr || s.goal_id.stamp > latest->goal_id.stamp) latest = &s;
    }
    if (latest != nullptr) goal_status_ = latest->status;
  }

  // CONTROL //

  // One step of the state machine, called at 10 Hz from run().
  void control() {
    // Every state needs the robot pose and a map; until both exist SLAM or tf are not up yet.
    if (!readScanPose() || map_.data.empty()) {
      ROS_INFO_THROTTLE(1.0, "WAITING FOR MAP AND SCAN POSE");
      return;
    }

    switch (state_) {
      case State::WAITING:
        startMeasuring();
        break;

      case State::MEASURING:
        if (!measure()) break;
        if (pubGoal()) {
          state_ = State::REACHING_GOAL;
        } else {
          ROS_INFO("EVERYTHING MEASURED");
          state_ = State::DONE;
        }
        break;

      case State::REACHING_GOAL:
        if (goalFinished(goal_status_)) {
          ROS_INFO(
              "GOAL %s (status %d)",
              goal_status_ == actionlib_msgs::GoalStatus::SUCCEEDED ? "REACHED" : "NOT REACHED",
              goal_status_);
          // Mark the area around the goal as measured. If the goal could not be reached, this
          // stops the search from choosing the same cell again and again.
          int goal_x;
          int goal_y;
          awm::worldToGrid(measure_map_, goal_pose_.pose, &goal_x, &goal_y);
          awm::markDisk(&measure_map_, goal_x, goal_y, measure_params_.measure_radius_m,
                        awm::kMeasured, awm::kBlocked);
          startMeasuring();
        }
        break;

      case State::DONE:
        break;
    }
  }

  // ACTIONS //

  // Enters the MEASURING state: the robot stays still for measure_duration_s_ seconds.
  void startMeasuring() {
    ROS_INFO("DOING MEASURING");
    measure_start_ = ros::Time::now();
    state_ = State::MEASURING;
  }

  // Called every tick while MEASURING. Returns true once the robot has been still long enough and
  // the measurement has been recorded.
  bool measure() {
    if ((ros::Time::now() - measure_start_) < ros::Duration(measure_duration_s_)) return false;

    std::string file_name;
    const bool written = awm::writeMeasurementFile(output_dir_, scan_pose_.pose, &file_name);
    ROS_INFO("MEASURE DONE: %s", file_name.c_str());
    if (!written) ROS_WARN("COULD NOT CREATE %s", file_name.c_str());

    updateMeasureMap();
    return true;
  }

  // Rebuilds measure_map_ from the latest SLAM map and the current robot position, then publishes.
  void updateMeasureMap() {
    awm::updateMeasureMap(map_, scan_pose_.pose.position.x, scan_pose_.pose.position.y,
                          measure_params_, &measure_map_);
    measure_map_.header.stamp = ros::Time::now();
    pub_measure_map_.publish(measure_map_);
  }

  // Chooses the closest unmeasured cell and publishes it as the goal.
  // Returns false if there is nothing left to measure.
  bool pubGoal() {
    int robot_x;
    int robot_y;
    awm::worldToGrid(measure_map_, scan_pose_.pose, &robot_x, &robot_y);
    int x;
    int y;
    int angle_deg;
    if (!awm::findNearestCell(measure_map_, robot_x, robot_y, awm::kToMeasure, search_limit_cells_,
                              &x, &y, &angle_deg))
      return false;

    // Cell -> world coordinates, facing the direction the cell was found in.
    goal_pose_ = scan_pose_;
    goal_pose_.header.stamp = ros::Time::now();
    awm::gridToWorld(measure_map_, x, y, &goal_pose_.pose.position.x, &goal_pose_.pose.position.y);
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, angle_deg * M_PI / 180);
    goal_pose_.pose.orientation = tf2::toMsg(orientation);

    ROS_INFO("NEW GOAL: cell x: %d y: %d, angle: %d", x, y, angle_deg);
    goal_status_ = actionlib_msgs::GoalStatus::PENDING;  // Forget the previous goal's result.
    goal_sent_time_ = ros::Time::now();
    pub_goal_.publish(goal_pose_);
    return true;
  }

  // HELPERS //

  // Robot pose in the "map" frame (-> scan_pose_), also published on /scan_pose.
  // Returns false (and keeps the old scan_pose_) if tf cannot provide the transform yet.
  bool readScanPose() {
    if (!robot_pose_.lookup(&scan_pose_)) return false;
    pub_scan_pose_.publish(scan_pose_);
    return true;
  }

  // True once move_base is done with the goal, successfully or not. Still running: PENDING,
  // ACTIVE, PREEMPTING, RECALLING.
  static bool goalFinished(int status) {
    using actionlib_msgs::GoalStatus;
    return status == GoalStatus::SUCCEEDED || status == GoalStatus::ABORTED ||
           status == GoalStatus::REJECTED || status == GoalStatus::PREEMPTED ||
           status == GoalStatus::RECALLED || status == GoalStatus::LOST;
  }

  // DATA //
  ros::NodeHandle n_;

  double measure_duration_s_;  // Time standing still per measurement.
  int search_limit_cells_;     // Farthest goal searched from the robot.
  awm::MeasureMapParams measure_params_;
  std::string output_dir_;  // Where the measurement files go.

  awm::RobotPose robot_pose_;
  geometry_msgs::PoseStamped scan_pose_;  // Robot pose in "map".
  ros::Publisher pub_scan_pose_;

  nav_msgs::OccupancyGrid map_;  // Latest SLAM map.
  nav_msgs::OccupancyGrid measure_map_;
  ros::Publisher pub_measure_map_;
  ros::Subscriber sub_map_;

  geometry_msgs::PoseStamped goal_pose_;
  ros::Publisher pub_goal_;
  ros::Subscriber sub_status_;
  int goal_status_ = actionlib_msgs::GoalStatus::PENDING;  // Status of our latest goal.
  ros::Time goal_sent_time_{0, 0};

  ros::Time measure_start_{0, 0};
  State state_ = State::WAITING;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "brain");
  Brain brain;
  brain.run();
  return 0;
}
