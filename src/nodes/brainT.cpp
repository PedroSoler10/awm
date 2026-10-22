/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

/*
 * brainT: the modular version of brain (ROS 1 Noetic, TurtleBot3 + SLAM + move_base).
 *
 * Same state machine as brain, but the goal is not searched here: it is asked from the
 * /goal_points service of points_identification. The node is still called "brain", so the topics
 * are the same and only one of brain / brainT can run at a time.
 *
 * Repeatedly: ask /goal_points for the best goal, send it to move_base, wait until move_base
 * finishes, stand still for a few seconds taking a "measurement", and start over. An empty answer
 * means there is nothing left to do. brainT does NOT drive the motors: move_base does.
 *
 * Needs running: map_processing (/safe_map), points_identification (/goal_points), see
 * launch/modular.launch.
 * Subscribes: /safe_map           (nav_msgs/OccupancyGrid)         map with the caution band
 *             /move_base/status   (actionlib_msgs/GoalStatusArray) state of the goals
 * Publishes:  /move_base_simple/goal (geometry_msgs/PoseStamped)   next goal for move_base
 *             /scan_pose          (geometry_msgs/PoseStamped)      robot pose in the "map" frame
 *             /measure_map        (nav_msgs/OccupancyGrid, latched) what has been measured so far
 * Calls:      /goal_points        (awm/GoalPoints)
 * Private params (defaults in brackets):
 *   ~goal_mode [3] mode sent to /goal_points (1 EXPLORATION, 2 MEASUREMENT, 3 FUSION),
 *   ~measure_duration_s [3.0], ~measure_radius_m [0.5], ~output_dir [.] for the measurement files
 *
 * The measure map has the meaning described in awm/measure_map.h. It is built on /safe_map, where
 * caution cells (value 50) already are the margin around obstacles, so they are blocked and never
 * become goals.
 */

#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <cstdint>
#include <string>

#include "awm/GoalPoints.h"
#include "awm/grid.h"
#include "awm/measure_map.h"
#include "awm/robot_pose.h"
#include "awm/safe_map.h"

class BrainT {
 public:
  // The constructor only sets up. The program itself runs in run().
  BrainT() {
    ros::NodeHandle pnh("~");
    int goal_mode;
    pnh.param("goal_mode", goal_mode, static_cast<int>(awm::GoalPoints::Request::FUSION));
    pnh.param("measure_duration_s", measure_duration_s_, 3.0);
    pnh.param("measure_radius_m", measure_params_.measure_radius_m, 0.5);
    pnh.param("output_dir", output_dir_, std::string("."));

    using Request = awm::GoalPoints::Request;
    if (goal_mode != Request::EXPLORATION && goal_mode != Request::MEASUREMENT &&
        goal_mode != Request::FUSION) {
      ROS_FATAL("~goal_mode must be 1 (EXPLORATION), 2 (MEASUREMENT) or 3 (FUSION), got %d",
                goal_mode);
      ros::shutdown();
      return;
    }
    goal_mode_ = static_cast<std::uint8_t>(goal_mode);

    // The safe map already holds the margin around obstacles (value 50), so cells above
    // kSafeCaution - 1 are blocked and no extra inflation is needed.
    measure_params_.occupied_threshold = awm::kSafeCaution - 1;
    measure_params_.inflate_radius_cells = 0;

    pub_scan_pose_ = n_.advertise<geometry_msgs::PoseStamped>("/scan_pose", 100);
    pub_measure_map_ = n_.advertise<nav_msgs::OccupancyGrid>("/measure_map", 100, true);
    pub_goal_ = n_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    sub_safe_map_ = n_.subscribe("/safe_map", 10, &BrainT::safeMapCallback, this);
    sub_status_ = n_.subscribe("/move_base/status", 10, &BrainT::statusCallback, this);
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
  // WAITING (once) -> MEASURING -> CHOOSING_GOAL -> REACHING_GOAL -> MEASURING -> ... -> DONE
  // CHOOSING_GOAL is separate from MEASURING because the measurement must be recorded only once,
  // while asking for a goal may have to be repeated until the service is there.
  enum class State { WAITING, MEASURING, CHOOSING_GOAL, REACHING_GOAL, DONE };

  // Result of asking /goal_points for a goal.
  enum class GoalResult { SENT, NOTHING_LEFT, SERVICE_UNAVAILABLE };

  // CALLBACKS //
  void safeMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) { safe_map_ = *msg; }

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
    if (!readScanPose() || safe_map_.data.empty()) {
      ROS_INFO_THROTTLE(1.0, "WAITING FOR SAFE MAP AND SCAN POSE");
      return;
    }

    switch (state_) {
      case State::WAITING:
        startMeasuring();
        break;

      case State::MEASURING:
        if (measure()) state_ = State::CHOOSING_GOAL;
        break;

      case State::CHOOSING_GOAL:
        switch (requestGoal()) {
          case GoalResult::SENT:
            state_ = State::REACHING_GOAL;
            break;
          case GoalResult::NOTHING_LEFT:
            ROS_INFO("NO MORE GOAL POINTS");
            state_ = State::DONE;
            break;
          case GoalResult::SERVICE_UNAVAILABLE:
            break;  // Try again on the next tick.
        }
        break;

      case State::REACHING_GOAL:
        if (goalFinished(goal_status_)) {
          ROS_INFO(
              "GOAL %s (status %d)",
              goal_status_ == actionlib_msgs::GoalStatus::SUCCEEDED ? "REACHED" : "NOT REACHED",
              goal_status_);
          // Mark the area around the goal as measured. If the goal could not be reached, this
          // stops the same cell from being chosen again and again.
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

  // Rebuilds measure_map_ from the latest safe map and the current robot position, then publishes.
  void updateMeasureMap() {
    awm::updateMeasureMap(safe_map_, scan_pose_.pose.position.x, scan_pose_.pose.position.y,
                          measure_params_, &measure_map_);
    measure_map_.header.stamp = ros::Time::now();
    pub_measure_map_.publish(measure_map_);
  }

  // Asks /goal_points for the best goal and publishes it for move_base.
  GoalResult requestGoal() {
    if (!ros::service::waitForService("/goal_points", ros::Duration(1.0))) {
      ROS_WARN_THROTTLE(5.0, "WAITING FOR SERVICE /goal_points (IS points_identification UP?)");
      return GoalResult::SERVICE_UNAVAILABLE;
    }

    awm::GoalPoints srv;
    srv.request.mode = goal_mode_;
    srv.request.num = 1;
    if (!ros::service::call("/goal_points", srv)) {
      ROS_WARN_THROTTLE(5.0, "CALL TO /goal_points FAILED");
      return GoalResult::SERVICE_UNAVAILABLE;
    }
    if (srv.response.points.empty()) return GoalResult::NOTHING_LEFT;

    // Face the direction of travel; the service gives positions only.
    const geometry_msgs::Point& target = srv.response.points.front();
    const double heading =
        std::atan2(target.y - scan_pose_.pose.position.y, target.x - scan_pose_.pose.position.x);
    goal_pose_ = scan_pose_;
    goal_pose_.header.stamp = ros::Time::now();
    goal_pose_.pose.position.x = target.x;
    goal_pose_.pose.position.y = target.y;
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, heading);
    goal_pose_.pose.orientation = tf2::toMsg(orientation);

    int x;
    int y;
    awm::worldToGrid(measure_map_, goal_pose_.pose, &x, &y);
    ROS_INFO("NEW GOAL: cell x: %d y: %d, angle: %d", x, y,
             static_cast<int>(std::lround(heading * 180 / M_PI)));
    goal_status_ = actionlib_msgs::GoalStatus::PENDING;  // Forget the previous goal's result.
    goal_sent_time_ = ros::Time::now();
    pub_goal_.publish(goal_pose_);
    return GoalResult::SENT;
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

  std::uint8_t goal_mode_;     // Mode of the /goal_points request.
  double measure_duration_s_;  // Time standing still per measurement.
  awm::MeasureMapParams measure_params_;
  std::string output_dir_;  // Where the measurement files go.

  awm::RobotPose robot_pose_;
  geometry_msgs::PoseStamped scan_pose_;  // Robot pose in "map".
  ros::Publisher pub_scan_pose_;

  nav_msgs::OccupancyGrid safe_map_;  // Latest safe map.
  nav_msgs::OccupancyGrid measure_map_;
  ros::Publisher pub_measure_map_;
  ros::Subscriber sub_safe_map_;

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
  BrainT brain;
  brain.run();
  return 0;
}
