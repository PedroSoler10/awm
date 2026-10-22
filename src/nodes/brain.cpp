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
 * Repeatedly: choose a goal, send it to move_base, wait until move_base finishes, stand still for a
 * moment taking a "measurement", and start over. brain does NOT drive the motors: move_base plans
 * the path and drives.
 *
 * How the goal is chosen is a GoalSource (awm/goal_source.h), picked by ~goal_source:
 *   internal  closest free cell not measured yet, searched here on a measure map built on /map.
 *   service   asked from /goal_points (points_identification, which needs map_processing); the
 *             measure map is built on /safe_map, whose caution band (value 50) is blocked.
 *
 * Subscribes: /map or /safe_map  (nav_msgs/OccupancyGrid)          SLAM map / safe map
 * Publishes:  /scan_pose         (geometry_msgs/PoseStamped)       robot pose in the "map" frame
 *             /measure_map       (nav_msgs/OccupancyGrid, latched) what has been measured so far
 * Action:     /move_base         (move_base_msgs/MoveBase)         client: sends the goals
 * Calls:      /goal_points       (awm/GoalPoints), service mode only
 * Private params (defaults in brackets, internal / service when they differ):
 *   ~goal_source [internal] internal or service, ~goal_mode [3] (service only: 1 EXPLORATION,
 *   2 MEASUREMENT, 3 FUSION), ~search_limit_cells [200] (internal only), ~measure_duration_s [1.0],
 *   ~measure_radius_m [0.5], ~inflate_radius_cells [6 / 0], ~occupied_threshold [90 / 49],
 *   ~output_dir [.] for the measurement files
 *
 * Grid values (nav_msgs/OccupancyGrid): -1 unknown, 0 free, 100 occupied. Cell (x, y) is stored at
 * data[y * width + x]. In measure_map (same message type, our own meaning):
 *   100  obstacle, unknown, or too close to an obstacle (cannot be a goal)
 *     0  free cell already measured
 *    -1  free cell still to be measured  <- candidates for the next goal
 */

// C system headers (.h): ROS core and ROS messages
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

// C++ standard library (no .h suffix)
#include <cstdint>
#include <memory>
#include <string>

// This package's library and service
#include "awm/GoalPoints.h"
#include "awm/goal_source.h"
#include "awm/grid.h"
#include "awm/measure_map.h"
#include "awm/nearest_unmeasured_goal_source.h"
#include "awm/robot_pose.h"
#include "awm/safe_map.h"
#include "awm/service_goal_source.h"

class Brain {
 public:
  // The constructor only sets up. The program itself runs in run(). A bad parameter logs
  // ROS_FATAL and shuts ROS down, so ros::ok() is false afterwards.
  Brain() {
    ros::NodeHandle pnh("~");
    std::string goal_source;
    pnh.param("goal_source", goal_source, std::string("internal"));
    const bool service = goal_source == "service";
    if (!service && goal_source != "internal") {
      ROS_FATAL("~goal_source must be internal or service, got %s", goal_source.c_str());
      ros::shutdown();
      return;
    }

    pnh.param("measure_duration_s", measure_duration_s_, 1.0);
    pnh.param("measure_radius_m", measure_params_.measure_radius_m, 0.5);
    // On /safe_map the caution band already is the margin around obstacles: cells from 50 up are
    // blocked and no extra inflation is needed.
    pnh.param("inflate_radius_cells", measure_params_.inflate_radius_cells, service ? 0 : 6);
    pnh.param("occupied_threshold", measure_params_.occupied_threshold,
              service ? awm::kSafeCaution - 1 : 90);
    pnh.param("output_dir", output_dir_, std::string("."));

    std::string map_topic = "/map";
    if (service) {
      int goal_mode;
      pnh.param("goal_mode", goal_mode, static_cast<int>(awm::GoalPoints::Request::FUSION));
      if (!awm::isValidGoalMode(goal_mode)) {
        ROS_FATAL("~goal_mode must be 1 (EXPLORATION), 2 (MEASUREMENT) or 3 (FUSION), got %d",
                  goal_mode);
        ros::shutdown();
        return;
      }
      goal_source_ = std::make_unique<awm::ServiceGoalSource>(static_cast<std::uint8_t>(goal_mode));
      map_topic = "/safe_map";
      map_name_ = "SAFE MAP";
    } else {
      int search_limit_cells;
      pnh.param("search_limit_cells", search_limit_cells, 200);
      goal_source_ = std::make_unique<awm::NearestUnmeasuredGoalSource>(search_limit_cells);
    }

    // Queues of 1: only the latest pose and map matter.
    pub_scan_pose_ = n_.advertise<geometry_msgs::PoseStamped>("/scan_pose", 1);
    pub_measure_map_ = n_.advertise<nav_msgs::OccupancyGrid>("/measure_map", 1, true);
    sub_map_ = n_.subscribe(map_topic, 1, &Brain::mapCallback, this);
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
  // The map is only read, so we keep the shared message instead of copying it.
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) { map_ = msg; }

  // CONTROL //

  // One step of the state machine, called at 10 Hz from run().
  void control() {
    // Every state needs the robot pose and a map; until both exist SLAM or tf are not up yet.
    if (!readScanPose() || !map_ || map_->data.empty() || !awm::hasFullData(*map_)) {
      ROS_INFO_THROTTLE(1.0, "WAITING FOR %s AND SCAN POSE", map_name_);
      return;
    }
    // A goal sent before move_base is up would be lost.
    if (!move_base_.isServerConnected()) {
      ROS_INFO_THROTTLE(1.0, "WAITING FOR move_base");
      return;
    }

    switch (state_) {
      case State::WAITING:
        startMeasuring();
        break;

      case State::MEASURING:
        if (!measure()) break;
        // The goal source logs why it has no goal left.
        state_ = sendGoal() ? State::REACHING_GOAL : State::DONE;
        break;

      case State::REACHING_GOAL: {
        // Done means finished either way: succeeded, aborted, rejected, preempted, ...
        const actionlib::SimpleClientGoalState goal_state = move_base_.getState();
        if (goal_state.isDone()) {
          const bool reached = goal_state == actionlib::SimpleClientGoalState::SUCCEEDED;
          ROS_INFO("GOAL %s (%s)", reached ? "REACHED" : "NOT REACHED",
                   goal_state.toString().c_str());
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
      }

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

  // Rebuilds measure_map_ from the latest map and the current robot position, then publishes.
  void updateMeasureMap() {
    awm::updateMeasureMap(*map_, scan_pose_.pose.position.x, scan_pose_.pose.position.y,
                          measure_params_, &measure_map_);
    measure_map_.header.stamp = ros::Time::now();
    pub_measure_map_.publish(measure_map_);
  }

  // Asks the goal source for the next goal and sends it to move_base.
  // Returns false if there is nothing left to do.
  bool sendGoal() {
    if (!goal_source_->next({scan_pose_, measure_map_}, &goal_pose_)) return false;

    goal_pose_.header.stamp = ros::Time::now();
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = goal_pose_;
    // The client tracks this goal by its id, so getState() never reports an older goal.
    move_base_.sendGoal(goal);
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

  // DATA //
  ros::NodeHandle n_;

  double measure_duration_s_;  // Time standing still per measurement.
  awm::MeasureMapParams measure_params_;
  std::string output_dir_;  // Where the measurement files go.
  std::unique_ptr<awm::GoalSource> goal_source_;

  awm::RobotPose robot_pose_;
  geometry_msgs::PoseStamped scan_pose_;  // Robot pose in "map".
  ros::Publisher pub_scan_pose_;

  nav_msgs::OccupancyGrid::ConstPtr map_;  // Latest /map (internal) or /safe_map (service).
  const char* map_name_ = "MAP";           // For the log.
  nav_msgs::OccupancyGrid measure_map_;
  ros::Publisher pub_measure_map_;
  ros::Subscriber sub_map_;

  geometry_msgs::PoseStamped goal_pose_;
  // No thread of its own (false): its callbacks run in ros::spinOnce() in run().
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_{"move_base", false};

  ros::Time measure_start_{0, 0};
  State state_ = State::WAITING;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "brain");
  Brain brain;
  if (!ros::ok()) return 1;  // Bad parameter, already reported.
  brain.run();
  return 0;
}
