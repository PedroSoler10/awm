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
#include <tf2_ros/transform_listener.h>

// C++ standard library (no .h suffix)
#include <cmath>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <string>

class Brain {
 public:
  // The constructor only sets up. The program itself runs in run().
  Brain() {
    // The LiDAR pose in its own frame: the origin, no rotation. Transformed to "map" it gives the
    // robot position (see fnReadScanPose).
    base_scan_pose_.header.frame_id = "base_scan";
    base_scan_pose_.pose.orientation.w = 1;

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

  static constexpr double kMeasureDurationS = 3.0;  // Time standing still per measurement.
  static constexpr double kMeasureRadiusM = 0.5;    // Cells this close to the robot are measured.
  static constexpr int kInflateRadiusCells = 6;     // Safety margin around obstacles.
  static constexpr int kSearchLimitCells = 200;     // Farthest goal searched from the robot.
  static constexpr int kOccupiedThreshold = 90;     // Occupancy above this is an obstacle.

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
    if (!fnReadScanPose() || map_.data.empty()) {
      ROS_INFO_THROTTLE(1.0, "WAITING FOR MAP AND SCAN POSE");
      return;
    }

    switch (state_) {
      case State::WAITING:
        fnStartMeasuring();
        break;

      case State::MEASURING:
        if (!fnMeasure()) break;
        if (fnPubGoal()) {
          state_ = State::REACHING_GOAL;
        } else {
          ROS_INFO("EVERYTHING MEASURED");
          state_ = State::DONE;
        }
        break;

      case State::REACHING_GOAL:
        if (fnGoalFinished(goal_status_)) {
          ROS_INFO(
              "GOAL %s (status %d)",
              goal_status_ == actionlib_msgs::GoalStatus::SUCCEEDED ? "REACHED" : "NOT REACHED",
              goal_status_);
          // Mark the area around the goal as measured. If the goal could not be reached, this
          // stops the search from choosing the same cell again and again.
          int goal_x;
          int goal_y;
          fnWorldToGrid(measure_map_, goal_pose_.pose, &goal_x, &goal_y);
          fnMarkMeasured(&measure_map_, goal_x, goal_y);
          fnStartMeasuring();
        }
        break;

      case State::DONE:
        break;
    }
  }

  // ACTIONS //

  // Enters the MEASURING state: the robot stays still for kMeasureDurationS seconds.
  void fnStartMeasuring() {
    ROS_INFO("DOING MEASURING");
    measure_start_ = ros::Time::now();
    state_ = State::MEASURING;
  }

  // Called every tick while MEASURING. Returns true once the robot has been still long enough and
  // the measurement has been recorded.
  bool fnMeasure() {
    if ((ros::Time::now() - measure_start_) < ros::Duration(kMeasureDurationS)) return false;

    // Placeholder for the real measurement: an empty file named after position and local time,
    // e.g. "1.2_-0.5_2026-10-21T10-30-05.txt" (no colons, so it is also valid on Windows).
    const std::time_t now = std::time(nullptr);
    std::tm local_time;
    char time_text[32] = "";
    if (localtime_r(&now, &local_time) != nullptr)
      std::strftime(time_text, sizeof(time_text), "%Y-%m-%dT%H-%M-%S", &local_time);
    const std::string file_name = std::to_string(scan_pose_.pose.position.x) + "_" +
                                  std::to_string(scan_pose_.pose.position.y) + "_" + time_text +
                                  ".txt";
    ROS_INFO("MEASURE DONE: %s", file_name.c_str());
    std::ofstream measure_file(file_name);
    if (!measure_file.is_open()) ROS_WARN("COULD NOT CREATE %s", file_name.c_str());

    fnUpdateMeasureMap();
    return true;
  }

  // Rebuilds measure_map_ from the latest SLAM map and the current robot position, then publishes.
  void fnUpdateMeasureMap() {
    // When the SLAM map grows its size changes and the old measure_map_ no longer fits: start
    // again from the new map (cells measured so far are forgotten).
    const bool resized =
        measure_map_.info.width != map_.info.width || measure_map_.info.height != map_.info.height;
    if (resized) measure_map_ = map_;

    // 1) Obstacles and unknown cells -> 100. Other cells -> -1 (to measure), keeping those already
    //    measured (0). After a resize the copied values come from the SLAM map, where 0 means
    //    "free", not "measured", so everything is reset.
    for (std::size_t k = 0; k < map_.data.size(); k++) {
      if (map_.data[k] == -1 || map_.data[k] > kOccupiedThreshold)
        measure_map_.data[k] = 100;
      else if (resized || measure_map_.data[k] != 0)
        measure_map_.data[k] = -1;
    }

    // 2) Cells close to the robot are now measured.
    int robot_x;
    int robot_y;
    fnWorldToGrid(measure_map_, scan_pose_.pose, &robot_x, &robot_y);
    fnMarkMeasured(&measure_map_, robot_x, robot_y);

    // 3) Safety margin: cells next to an obstacle cannot be goals (-> 100) unless measured.
    for (std::size_t k = 0; k < map_.data.size(); k++) {
      if (map_.data[k] <= kOccupiedThreshold) continue;
      const int x = k % map_.info.width;
      const int y = k / map_.info.width;
      for (int ny = y - kInflateRadiusCells; ny <= y + kInflateRadiusCells; ny++) {
        for (int nx = x - kInflateRadiusCells; nx <= x + kInflateRadiusCells; nx++) {
          std::int8_t* cell = fnCell(&measure_map_, nx, ny);
          if (cell != nullptr && *cell != 0) *cell = 100;
        }
      }
    }

    measure_map_.header.stamp = ros::Time::now();
    pub_measure_map_.publish(measure_map_);
  }

  // Chooses the closest unmeasured cell and publishes it as the goal.
  // Returns false if there is nothing left to measure.
  bool fnPubGoal() {
    int robot_x;
    int robot_y;
    fnWorldToGrid(measure_map_, scan_pose_.pose, &robot_x, &robot_y);

    // Look at cells on circles of growing radius d around the robot, sweeping the angle from 0 to
    // 359 degrees. The first unmeasured free cell (-1) found is the closest one.
    for (int d = 0; d <= kSearchLimitCells; d++) {
      for (int angle_deg = 0; angle_deg < 360; angle_deg++) {
        const double angle = angle_deg * M_PI / 180;
        const int x = robot_x + static_cast<int>(std::round(d * std::cos(angle)));
        const int y = robot_y + static_cast<int>(std::round(d * std::sin(angle)));
        const std::int8_t* cell = fnCell(&measure_map_, x, y);
        if (cell == nullptr || *cell != -1) continue;

        // Cell -> world coordinates, facing the direction the cell was found in.
        goal_pose_ = scan_pose_;
        goal_pose_.header.stamp = ros::Time::now();
        goal_pose_.pose.position.x =
            x * measure_map_.info.resolution + measure_map_.info.origin.position.x;
        goal_pose_.pose.position.y =
            y * measure_map_.info.resolution + measure_map_.info.origin.position.y;
        tf2::Quaternion orientation;
        orientation.setRPY(0, 0, angle);
        goal_pose_.pose.orientation = tf2::toMsg(orientation);

        ROS_INFO("NEW GOAL: cell x: %d y: %d, angle: %d", x, y, angle_deg);
        goal_status_ = actionlib_msgs::GoalStatus::PENDING;  // Forget the previous goal's result.
        goal_sent_time_ = ros::Time::now();
        pub_goal_.publish(goal_pose_);
        return true;
      }
    }
    return false;
  }

  // HELPERS //

  // Robot pose in the "map" frame (-> scan_pose_), also published on /scan_pose.
  // Returns false (and keeps the old scan_pose_) if tf cannot provide the transform yet.
  bool fnReadScanPose() {
    // Stamp 0 means "latest available transform". A fixed old stamp would ask tf for the pose at
    // that old moment, which tf stops storing after about 10 seconds.
    base_scan_pose_.header.stamp = ros::Time(0);
    try {
      buffer_.transform(base_scan_pose_, scan_pose_, "map");
    } catch (tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(1.0, "ERROR WITH SCAN POSE: %s", ex.what());
      return false;
    }
    scan_pose_.header.stamp = ros::Time::now();
    pub_scan_pose_.publish(scan_pose_);
    return true;
  }

  // True once move_base is done with the goal, successfully or not. Still running: PENDING,
  // ACTIVE, PREEMPTING, RECALLING.
  static bool fnGoalFinished(int status) {
    using actionlib_msgs::GoalStatus;
    return status == GoalStatus::SUCCEEDED || status == GoalStatus::ABORTED ||
           status == GoalStatus::REJECTED || status == GoalStatus::PREEMPTED ||
           status == GoalStatus::RECALLED || status == GoalStatus::LOST;
  }

  // World position (metres) -> grid cell (x = column, y = row).
  static void fnWorldToGrid(const nav_msgs::OccupancyGrid& grid, const geometry_msgs::Pose& pose,
                            int* x, int* y) {
    *x = static_cast<int>(
        std::round((pose.position.x - grid.info.origin.position.x) / grid.info.resolution));
    *y = static_cast<int>(
        std::round((pose.position.y - grid.info.origin.position.y) / grid.info.resolution));
  }

  // Cell (x, y) of the grid, or nullptr if it is outside the grid.
  static std::int8_t* fnCell(nav_msgs::OccupancyGrid* grid, int x, int y) {
    if (x < 0 || y < 0 || x >= static_cast<int>(grid->info.width) ||
        y >= static_cast<int>(grid->info.height))
      return nullptr;
    return &grid->data[static_cast<std::size_t>(y) * grid->info.width + x];
  }

  // Sets to 0 ("measured") every cell within kMeasureRadiusM of (cx, cy), except obstacles (100).
  static void fnMarkMeasured(nav_msgs::OccupancyGrid* grid, int cx, int cy) {
    const int radius = static_cast<int>(std::ceil(kMeasureRadiusM / grid->info.resolution));
    for (int y = cy - radius; y <= cy + radius; y++) {
      for (int x = cx - radius; x <= cx + radius; x++) {
        std::int8_t* cell = fnCell(grid, x, y);
        const double distance = std::hypot(x - cx, y - cy) * grid->info.resolution;
        if (cell != nullptr && distance <= kMeasureRadiusM && *cell != 100) *cell = 0;
      }
    }
  }

  // DATA //
  ros::NodeHandle n_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_{buffer_};    // Must be declared after buffer_.
  geometry_msgs::PoseStamped base_scan_pose_;  // Constant: origin of the base_scan frame.
  geometry_msgs::PoseStamped scan_pose_;       // Robot pose in "map".
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
