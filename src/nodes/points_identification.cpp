/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
 */

/*
 * points_identification: serves goal points for the robot.
 *
 * Subscribes: /map          (nav_msgs/OccupancyGrid)  SLAM map
 *             /safe_map     (nav_msgs/OccupancyGrid)  from map_processing, to keep goals off walls
 *             /measure_map  (nav_msgs/OccupancyGrid)  from brain, what has been measured
 * Service:    /goal_points  (awm/GoalPoints)
 *   mode 1 EXPLORATION  frontier goals: one per cluster of free cells at the edge of the known map
 *   mode 2 MEASUREMENT  the nearest cells of /measure_map still to be measured
 *   mode 3 FUSION       MEASUREMENT first; if it finds nothing, EXPLORATION
 *   num: most goals returned, 0 = all. Points are in the "map" frame, closest to the robot first.
 *   An unknown mode is an error. Missing maps or robot pose give an empty list.
 * Private params (defaults in brackets):
 *   ~min_cluster_size [5] frontiers with fewer cells are noise, ~search_limit_cells [200] farthest
 *   unmeasured cell searched from the robot, ~dump_maps [false] write map_with_goals.txt on every
 *   request (goals marked 8), ~dump_dir [.]
 */

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>

#include "awm/GoalPoints.h"
#include "awm/frontiers.h"
#include "awm/grid.h"
#include "awm/map_dump.h"
#include "awm/measure_map.h"
#include "awm/robot_pose.h"

class PointsIdentification {
 public:
  // The constructor only sets up. The work happens in getGoalPoints().
  PointsIdentification() {
    ros::NodeHandle pnh("~");
    pnh.param("min_cluster_size", min_cluster_size_, 5);
    pnh.param("search_limit_cells", search_limit_cells_, 200);
    pnh.param("dump_maps", dump_maps_, false);
    pnh.param("dump_dir", dump_dir_, std::string("."));

    sub_map_ = nh_.subscribe("/map", 1, &PointsIdentification::mapCallback, this);
    sub_safe_map_ = nh_.subscribe("/safe_map", 1, &PointsIdentification::safeMapCallback, this);
    sub_measure_map_ =
        nh_.subscribe("/measure_map", 1, &PointsIdentification::measureMapCallback, this);
    srv_goal_points_ =
        nh_.advertiseService("/goal_points", &PointsIdentification::getGoalPoints, this);
    ROS_INFO("INITIALIZATION DONE");
  }

 private:
  // The maps are only read, so we keep the shared message instead of copying it.
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) { map_ = msg; }
  void safeMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) { safe_map_ = msg; }
  void measureMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) { measure_map_ = msg; }

  // Returning false makes the caller's service call fail: only for a request that cannot be
  // answered (bad mode). A request that is fine but has no answer yet gets an empty list. The
  // non-const references are the signature advertiseService requires.
  bool getGoalPoints(awm::GoalPoints::Request& req,     // NOLINT(runtime/references)
                     awm::GoalPoints::Response& res) {  // NOLINT(runtime/references)
    using Request = awm::GoalPoints::Request;
    if (req.mode != Request::EXPLORATION && req.mode != Request::MEASUREMENT &&
        req.mode != Request::FUSION) {
      ROS_WARN("GOAL POINTS: UNKNOWN MODE %d (1 EXPLORATION, 2 MEASUREMENT, 3 FUSION)", req.mode);
      return false;
    }

    geometry_msgs::PoseStamped robot;
    if (!robot_pose_.lookup(&robot)) {
      ROS_WARN_THROTTLE(5.0, "GOAL POINTS: NO ROBOT POSE YET");
      return true;
    }

    if (req.mode != Request::EXPLORATION) {
      if (!measurementPoints(robot, req.num, &res.points) && req.mode == Request::MEASUREMENT)
        ROS_WARN_THROTTLE(5.0, "GOAL POINTS: NO /measure_map YET");
    }
    if (res.points.empty() && req.mode != Request::MEASUREMENT) {
      if (!explorationPoints(robot, req.num, &res.points))
        ROS_WARN_THROTTLE(5.0, "GOAL POINTS: NO USABLE /map AND /safe_map YET");
    }

    ROS_INFO("GOAL POINTS: mode %d, %zu points", req.mode, res.points.size());
    if (dump_maps_ && map_) dumpGoals(res.points);
    return true;
  }

  // Frontier goals. Returns false if /map and /safe_map are not both there and of the same size
  // (map_processing may be a moment behind a map that just grew).
  bool explorationPoints(const geometry_msgs::PoseStamped& robot, int num,
                         std::vector<geometry_msgs::Point>* points) const {
    if (!map_ || !safe_map_) return false;
    std::vector<awm::GridCell> frontier_cells;
    if (!awm::findFrontierCells(*map_, *safe_map_, &frontier_cells)) return false;

    std::vector<std::vector<awm::GridCell>> clusters;
    awm::clusterFrontiers(frontier_cells, map_->info.width, min_cluster_size_, &clusters);

    int robot_x;
    int robot_y;
    awm::worldToGrid(*map_, robot.pose, &robot_x, &robot_y);
    std::vector<awm::GridCell> goals;
    awm::frontierGoals(clusters, robot_x, robot_y, &goals);
    appendWorldPoints(*map_, goals, num, points);
    return true;
  }

  // Nearest cells still to be measured. Returns false if /measure_map has not been received.
  bool measurementPoints(const geometry_msgs::PoseStamped& robot, int num,
                         std::vector<geometry_msgs::Point>* points) const {
    if (!measure_map_) return false;
    int robot_x;
    int robot_y;
    awm::worldToGrid(*measure_map_, robot.pose, &robot_x, &robot_y);
    std::vector<awm::GridCell> cells;
    awm::findNearestCells(*measure_map_, robot_x, robot_y, awm::kToMeasure, search_limit_cells_,
                          num, &cells);
    appendWorldPoints(*measure_map_, cells, num, points);
    return true;
  }

  // Cell -> world position of the cell centre, the same convention brain uses for its goals.
  static void appendWorldPoints(const nav_msgs::OccupancyGrid& grid,
                                const std::vector<awm::GridCell>& cells, int num,
                                std::vector<geometry_msgs::Point>* points) {
    const std::size_t count = num == 0 ? cells.size() : std::min(cells.size(), std::size_t(num));
    for (std::size_t i = 0; i < count; i++) {
      geometry_msgs::Point point;
      awm::gridToWorld(grid, cells[i].x, cells[i].y, &point.x, &point.y);
      points->push_back(point);
    }
  }

  // Text picture of /map with the goals drawn as 8, for looking at what the service answered.
  void dumpGoals(const std::vector<geometry_msgs::Point>& points) const {
    std::vector<awm::GridCell> marks;
    for (const geometry_msgs::Point& point : points) {
      geometry_msgs::Pose pose;
      pose.position = point;
      awm::GridCell mark;
      awm::worldToGrid(*map_, pose, &mark.x, &mark.y);
      marks.push_back(mark);
    }
    const std::string path = dump_dir_ + "/map_with_goals.txt";
    if (!awm::writeMapText(*map_, path, marks)) ROS_WARN("COULD NOT WRITE %s", path.c_str());
  }

  ros::NodeHandle nh_;
  int min_cluster_size_;
  int search_limit_cells_;
  bool dump_maps_;
  std::string dump_dir_;

  awm::RobotPose robot_pose_;
  nav_msgs::OccupancyGrid::ConstPtr map_;
  nav_msgs::OccupancyGrid::ConstPtr safe_map_;
  nav_msgs::OccupancyGrid::ConstPtr measure_map_;
  ros::Subscriber sub_map_;
  ros::Subscriber sub_safe_map_;
  ros::Subscriber sub_measure_map_;
  ros::ServiceServer srv_goal_points_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "points_identification");
  PointsIdentification points_identification;
  ros::spin();
  return 0;
}
