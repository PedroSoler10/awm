/*
 * Copyright (c) 2021, Pedro Soler
 * All rights reserved.
 *
 * This file is part of awm and is licensed under the BSD 3-Clause License.
 * See the LICENSE file at the root of this repository for the full text.
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
#include <cstdio>
#include <ctime>
#include <fstream>
#include <string>

using std::string;
using std::ofstream;
using std::to_string;

class Brain {
 public:
  Brain() {
    // INITIALIZATION
    fnInit();
    ROS_INFO("INITIALIZATION DONE");
    fnWait(debug_);
    ROS_INFO("STARTING PROGRAM");

    // LOOP
    // Each pass processes callbacks, then runs one control step.
    // 10 Hz: fast enough for the controller, slow enough to keep CPU load low.
    ros::Rate loop_rate(10);
    while (ros::ok()) {
      ros::spinOnce();

      // STATE MACHINE
      control();

      loop_rate.sleep();
    }
    fnReadScanPose();
    pub_goal_.publish(scan_pose_);
  }
  void fnInit() {
    // SCAN POSE
    buffer_.setUsingDedicatedThread(true);
    base_scan_pose_.header.frame_id = "base_scan";
    base_scan_pose_.header.stamp = ros::Time::now();
    base_scan_pose_.pose.orientation.x = 0;
    base_scan_pose_.pose.orientation.y = 0;
    base_scan_pose_.pose.orientation.z = 0;
    base_scan_pose_.pose.orientation.w = 1;
    // scan_pose_.header.frame_id = "base_scan";
    pub_scan_pose_ = n_.advertise<geometry_msgs::PoseStamped>("/scan_pose", 100);

    // MAP //
    sub_map_ = n_.subscribe("/map", 100, &Brain::mapCallback, this);
    local_map_.header.frame_id = "map";
    local_map_.info.resolution = local_map_resolution_;
    local_map_.info.width = local_map_size_ + 1;
    local_map_.info.height = local_map_size_ + 1;
    local_map_.info.origin.orientation.w = 1;
    pub_local_map_ = n_.advertise<nav_msgs::OccupancyGrid>("/local_map", 100);

    // NAVIGATION //
    goal_pose_.header.frame_id = "map";
    pub_goal_ = n_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    sub_status_ = n_.subscribe("/move_base/status", 10, &Brain::statusCallback, this);
    sub_goal_ = n_.subscribe("/move_base_simple/goal", 100, &Brain::goalCallback, this);
    pub_measure_map_ = n_.advertise<nav_msgs::OccupancyGrid>("/measure_map", 100);

    // MEASURE //
  }

  void fnWait(bool d) {
    if (d) {
      printf("\nPRESS ENTER TO CONTINUE");
      char c;
      scanf("%c", &c);
    }
  }

  // SCAN POSE //
  void fnReadScanPose() {
    try {
      buffer_.transform(base_scan_pose_, scan_pose_, "map");
      scan_pose_.header.stamp = ros::Time::now();
      pub_scan_pose_.publish(scan_pose_);
    } catch (tf2::TransformException& ex) {
      ROS_INFO("ERROR WITH SCAN POSE");
      ROS_WARN("%s", ex.what());
      fnWait(debug_);
      // return;
    }
    fnPrintPose(scan_pose_.pose, "SCAN POSE");
  }
  void fnPrintPose(geometry_msgs::Pose p, const char* n) {
    ROS_INFO("PRINT POSE:\n%s POSE:\nx:%f y:%f z:%f x:%f y:%f z:%f w:%f", n, p.position.x,
             p.position.y, p.position.z, p.orientation.x, p.orientation.y, p.orientation.z,
             p.orientation.w);
  }

  // MAP //
  void mapCallback(const nav_msgs::OccupancyGrid msg) {
    // ROS_INFO("MAP CALLBACK");
    fnReadScanPose();
    if (msg.info.width != map_.info.width) {
      if (remove("map.txt") == 0)
        ROS_INFO("OLD MAP DELETED");
      else
        ROS_INFO("OLD MAP NOT DELETED");
    }
    map_ = msg;
    grid_x_ =
        round((scan_pose_.pose.position.x - map_.info.origin.position.x) / map_.info.resolution);
    grid_y_ =
        round((scan_pose_.pose.position.y - map_.info.origin.position.y) / map_.info.resolution);
    map_file_.open("map.txt");
    for (int j = map_.info.width - 1; j >= 0; j--) {
      for (int i = map_.info.height - 1; i >= 0; i--) {
        if (map_.data[i * map_.info.width + j] == 100)
          map_file_ << "*\t";
        else if (map_.data[i * map_.info.width + j] == -1)
          map_file_ << "·\t";
        else if (map_.data[i * map_.info.width + j] == 0)
          map_file_ << "+\t";
      }
      map_file_ << "\n";
    }
    map_file_.close();
    // ROS_INFO("MAP CALLBACK:\nGRID CELLS:\nx: %d y: %d",grid_x_, grid_y_);
  }
  void fnReadLocalMap() {
    local_map_.info.origin.position.x =
        scan_pose_.pose.position.x - (local_map_.info.width - 1) / 2 * local_map_.info.resolution;
    local_map_.info.origin.position.y =
        scan_pose_.pose.position.y - (local_map_.info.height - 1) / 2 * local_map_.info.resolution;

    if ((grid_x_ < map_.info.width) && (grid_y_ < map_.info.height)) {
      ROS_INFO("READ LOCAL MAP:");
      local_map_file_.open("local_map.txt");
      int k = grid_y_ - local_map_.info.height / 2;
      int l = grid_x_ - local_map_.info.width / 2;
      for (int i = 0; i < local_map_.info.height; i++) {
        for (int j = 0; j < local_map_.info.width; j++) {
          local_map_.data[i * local_map_.info.width + j] = map_.data[k * map_.info.width + l];
          local_map_file_ << int(local_map_.data[i * local_map_.info.width + j]) << "\t\t";
          l++;
        }
        local_map_file_ << "\n";
        l = grid_x_ - local_map_.info.width / 2;
        k++;
      }
      local_map_file_.close();
      pub_local_map_.publish(local_map_);
    }
  }

  // NAVIGATION //
  void fnPubGoal(int mode) {
    if (goal_published_ == false) {
      ROS_INFO("IDENTIFYING GOAL");
      if (mode == 0) {
      } else if (mode == 1) {
        goal_pose_ = scan_pose_;
        if (flag_)
          goal_pose_.pose.position.x += 1;
        else
          goal_pose_.pose.position.x -= 0.75;
        flag_ = !flag_;
        goal_pose_.pose.orientation.x = 0;
        goal_pose_.pose.orientation.y = 0;
        goal_pose_.pose.orientation.z = 0;
        goal_pose_.pose.orientation.w = 1;
        pub_goal_.publish(goal_pose_);
        goal_published_ = true;
      } else if (mode == 2) {
        int r_min = 0;
        int r_max = 0;
        int r_step = 10;
        int r_lim = 200;
        int i;
        int j;
        int d;
        int a;
        int i_min = grid_y_;
        int j_min = grid_x_;
        int d_min = r_lim + 1;
        int a_min = 0;

        while (d_min > r_max) {
          r_min = r_max;
          r_max += r_step;
          d = r_min;
          do {
            a = 0;
            do {
              i = round(grid_y_ + d * sin(a * PI / 180));
              j = round(grid_x_ + d * cos(a * PI / 180));
              if ((i >= measure_map_.info.height) || (j >= measure_map_.info.width) || (i < 0) ||
                  (j < 0)) {
                i = grid_y_;
                j = grid_x_;
              }
              a++;
              // ROS_INFO("GOAL OPTION:\nX: %d Y: %d DATA: %d", j, i,
              // measure_map_.data[i*measure_map_.info.width + j]);
            } while ((measure_map_.data[i * measure_map_.info.width + j] != -1) && (a < 360));
            d++;
          } while ((measure_map_.data[i * measure_map_.info.width + j] != -1) && (d < r_max));
          a--;
          d--;
          if (measure_map_.data[i * measure_map_.info.width + j] == -1) {
            i_min = i;
            j_min = j;
            d_min = d;
            a_min = a;
            ROS_INFO("I_MIN: %d J_MIN: %d D_MIN: %d A_MIN: %d ", i_min, j_min, d_min, a_min);
          }
        }
        if ((i_min == grid_x_) && (j_min == grid_y_) && (d_min == r_lim + 1) && (a_min == 0)) {
          ROS_INFO("EVERYTHING MEASURED");
        } else {
          goal_pose_ = scan_pose_;
          goal_pose_.pose.position.x =
              j_min * measure_map_.info.resolution + measure_map_.info.origin.position.x;
          goal_pose_.pose.position.y =
              i_min * measure_map_.info.resolution + measure_map_.info.origin.position.y;
          tf2::Quaternion goal_angle_tf;
          goal_angle_tf.setRPY(0, 0, a * PI / 180);
          geometry_msgs::Quaternion goal_angle;
          goal_angle = tf2::toMsg(goal_angle_tf);
          goal_pose_.pose.orientation.x = goal_angle.x;
          goal_pose_.pose.orientation.y = goal_angle.y;
          goal_pose_.pose.orientation.z = goal_angle.z;
          goal_pose_.pose.orientation.w = goal_angle.w;
          pub_goal_.publish(goal_pose_);
          goal_published_ = true;
        }
      }
    } else {
      ROS_INFO("GOAL ALREADY PUBLISHED");
    }
  }
  void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& st) {
    if (!st->status_list.empty()) {
      if ((st->status_list[1].status >= 0) && (st->status_list[1].status <= 9) &&
          (st->status_list[0].goal_id.stamp.sec < st->status_list[1].goal_id.stamp.sec) &&
          (st->status_list[0].goal_id.stamp.nsec < st->status_list[1].goal_id.stamp.nsec)) {
        if ((st->status_list[2].status >= 0) && (st->status_list[2].status <= 9) &&
            (st->status_list[1].goal_id.stamp.sec < st->status_list[2].goal_id.stamp.sec) &&
            (st->status_list[1].goal_id.stamp.nsec < st->status_list[2].goal_id.stamp.nsec)) {
          goal_status_ = st->status_list[2].status;
        } else {
          goal_status_ = st->status_list[1].status;
        }
      } else {
        goal_status_ = st->status_list[0].status;
      }
      if (goal_status_ < 3) goal_published_ = true;
      // ROS_INFO("STATUS  : %d",goal_status_);
      // ROS_INFO("STATUS 0: %d TIME: %d.%d", st->status_list[0].status,
      // st->status_list[0].goal_id.stamp.sec, st->status_list[0].goal_id.stamp.nsec);
      // ROS_INFO("STATUS 1: %d TIME: %d.%d", st->status_list[1].status,
      // st->status_list[1].goal_id.stamp.sec, st->status_list[1].goal_id.stamp.nsec);
      // ROS_INFO("STATUS 2: %d TIME: %d.%d", st->status_list[2].status,
      // st->status_list[2].goal_id.stamp.sec, st->status_list[2].goal_id.stamp.nsec);
    }
  }
  void goalCallback(const geometry_msgs::PoseStamped msg) {
    /*if ((goal_pose_.pose.position.x != msg.pose.position.x) || (goal_pose_.pose.position.y !=
    msg.pose.position.y))
    {
      goal_pose_ = msg;
      goal_published_ = true;
      new_goal_ = true;
    }
    else
      new_goal_ = false;*/
  }

  // MEASURE //
  void fnMeasure() {
    ros::Duration d(3.0);
    ros::Time n = ros::Time::now();

    if ((n - mt_) < d) {
      measure_done_ = false;
    } else {
      // Local wall-clock time, e.g. "2026-10-21T10-30-05".
      // Colons are avoided so the file name also works on Windows.
      time_t now = time(nullptr);
      tm local_time;
      localtime_r(&now, &local_time);
      char time_buffer[32];
      strftime(time_buffer, sizeof(time_buffer), "%Y-%m-%dT%H-%M-%S", &local_time);
      string iso_time_str = time_buffer;
      fnReadScanPose();
      ROS_INFO("MEASURE DATA:\nx: %f y: %f t: %s", scan_pose_.pose.position.x,
               scan_pose_.pose.position.y, iso_time_str.c_str());
      string f_n = to_string(scan_pose_.pose.position.x) + "_" +
                   to_string(scan_pose_.pose.position.y) + "_" + iso_time_str + ".txt";
      ofstream m_f;
      m_f.open(f_n);
      m_f.close();
      fnUpdateMeasureMap();
      measure_done_ = true;
    }
  }
  void fnUpdateMeasureMap() {
    measure_map_.header.stamp = ros::Time::now();
    double distance;
    float measure_ratio = 0.5;
    ROS_INFO("UPDATE MEASURE MAP");
    if (measure_map_.info.width != map_.info.width) {
      if (remove("measure_map.txt") == 0)
        ROS_INFO("OLD MEASURE MAP DELETED");
      else
        ROS_INFO("OLD MEASURE MAP NOT DELETED");
      measure_map_ = map_;
      for (int i = 0; i < measure_map_.info.height; i++) {
        for (int j = 0; j < measure_map_.info.width; j++) {
          if ((map_.data[i * map_.info.width + j] == -1) ||
              (map_.data[i * map_.info.width + j] > 90))
            measure_map_.data[i * measure_map_.info.width + j] = 100;
          else
            measure_map_.data[i * measure_map_.info.width + j] = -1;
        }
      }
    }
    ROS_INFO("READ MEASURE MAP:");
    for (int i = 0; i < measure_map_.info.height; i++) {
      for (int j = 0; j < measure_map_.info.width; j++) {
        distance = sqrt(pow(grid_y_ - i, 2) + pow(grid_x_ - j, 2)) * measure_map_.info.resolution;
        if ((map_.data[i * map_.info.width + j] == -1) || (map_.data[i * map_.info.width + j] > 90))
          measure_map_.data[i * measure_map_.info.width + j] = 100;
        else if (distance <= measure_ratio)
          measure_map_.data[i * measure_map_.info.width + j] = 0;
        else if (measure_map_.data[i * measure_map_.info.width + j] != 0)
          measure_map_.data[i * measure_map_.info.width + j] = -1;
      }
    }
    int radius = 6;
    for (int i = 0; i < measure_map_.info.height; i++) {
      for (int j = 0; j < measure_map_.info.width; j++) {
        if (map_.data[i * map_.info.width + j] > 90) {
          for (int k = i - radius; k <= i + radius; k++) {
            for (int l = j - radius; l <= j + radius; l++) {
              if ((k < measure_map_.info.height) && (l < measure_map_.info.width) && (k > 0) &&
                  (l > 0) && (measure_map_.data[k * measure_map_.info.width + l] != 0))
                measure_map_.data[k * measure_map_.info.width + l] = 100;
            }
          }
        }
      }
    }
    measure_map_file_.open("measure_map.txt");
    if (measure_map_file_.is_open()) {
      ROS_INFO("MEASURE MAP FILE OPENED");
      for (int j = measure_map_.info.width - 1; j >= 0; j--) {
        for (int i = measure_map_.info.height - 1; i >= 0; i--) {
          if (measure_map_.data[i * measure_map_.info.width + j] == 100)
            measure_map_file_ << "*\t";
          else if (measure_map_.data[i * measure_map_.info.width + j] == -1)
            measure_map_file_ << " \t";
          else if (measure_map_.data[i * measure_map_.info.width + j] == 0)
            measure_map_file_ << "·\t";
        }
        measure_map_file_ << "\n";
      }
      measure_map_file_.close();
    } else {
      ROS_INFO("ERROR WITH MAP FILE");
    }
    pub_measure_map_.publish(measure_map_);
  }

  // CONTROL //
  void control() {
    if (sequence_ == 0) {
      ROS_INFO("WAITING");
      //
      // fnPrintPose(scan_pose_.pose, "SCAN");
      ROS_INFO("SCAN POSE HEADER FRAME ID: %s", scan_pose_.header.frame_id.c_str());
      if (scan_pose_.header.frame_id == "map") {
        fnPrintPose(scan_pose_.pose, "SCAN");
        mt_ = ros::Time::now();
        fnMeasure();
        sequence_ = 3;
        ROS_INFO("DOING MEASURING");
        fnWait(debug_);
      }
    }
    if (sequence_ == 1) {  // Reading Goal
      if (goal_published_ == true) {
        ROS_INFO("GOAL PUBLISHED");
        fnPrintPose(goal_pose_.pose, "GOAL");
        fnPrintPose(scan_pose_.pose, "SCAN");
        sequence_ = 2;
        ROS_INFO("REACHING GOAL");
        goal_published_ = false;
        new_goal_ = false;
        fnWait(debug_);
      } else {
        fnPubGoal(2);
      }
    } else if (sequence_ == 2) {  // Reaching Goal
      if (goal_status_ == 3) {
        ROS_INFO("GOAL REACHED");
        mt_ = ros::Time::now();
        fnMeasure();
        sequence_ = 3;
        ROS_INFO("DOING MEASURING");
        fnWait(debug_);
      } else if (goal_status_ == 4) {
        ROS_INFO("GOAL IMPOSSIBLE TO REACH");
        mt_ = ros::Time::now();
        fnMeasure();
        sequence_ = 3;
        ROS_INFO("DOING MEASURING");
        fnWait(debug_);
      }
    } else if (sequence_ == 3) {  // Measuring
      if (measure_done_) {
        ROS_INFO("MEASURE DONE");
        sequence_ = 1;
        ROS_INFO("SEARCHING NEW GOAL");
        fnWait(debug_);
      } else {
        fnMeasure();
      }
    }
  }

 private:
  ros::NodeHandle n_;

  // SCAN POSE
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_{buffer_};
  geometry_msgs::PoseStamped base_scan_pose_;
  geometry_msgs::PoseStamped scan_pose_;
  ros::Publisher pub_scan_pose_;

  // MAP
  ros::Subscriber sub_map_;
  nav_msgs::OccupancyGrid map_;
  int grid_x_ = 0;
  int grid_y_ = 0;
  nav_msgs::OccupancyGrid local_map_;
  float local_map_resolution_ = 0.05;
  int local_map_size_ = 60;
  ofstream map_file_;
  ofstream local_map_file_;
  ros::Publisher pub_local_map_;

  // NAVIGATION
  geometry_msgs::PoseStamped goal_pose_;
  ros::Publisher pub_goal_;
  ros::Subscriber sub_status_;
  int goal_status_ = 0;
  ros::Subscriber sub_goal_;
  bool goal_published_ = false;
  bool flag_ = true;
  static constexpr double PI = 3.14159265358979323846;

  // MEASURE
  ros::Time mt_{0, 0};
  nav_msgs::OccupancyGrid measure_map_;
  ros::Publisher pub_measure_map_;
  ofstream measure_map_file_;

  // CONTROL
  int sequence_ = 0;
  bool new_goal_ = false;
  bool measure_done_ = false;
  bool debug_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "brain");
  Brain brain;
  return 0;
}
