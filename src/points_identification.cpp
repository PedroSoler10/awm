#include "ros/ros.h"
#include "stdio.h"
#include <string>
#include <math.h>
#include "fstream"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "boost/date_time/posix_time/posix_time.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "awm/GoalPoints.h"
#define PI 3.14159265

class PointsIdentification
{
  public:
  
    PointsIdentification()  
    {
      // INITIALIZATION //
      fnInit();
      ROS_INFO("INITIALIZATION DONE");
      fnWait(debug);
      
      // LOOP //
      ros::Rate loop_rate(10);
      
      while ( ros::ok())
      {
        // STATE MACHINE
        ros::spinOnce();
        loop_rate.sleep();
      }
    }
    
    void fnInit()
    {
      // SCAN POSE //
      sub_scan_pose = n.subscribe("/scan_pose", 100, &PointsIdentification::scanPoseCallback, this);
      
      // MAP //
      sub_map = n.subscribe("/map", 100, &PointsIdentification::mapCallback, this);

      // SAVE MAP //
      sub_safe_map = n.subscribe("/safe_map", 100, &PointsIdentification::safeMapCallback, this);
      //pub_safe_points = n.advertise<geometry_msgs::PointStamped>("/safe_points", 10);
      
      // TASK //
      sub_task_map = n.subscribe("/task_map", 100, &PointsIdentification::taskMapCallback, this);
      //pub_task_points = n.advertise<geometry_msgs::pointStamped>("/task_points", 10);
      
      // POINTS IDENTIFICATION //
      //srv_goal_points = n.advertiseService("goal_points", getGoalPoints);
    }
    
    // SCAN POSE //
    void scanPoseCallback(geometry_msgs::PoseStamped sp)
    {
      scan_pose = sp;
      grid_x = round((scan_pose.pose.position.x - map.info.origin.position.x) / map.info.resolution);
      grid_y = round((scan_pose.pose.position.y - map.info.origin.position.y) / map.info.resolution);
    }
    
    // MAP //
    void mapCallback(const nav_msgs::OccupancyGrid m)
    {
      map = m;
      getExplorePoints(map, 100);
      writeGoals(map, "map_with_goals.txt", x_e, y_e, 100);
    }
    
    // EXPLORE MAP //
    void safeMapCallback(const nav_msgs::OccupancyGrid sm)
    {
      safe_map = sm;
    }
      
    void writeGoals(nav_msgs::OccupancyGrid m, const char* s, int * x, int * y, int size)
    {
      std::ofstream m_f;
      m_f.open(s);
      for(int j = m.info.width-1; j >= 0; j--)
      {
        for(int i = m.info.height-1; i >= 0; i--)
        {
          if (m.data[i*m.info.width + j] >= 90)
            m_f<<"*\t";
          else if (m.data[i*m.info.width + j] >= 50)
            m_f<<"+\t";
          else if (m.data[i*m.info.width + j] == -1)
            m_f<<"·\t";
          else if (m.data[i*m.info.width + j] < 50)
            m_f<<" \t";
          for(int k = 0; k < size; k++)
          {
            if ((i == y[k]) && (j == x[k]))
              m_f<<"8\t";
          }
        }
        m_f <<"\n";
      }
      m_f.close();
    }
    
    // TASK MAP //
    void taskMapCallback(const nav_msgs::OccupancyGrid tm)
    {
      task_map = tm;
    }
    
    // GOAL POINTS IDENTIFICATION //
    //void getGoalPoints(GoalPoints &gp){}
    /*void getGoalPoints(awm::GoalPoints::Request &req, awm::GoalPoints::Response &res)
    {
      if (req.mode == 1)
        getExplorePoints(map, goal_points_list, req.num);
      else if (req.mode == 2)
        getTaskPoints();
      else if (req.mode == 3)
        getFusionPoints();
      else
        ROS_INFO("ERROR:\nMODE = %d", m);
      res.goal_points_list = goal_points_list;
    }*/
    
    // EXPLORE POINTS IDENTIFICATION //
    void getExplorePoints(const nav_msgs::OccupancyGrid m, int n)
    {
      detectFrontiers(m, x_e, y_e, dist_e, size_e);
      //orderMaxToMin(x_e, y_e, dist_e, size_e, n);
      //setGoalPointsList(x_e, y_e, n, gpl);
    }
    
    // DETECTING FRONTIERS //
    void detectFrontiers(const nav_msgs::OccupancyGrid m,  int * x, int * y, double * d, int & s)
    {
    int max_size = m.info.width * m.info.height;
    int c_x [max_size] = {};
    int c_y [max_size] = {};
    double c_d[max_size] = {};
    int c_f = 0;
      for(int i = 0 ; i < m.info.height; i++)
      {
        for(int j = 0 ; j < m.info.width; j++)
        {
          if(m.data[i*m.info.width + j] == -1)
          {
            for(int k = i-1; k < i+1; k++)
            {
              for(int l = j-1; l < j+1; l++)
              {
                if((k > 0) && (k < m.info.height) && (l > 0) && (l < m.info.width) && (m.data[k*m.info.width + l] < 80))
                {
                  c_y[c_f] = i;
                  c_x[c_f] = j;
                  c_d[c_f] = sqrt(pow(grid_y-c_y[c_f],2)+pow(grid_x-c_x[c_f],2))*m.info.resolution;
                  c_f++;
                  k = i+1;
                  l = j+1;
                  ROS_INFO("x: %d, y: %d", j, i);
                }
              }
            }
          }
        }
      }  

    s = c_f;
    x = c_x;
    y = c_y;
    d = c_d;
    }
    
    void orderMaxToMin(int * x, int * y,  double * d, int s, int n)
    {
      double c_d[n] = {};
      int c_x[n] = {};
      int c_y[n] = {};
      double d_min = 100;
      int j_min;
      for(int i = 0 ; i < n ; i++)
      {
       d_min = 100;
       for(int j = 0 ; j < s ; j++)
        {
          if(d[j] < d_min)
          {
            j_min = j;
            d_min = c_d[j];
          }
        }
      c_x[i] = x[j_min];
      c_y[i] = y[j_min];
      c_d[i] = d[j_min];
      d[j_min] = 100;
      }
    ROS_INFO("ANTES");
    x = c_x;
    y = c_y;
    d = c_d;
    ROS_INFO("DESPUÉS");
    }

    void setGoalPointsList(int * x, int * y, int n, geometry_msgs::PointStamped* gpl)
    {
      geometry_msgs::PointStamped c_gpl[n];
      
      for(int i = 0; i < n; i++)
      {
        //c_gpl[i].header.seq = i;
        c_gpl[i].header.frame_id = "map";
        c_gpl[i].header.stamp = ros::Time::now();
        c_gpl[i].point.x = x[i];
        c_gpl[i].point.y = y[i];
        c_gpl[i].point.z = 0;
      }
      gpl = c_gpl;
    }

    // TASK POINTS IDENTIFICATION //
    void getTaskPoints()
    {
      
    }
    
    void fnWait(bool d)
    {
      if(d)
      {
        printf("\nPRESS ENTER TO CONTINUE");
        char c;
        scanf("%c",&c);
      }
    }
    
    void fnInitStampedPose(geometry_msgs::PoseStamped &p, const char* s)
    {
      p.header.frame_id = s;
      p.header.stamp = ros::Time::now();
      fnInitPose(p.pose);
    }
    
    void fnInitPose(geometry_msgs::Pose &p)
    {
      p.position.x = 0;
      p.position.y = 0;
      p.position.z = 0;
      p.orientation.x = 0;
      p.orientation.y = 0;
      p.orientation.z = 0;
      p.orientation.w = 1;
    }
    
    void fnPrintStampedPose(geometry_msgs::PoseStamped p, const char* s)
    {
      ROS_INFO("PRINT STAMPED POSE: %s \n\tFRAME_ID: %s \n\tSTAMP: \n\t\tsec: %d \n\t\tnsec: %d",
          s,
          p.header.frame_id.c_str(),
          p.header.stamp.sec,
          p.header.stamp.nsec);
       fnPrintPose(p.pose, s);
    }
    
    void fnPrintPose(geometry_msgs::Pose p, const char* s)
    {
      ROS_INFO("PRINT POSE: %s \nPOSE: \n\tPOSITION: \n\t\tx:%f \n\t\ty:%f \n\t\tz:%f \n\tORIENTATION: \n\t\tx:%f \n\t\ty:%f \n\t\tz:%f \n\t\tw:%f",
          s,
          p.position.x,
          p.position.y,
          p.position.z,
          p.orientation.x,
          p.orientation.y,
          p.orientation.z,
          p.orientation.w);
    }
    
    void fnDeleteFile(const char* s)
    {
      if (remove(s) == 0)
        ROS_INFO("%s DELETED",s);
      else
        ROS_INFO("%s NOT DELETED",s);
    }
  private:
    ros::NodeHandle n;
    
    // SCAN POSE //
    geometry_msgs::PoseStamped scan_pose;
    ros::Subscriber sub_scan_pose;
    
    // MAP //
    nav_msgs::OccupancyGrid map;
    ros::Subscriber sub_map;
    int grid_x;
    int grid_y;
    int size;
    
    // EXPLORE MAP //
    nav_msgs::OccupancyGrid safe_map;
    ros::Subscriber sub_safe_map;
    geometry_msgs::PoseStamped safe_pose;
    int size_e;
    int * x_e;
    int * y_e;
    double * dist_e;
    
    // TASK MAP //
    nav_msgs::OccupancyGrid task_map;
    ros::Subscriber sub_task_map;
    geometry_msgs::PoseStamped task_pose;

    //int m_grid_x;
    //int m_grid_y;
    
    // POINTS IDENTIFICATION //
    ros::ServiceServer srv_goal_points;
    geometry_msgs::PointStamped * goal_points_list;
    
    // CONTROL //
    bool debug = false;
};



// MAIN AND GENERAL FUNCTIONS //
int main(int argc, char **argv)
{
  ros::init(argc, argv, "points_identification");
  PointsIdentification pointsIdentification;
  ros::spin();
  return 0;
}



/*void fnReadLocalMap()
    {
      task_map.info.origin.position.x = scan_pose.pose.position.x - (task_map.info.width-1) / 2*task_map.info.resolution;
      task_map.info.origin.position.y = scan_pose.pose.position.y - (task_map.info.height-1)/ 2*task_map.info.resolution;
      
      if((grid_x<map.info.width)&&(grid_y<map.info.height))
      {
        //ROS_INFO("READ LOCAL MAP:");
        task_map_file.open ("task_map.txt");
        int k = grid_y-task_map.info.height/2;
        int l = grid_x-task_map.info.width/2;
        for(int i = 0 ; i < task_map.info.height ; i++)
        {
          for(int j = 0 ; j < task_map.info.width; j++)
          {
            task_map.data[i*task_map.info.width + j] = map.data[k*map.info.width + l];
            task_map_file<<int(task_map.data[i*task_map.info.width + j])<<"\t\t";
            l++;
          }
          task_map_file <<"\n";
          l = grid_x-task_map.info.width/2;
          k++;
        }
        task_map_file.close();
        pub_task_map.publish(task_map);
      }
    }*/
