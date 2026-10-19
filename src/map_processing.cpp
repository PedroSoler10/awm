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
#define PI 3.14159265

class MapProcessing
{
  public:
  
    MapProcessing() :
    tf2_(buffer_)    
    {
      // INITIALIZATION
      init();
      //ROS_INFO("INITIALIZATION DONE");
      wait(debug);
      //ROS_INFO("STARTING PROGRAM");
      //ros::Time timer = ros::Time::now();
      // LOOP
      ros::Rate loop_rate(10);
      
      while ( ros::ok())// && ( ( ros::Time::now().toSec() - timer.toSec() ) < 5 ) )
      {
        // STATE MACHINE
        //ROS_INFO("LOOP");
        ros::spinOnce();
        loop_rate.sleep();
      }
    }
    
    void init()
    {
      // SCAN POSE
      buffer_.setUsingDedicatedThread(true);
      initStampedPose(base_scan_pose, "base_scan");
      initStampedPose(scan_pose, "map");
      pub_scan_pose = n.advertise<geometry_msgs::PoseStamped>("/scan_pose", 10);
      
      // MAP //
      deleteFile("map.txt");
      sub_map = n.subscribe("/map", 100, &MapProcessing::mapCallback, this);
      
      // SAVE MAP //
      deleteFile("safe_map.txt");
      pub_safe_map = n.advertise<nav_msgs::OccupancyGrid>("/safe_map", 10);
      
      // TASK //
      deleteFile("task_map.txt");
      pub_task_map = n.advertise<nav_msgs::OccupancyGrid>("/task_map", 10);
    }
    
    // SCAN POSE //
    void readScanPose()
    {
      try
      {
        buffer_.transform(base_scan_pose, scan_pose, "map");
        scan_pose.header.stamp = ros::Time::now();
        //PrintStampedPose(scan_pose, "SCAN_POSE");
        pub_scan_pose.publish(scan_pose);
        //grid_x = round((scan_pose.pose.position.x - map.info.origin.position.x) / map.info.resolution);
        //grid_y = round((scan_pose.pose.position.y - map.info.origin.position.y) / map.info.resolution);
        //ROS_INFO("GRID CELLS:\nx: %d y: %d",grid_x, grid_y);
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
      }
    }
    
    // MAP //
    void mapCallback(const nav_msgs::OccupancyGrid m)
    {
      // MAP //
      map = m;
      writeMap(map, "map.txt");
      readScanPose();
      
      // SAFE_MAP //
      safe_map = map;
      discretiseMap(map, safe_map);
      safeProcessing(map, safe_map);
      writeMap(safe_map, "safe_map.txt");
      pub_safe_map.publish(safe_map);
      
      // TASK_MAP //
      /*if (new_map)
      {
        task_map = safe_map;
        initTaskMap(safe_map, task_map);
        new_map = false;
      }
      task_map = safe_map;
      initTaskMap(safe_map, task_map);
      updateTaskMap(safe_map, task_map);
      task_pose = scan_pose;
      updateTask(task_pose.pose, task_map);
      updateTaskMap(safe_map, task_map);
      writeMap(task_map, "task_map.txt");
      pub_task_map.publish(task_map);*/
    }
    
    void writeMap(nav_msgs::OccupancyGrid m, const char* s)
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
        }
        m_f <<"\n";
      }
      m_f.close();
    }
    
    // SAFE MAP //
    void discretiseMap(nav_msgs::OccupancyGrid m, nav_msgs::OccupancyGrid &s_m)
    {
      for(int i = 0 ; i < m.info.height ; i++)
      {
        for(int j = 0 ; j < m.info.width; j++)
        {
          if (m.data[i*m.info.width + j] >= 90)
            s_m.data[i*s_m.info.width + j] = 100; 
          else if (m.data[i*m.info.width + j] >= 50)
            s_m.data[i*s_m.info.width + j] = 50;
          else if (m.data[i*m.info.width + j] >= 0)
            s_m.data[i*s_m.info.width + j] = 0;
          else
            s_m.data[i*s_m.info.width + j] = -1;
        }
      }
    }
    
    // MAP PROCESSING //
    void safeProcessing(nav_msgs::OccupancyGrid m, nav_msgs::OccupancyGrid &p_m)
    {
      int radius = 6;
      for(int i = 0 ; i < p_m.info.height ; i++)
      {
        for(int j = 0 ; j < p_m.info.width; j++)
        {
          if (m.data[i*m.info.width + j] >= 90)
          {
            for(int k = i-radius; k <= i+radius; k++)
            {
              for(int l = j-radius; l <= j+radius; l++)
              {
                if((k > 0) && (k < p_m.info.height) && (l > 0) && (l < p_m.info.width) && ((m.data[k*m.info.width + l] < 50)))
                {
                  p_m.data[k*p_m.info.width + l] = 50;
                  //ROS_INFO("k: %d l: %d", k, l);
                }
              }
            }
          }
        }
      }
    }
    
    /*// TASK //
    void task()
    {
      ros::Duration d(3.0);
      ros::Time n = ros::Time::now();
      
      if ((n - mt) < d)
        task_done = false;
      else
      {
        ros::WallDuration two_hours = ros::WallDuration(2*60*60);
        ros::WallTime n = ros::WallTime::now() + two_hours;
        boost::posix_time::ptime my_posix_time = n.toBoost();
        std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);
        readScanPose();
        //ROS_INFO("TASK DATA:\nx: %f y: %f z: %f t: %s", 
          scan_pose.pose.position.x,
          scan_pose.pose.position.y,
          scan_pose.pose.position.z,
          iso_time_str.c_str());
        updateTaskMap();
        task_done = true;
      }
    }*/
    
    void initTaskMap(nav_msgs::OccupancyGrid s_m, nav_msgs::OccupancyGrid &t_m)
    {
      for(int i = 0 ; i < t_m.info.height ; i++)
      {
        for(int j = 0 ; j < t_m.info.width; j++)
        {
          if ((s_m.data[i*s_m.info.width + j] == -1) || (s_m.data[i*s_m.info.width + j] >= 90))
            t_m.data[i*t_m.info.width + j] = 100; 
          else if (s_m.data[i*s_m.info.width + j] >= 50)
            t_m.data[i*t_m.info.width + j] = 50;
          else
            t_m.data[i*t_m.info.width + j] = -1;
        }
      }
    }
    
    void updateTaskMap(nav_msgs::OccupancyGrid s_m, nav_msgs::OccupancyGrid &t_m)
    {
      t_m.header.stamp = ros::Time::now();
      for(int i = 0 ; i < t_m.info.height ; i++)
      {
        for(int j = 0 ; j < t_m.info.width; j++)
        {
          if ((s_m.data[i*s_m.info.width + j] == -1) && (s_m.data[i*s_m.info.width + j] > 90))
              t_m.data[i*t_m.info.width + j] = 100;
          else if (t_m.data[i*s_m.info.width + j] != 0)
          {
            if (s_m.data[i*s_m.info.width + j] < 50)
              t_m.data[i*t_m.info.width + j] = -1;
            else
              t_m.data[i*t_m.info.width + j] = 50;
          }
        }
      }
    }
    
    void updateTask(geometry_msgs::Pose t_p, nav_msgs::OccupancyGrid &t_m)
    {
      t_m.header.stamp = ros::Time::now();
      double distance;
      float task_ratio = 0.5;
      float d;
      int i;
      int j;
      int m_grid_x = round((t_p.position.x - t_m.info.origin.position.x) / t_m.info.resolution);
      int m_grid_y = round((t_p.position.y - t_m.info.origin.position.y) / t_m.info.resolution);
      for(int a = 0; a < 360; a++)
      {
        d=0.5;
        distance = 0;
        while (distance <= task_ratio)
        {
          i = round(m_grid_y + d*sin(a*PI/180));
          j = round(m_grid_x + d*cos(a*PI/180));
          distance = sqrt(pow(m_grid_y-i,2)+pow(m_grid_x-j,2))*t_m.info.resolution;
          //ROS_INFO("i: %d, j: %d, distance: %f", i, j, distance);
          if ((i > 0) && (j > 0) && (i < t_m.info.height) && (j < t_m.info.width) && (t_m.data[i*t_m.info.width + j] < 90))
            t_m.data[i*t_m.info.width + j] = 0;
          else
            distance = task_ratio;
          d+=1;
        }
      }
    }
  void wait(bool d)
  {
    if(d)
    {
      printf("\nPRESS ENTER TO CONTINUE");
      char c;
      scanf("%c",&c);
    }
  }

  void initStampedPose(geometry_msgs::PoseStamped &p, const char* s)
  {
    p.header.frame_id = s;
    p.header.stamp = ros::Time::now();
    initPose(p.pose);
  }

  void initPose(geometry_msgs::Pose &p)
  {
    p.position.x = 0;
    p.position.y = 0;
    p.position.z = 0;
    p.orientation.x = 0;
    p.orientation.y = 0;
    p.orientation.z = 0;
    p.orientation.w = 1;
  }

  void printStampedPose(geometry_msgs::PoseStamped p, const char* s)
  {
    ROS_INFO("PRINT STAMPED POSE: %s \n\tFRAME_ID: %s \n\tSTAMP: \n\t\tsec: %d \n\t\tnsec: %d",
        s,
        p.header.frame_id.c_str(),
        p.header.stamp.sec,
        p.header.stamp.nsec);
     printPose(p.pose, s);
  }

  void printPose(geometry_msgs::Pose p, const char* s)
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

  void deleteFile(const char* s)
  {
    if (remove(s) == 0)
      ROS_INFO("%s DELETED",s);
    else
      ROS_INFO("%s NOT DELETED",s);
  }
  private:
    ros::NodeHandle n;
    
    // SCAN POSE //
    tf2_ros::TransformListener tf2_;
    tf2_ros::Buffer buffer_;
    geometry_msgs::PoseStamped base_scan_pose;
    geometry_msgs::PoseStamped scan_pose;
    ros::Publisher pub_scan_pose;
    
    // MAP //
    ros::Subscriber sub_map;
    nav_msgs::OccupancyGrid map;
    //int grid_x;
    //int grid_y;
    bool new_map = true;
    
    // SAFE MAP //
    nav_msgs::OccupancyGrid safe_map;
    ros::Publisher pub_safe_map;

    // TASK MAP //
    nav_msgs::OccupancyGrid task_map;
    ros::Publisher pub_task_map;
    geometry_msgs::PoseStamped task_pose;
    //int m_grid_x;
    //int m_grid_y;
    
    // CONTROL //
    bool debug = false;
};

// MAIN AND GENERAL FUNCTIONS //
int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_processing");
  MapProcessing mapProcessing;
  ros::spin();
  return 0;
}



/*void ReadLocalMap()
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
