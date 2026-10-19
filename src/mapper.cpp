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

class Mapper
{
  public:
  
    Mapper() :
    tf2_(buffer_)    
    {
      // INITIALIZATION
      fnInit();
      //ROS_INFO("INITIALIZATION DONE");
      fnWait(debug);
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
    
    void fnInit()
    {
      // SCAN POSE
      buffer_.setUsingDedicatedThread(true);
      fnInitStampedPose(base_scan_pose, "base_scan");
      fnInitStampedPose(scan_pose, "map");
      pub_scan_pose = n.advertise<geometry_msgs::PoseStamped>("/scan_pose", 10);
      
      // MAP //
      fnDeleteFile("map.txt");
      sub_map = n.subscribe("/map", 100, &Mapper::mapCallback, this);
      
      // SAVE MAP //
      fnDeleteFile("safe_map.txt");
      pub_safe_map = n.advertise<nav_msgs::OccupancyGrid>("/safe_map", 10);
      
      // MEASURE //
      fnDeleteFile("measure_map.txt");
      pub_measure_map = n.advertise<nav_msgs::OccupancyGrid>("/measure_map", 10);
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
    
    // SCAN POSE //
    void fnReadScanPose()
    {
      try
      {
        buffer_.transform(base_scan_pose, scan_pose, "map");
        scan_pose.header.stamp = ros::Time::now();
        //fnPrintStampedPose(scan_pose, "SCAN_POSE");
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
      fnWriteMap(map, "map.txt");
      fnReadScanPose();
      
      // SAFE_MAP //
      safe_map = map;
      fnDiscretiseMap(map, safe_map);
      fnMapProcessing(map, safe_map);
      fnWriteMap(safe_map, "safe_map.txt");
      pub_safe_map.publish(safe_map);
      
      // MEASURE_MAP //
      if (new_map)
      {
        measure_map = safe_map;
        fnInitMeasureMap(safe_map, measure_map);
        new_map = false;
      }
      fnUpdateMeasureMap(safe_map, measure_map);
      fnWriteMap(measure_map, "measure_map.txt");
      pub_measure_map.publish(measure_map);
      measure_pose = scan_pose;
      fnUpdateMeasurement(measure_pose.pose, measure_map);
    }
    
    void fnWriteMap(nav_msgs::OccupancyGrid m, const char* s)
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
    void fnDiscretiseMap(nav_msgs::OccupancyGrid m, nav_msgs::OccupancyGrid &s_m)
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
    void fnMapProcessing(nav_msgs::OccupancyGrid m, nav_msgs::OccupancyGrid &p_m)
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
    
    /*// MEASURE //
    void fnMeasure()
    {
      ros::Duration d(3.0);
      ros::Time n = ros::Time::now();
      
      if ((n - mt) < d)
        measure_done = false;
      else
      {
        ros::WallDuration two_hours = ros::WallDuration(2*60*60);
        ros::WallTime n = ros::WallTime::now() + two_hours;
        boost::posix_time::ptime my_posix_time = n.toBoost();
        std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);
        fnReadScanPose();
        //ROS_INFO("MEASURE DATA:\nx: %f y: %f z: %f t: %s", 
          scan_pose.pose.position.x,
          scan_pose.pose.position.y,
          scan_pose.pose.position.z,
          iso_time_str.c_str());
        fnUpdateMeasureMap();
        measure_done = true;
      }
    }*/
    
    void fnInitMeasureMap(nav_msgs::OccupancyGrid s_m, nav_msgs::OccupancyGrid &m_m)
    {
      for(int i = 0 ; i < m_m.info.height ; i++)
      {
        for(int j = 0 ; j < m_m.info.width; j++)
        {
          if ((s_m.data[i*s_m.info.width + j] == -1) || (s_m.data[i*s_m.info.width + j] >= 90))
            m_m.data[i*m_m.info.width + j] = 100; 
          else if (s_m.data[i*s_m.info.width + j] >= 50)
            m_m.data[i*m_m.info.width + j] = 50;
          else
            m_m.data[i*m_m.info.width + j] = -1;
        }
      }
    }
    
    void fnUpdateMeasureMap(nav_msgs::OccupancyGrid s_m, nav_msgs::OccupancyGrid &m_m)
    {
      m_m.header.stamp = ros::Time::now();
      for(int i = 0 ; i < m_m.info.height ; i++)
      {
        for(int j = 0 ; j < m_m.info.width; j++)
        {
          if ((s_m.data[i*s_m.info.width + j] == -1) && (s_m.data[i*s_m.info.width + j] > 90))
              m_m.data[i*m_m.info.width + j] = 100;
          else if (m_m.data[i*s_m.info.width + j] != 0)
          {
            if (s_m.data[i*s_m.info.width + j] < 50)
              m_m.data[i*m_m.info.width + j] = -1;
            else
              m_m.data[i*m_m.info.width + j] = 50;
          }
        }
      }
    }
    
    void fnUpdateMeasurement(geometry_msgs::Pose m_p, nav_msgs::OccupancyGrid &m_m)
    {
      m_m.header.stamp = ros::Time::now();
      double distance;
      float measure_ratio = 0.5;
      float d;
      int i;
      int j;
      int m_grid_x = round((m_p.position.x - m_m.info.origin.position.x) / m_m.info.resolution);
      int m_grid_y = round((m_p.position.y - m_m.info.origin.position.y) / m_m.info.resolution);
      //ROS_INFO("GRID CELLS:\nx: %d y: %d",m_grid_x, m_grid_y);
        
      for(int a = 0; a < 360; a++)
      {
        d=0.5;
        distance = 0;
        while (distance <= measure_ratio)
        {
          i = round(m_grid_y + d*sin(a*PI/180));
          j = round(m_grid_x + d*cos(a*PI/180));
          distance = sqrt(pow(m_grid_y-i,2)+pow(m_grid_x-j,2))*m_m.info.resolution;
          //ROS_INFO("i: %d, j: %d, distance: %f", i, j, distance);
          if ((i > 0) && (j > 0) && (i < m_m.info.height) && (j < m_m.info.width) && (m_m.data[i*m_m.info.width + j] < 90))
            m_m.data[i*m_m.info.width + j] = 0;
          else
            distance = measure_ratio;
          d+=1;
        }
      }
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

    // MEASURE MAP //
    nav_msgs::OccupancyGrid measure_map;
    ros::Publisher pub_measure_map;
    geometry_msgs::PoseStamped measure_pose;
    //int m_grid_x;
    //int m_grid_y;
    
    // CONTROL //
    bool debug = false;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapper");
  Mapper mapper;
  ros::spin();
  return 0;
}
