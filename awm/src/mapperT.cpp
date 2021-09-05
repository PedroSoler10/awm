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
      ROS_INFO("INITIALIZATION DONE");
      fnWait(debug);
      ROS_INFO("STARTING PROGRAM");
      ros::Time timer = ros::Time::now();
      // LOOP
      ros::Rate loop_rate(10);
      
      while ( ros::ok())// && ( ( ros::Time::now().toSec() - timer.toSec() ) < 5 ) )
      {
        // STATE MACHINE
        
        ros::spinOnce();
        loop_rate.sleep();
      }
    }
    
    void fnInit()
    {
      // SCAN POSE
      buffer_.setUsingDedicatedThread(true);
      fnInitStampedPose(base_scan_pose, "base_scan");
      fnPrintPose(base_scan_pose.pose, "BASE_SCAN_POSE");
      fnInitStampedPose(scan_pose, "map");
      fnPrintPose(scan_pose.pose, "SCAN_POSE");
      pub_scan_pose = n.advertise<geometry_msgs::PoseStamped>("/scan_pose", 1);
      
      // MAP //
      fnDeleteFile("map.txt");
      sub_map = n.subscribe("/map", 100, &Mapper::mapCallback, this);
      
      // SAVE MAP //
      fnDeleteFile("safe_map.txt");
      pub_safe_map = n.advertise<nav_msgs::OccupancyGrid>("/safe_map", 1);
      
      // MEASURE //
      fnDeleteFile("measure_map.txt");
      pub_measure_map = n.advertise<nav_msgs::OccupancyGrid>("/measure_map", 1);
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
      ROS_INFO("%s", p.header.frame_id.c_str());
      p.header.stamp = ros::Time::now();
      fnInitPose(p.pose);
      ROS_INFO("%f", p.pose.orientation.w);
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
      ROS_INFO("%f", p.orientation.w);
    }
    
    void fnPrintPose(geometry_msgs::Pose p, const char* s)
    {
      ROS_INFO("PRINT POSE:\n%s POSE:\nx:%f y:%f z:%f x:%f y:%f z:%f w:%f",
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
        fnPrintPose(scan_pose.pose, "SCAN_POSE");
        pub_scan_pose.publish(scan_pose);
        grid_x = round((scan_pose.pose.position.x - map.info.origin.position.x) / map.info.resolution);
        grid_y = round((scan_pose.pose.position.y - map.info.origin.position.y) / map.info.resolution);
        //ROS_INFO("READ SCAN POSE:\nGRID CELLS:\nx: %d y: %d",grid_x, grid_y);
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
      fnMapProcessing(map, safe_map);
      fnWriteMap(safe_map, "safe_map.txt");
      pub_safe_map.publish(safe_map);
      if (new_map)
      {
        // MEASURE_MAP //
        measure_map = safe_map;
        fnInitMeasureMap(safe_map, measure_map);
        fnWriteMap(measure_map, "measure_map.txt");
        pub_measure_map.publish(measure_map);
        new_map = false;
      }
      fnUpdateMeasureMap(safe_map, measure_map);
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
          else if (m.data[i*m.info.width + j] == 0)
            m_f<<" \t";
        }
        m_f <<"\n";
      }
      m_f.close();
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
                if((k > 0) && (k < p_m.info.height) && (l > 0) && (l < p_m.info.width) && ((m.data[i*m.info.width + j] < 50)))
                {
                  p_m.data[k*p_m.info.width + l] = 50;
                  ROS_INFO("%d", p_m.data[k*p_m.info.width + l]);
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
        ROS_INFO("MEASURE DATA:\nx: %f y: %f z: %f t: %s", 
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
          if (s_m.data[i*s_m.info.width + j] == 0)
            m_m.data[i*m_m.info.width + j] = -1;
          else if (s_m.data[i*s_m.info.width + j] == -1)
            m_m.data[i*m_m.info.width + j] == 50;
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
          if (m_m.data[i*s_m.info.width + j] != 0)
          {
            if (s_m.data[i*s_m.info.width + j] == 0)
              m_m.data[i*m_m.info.width + j] = -1;
            else if (s_m.data[i*s_m.info.width + j] == -1)
              m_m.data[i*m_m.info.width + j] == 50;
            else
              m_m.data[i*m_m.info.width + j] = s_m.data[i*s_m.info.width + j];
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
      m_grid_x = round((m_p.position.x - m_m.info.origin.position.x) / m_m.info.resolution);
      m_grid_y = round((m_p.position.y - m_m.info.origin.position.y) / m_m.info.resolution);
      for(int a = 0; a < 360; a++)
      {
        d=0.5;
        distance = 0;
        while (distance <= measure_ratio)
        {
          i = round(m_grid_y + d*sin(a*PI/180));
          j = round(m_grid_x + d*cos(a*PI/180));
          if ((i > 0) && (j > 0) && (i < m_m.info.height) && (j < m_m.info.width) && (m_m.data[i*m_m.info.width + j] != 100))
          {
            m_m.data[i*m_m.info.width + j] = 0;
            distance = sqrt(pow(m_grid_y-i,2)+pow(m_grid_x-j,2))*m_m.info.resolution;
            d+=1;
          }
          else
            distance = measure_ratio;
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
    int grid_x;
    int grid_y;
    bool new_map = true;
    
    // SAFE MAP //
    nav_msgs::OccupancyGrid safe_map;
    ros::Publisher pub_safe_map;

    // MEASURE MAP //
    nav_msgs::OccupancyGrid measure_map;
    ros::Publisher pub_measure_map;
    geometry_msgs::PoseStamped measure_pose;
    int m_grid_x;
    int m_grid_y;
    
    // CONTROL //
    bool debug = true;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapper");
  Mapper mapper;
  ros::spin();
  return 0;
}































/*void fnReadLocalMap()
    {
      measure_map.info.origin.position.x = scan_pose.pose.position.x - (measure_map.info.width-1) / 2*measure_map.info.resolution;
      measure_map.info.origin.position.y = scan_pose.pose.position.y - (measure_map.info.height-1)/ 2*measure_map.info.resolution;
      
      if((grid_x<map.info.width)&&(grid_y<map.info.height))
      {
        ROS_INFO("READ LOCAL MAP:");
        measure_map_file.open ("measure_map.txt");
        int k = grid_y-measure_map.info.height/2;
        int l = grid_x-measure_map.info.width/2;
        for(int i = 0 ; i < measure_map.info.height ; i++)
        {
          for(int j = 0 ; j < measure_map.info.width; j++)
          {
            measure_map.data[i*measure_map.info.width + j] = map.data[k*map.info.width + l];
            measure_map_file<<int(measure_map.data[i*measure_map.info.width + j])<<"\t\t";
            l++;
          }
          measure_map_file <<"\n";
          l = grid_x-measure_map.info.width/2;
          k++;
        }
        measure_map_file.close();
        pub_measure_map.publish(measure_map);
      }
    }*/
