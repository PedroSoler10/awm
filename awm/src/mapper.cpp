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
      if (remove("map.txt") == 0)
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
      fnReadScanPose();
      if (new_map)
      {
        safe_map = map;
        // MEASURE_MAP //
        measure_map = map;
        fnMapProcessing(map, measure_map);
        fnWriteMap(measure_map, "measure_map.txt");
        pub_measure_map.publish(measure_map);
        new_map = false;
      }
      fnWriteMap(map, "map.txt");
      
      // SAFE_MAP //
      fnMapProcessing(map, safe_map);
      fnWriteMap(safe_map, "safe_map.txt");
      pub_safe_map.publish(safe_map);
    }
    
    void fnWriteMap(nav_msgs::OccupancyGrid m, const char* s)
    {
      std::ofstream m_f;
      m_f.open(s);
      for(int j = m.info.width-1; j >= 0; j--)
      {
        for(int i = m.info.height-1; i >= 0; i--)
        {
          if (m.data[i*map.info.width + j] == 100)
            m_f<<"*\t";
          else if (m.data[i*map.info.width + j] == -1)
            m_f<<"·\t";
          else if (m.data[i*map.info.width + j] == 0)
            m_f<<"+\t";
        }
        m_f <<"\n";
      }
      m_f.close();
    }
    
    // MAP PROCESSING //
    void fnMapProcessing(nav_msgs::OccupancyGrid m_i, nav_msgs::OccupancyGrid m_o)
    {
      int radius = 6;
      for(int i = 0 ; i < m_o.info.height ; i++)
      {
        for(int j = 0 ; j < m_o.info.width; j++)
        {
          if (m_i.data[i*m_i.info.width + j] > 90)
          {
            for(int k = i-radius; k <= i+radius; k++)
            {
              for(int l = j-radius; l <= j+radius; l++)
              {
                if((k > 0) && (k < m_o.info.height) && (l > 0) && (l < m_o.info.width))
                  m_o.data[k*m_o.info.width + l] = 100;
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
    }
    void fnUpdateMeasureMap()
    {
      measure_map.header.stamp = ros::Time::now();
      double distance;
      float measure_ratio = 0.5;
      ROS_INFO("UPDATE MEASURE MAP");
      if(measure_map.info.width != map.info.width)
      {
        if (remove("measure_map.txt") == 0)
          ROS_INFO("OLD MEASURE MAP DELETED");
        else
          ROS_INFO("OLD MEASURE MAP NOT DELETED");
        measure_map = map;
        for(int i = 0 ; i < measure_map.info.height ; i++)
        {
          for(int j = 0 ; j < measure_map.info.width; j++)
          {
            if ((map.data[i*map.info.width + j] == -1) || (map.data[i*map.info.width + j] > 90))
              measure_map.data[i*measure_map.info.width + j] = 100;
            else
              measure_map.data[i*measure_map.info.width + j] = -1;
          }
        }
      }
      ROS_INFO("READ MEASURE MAP:");
      for(int i = 0 ; i < measure_map.info.height ; i++)
      {
        for(int j = 0 ; j < measure_map.info.width; j++)
        {
          distance = sqrt(pow(grid_y-i,2)+pow(grid_x-j,2))*measure_map.info.resolution;
          if ((map.data[i*map.info.width + j] == -1) || (map.data[i*map.info.width + j] > 90))
            measure_map.data[i*measure_map.info.width + j] = 100;
          else if (distance <= measure_ratio)
            measure_map.data[i*measure_map.info.width + j] = 0;
          else if (measure_map.data[i*measure_map.info.width + j] != 0)
            measure_map.data[i*measure_map.info.width + j] = -1;
        }
      }
      fnMapProcessing(measure_map);
      measure_map_file.open("measure_map.txt");
      if(measure_map_file.is_open())
      {
        ROS_INFO("MAP FILE OPENED");
        for(int j = measure_map.info.width-1; j >= 0; j--)
        {
          for(int i = measure_map.info.height-1; i >= 0; i--)
          {
            if (measure_map.data[i*measure_map.info.width + j] == 100)
              measure_map_file<<"*\t";
            else if (measure_map.data[i*measure_map.info.width + j] == -1)
              measure_map_file<<" \t";
            else if (measure_map.data[i*measure_map.info.width + j] == 0)
              measure_map_file<<"·\t";
          }
          measure_map_file <<"\n";
        }
        measure_map_file.close();
      }
      else
        ROS_INFO("ERROR WITH MAP FILE");
      pub_measure_map.publish(measure_map);
    }*/
    
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
    ros::Time mt;
    nav_msgs::OccupancyGrid measure_map;
    ros::Publisher pub_measure_map;
    
    // CONTROL //
    bool measure_done = false;
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
