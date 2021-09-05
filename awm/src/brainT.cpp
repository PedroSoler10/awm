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
using namespace std;

class Brain
{
  public:
    Brain() :
    tf2_(buffer_)    
    {
      // INITIALIZATION
      fnInit();
      ROS_INFO("INITIALIZATION DONE");
      fnWait(debug);
      //ros::Duration(5).sleep();
      ROS_INFO("STARTING PROGRAM");
      ros::Time timer = ros::Time::now();
      // LOOP
      ros::Rate loop_rate(10);
      
      while ( ros::ok())// && ( ( ros::Time::now().toSec() - timer.toSec() ) < 5 ) )
      {
        // STATE MACHINE
        control();
        ros::spinOnce();
        loop_rate.sleep();
      }
      //fnReadScanPose();
      //pub_goal.publish(scan_pose);
    }
    void fnInit()
    {
      // SCAN POSE
      sub_scan_pose = n.subscribe("/scan_pose", 100, &Brain::scanPoseCallback, this);
      
      // MAP //
      sub_map = n.subscribe("/map", 10, &Brain::mapCallback, this);
      sub_safe_map = n.subscribe("/safe_map", 10, &Brain::mapCallback, this);
      sub_measure_map = n.subscribe("/measure_map", 10, &Brain::mapCallback, this);
      local_map.header.frame_id = "map";
      local_map.info.resolution = local_map_resolution;
      local_map.info.width = local_map_size+1;
      local_map.info.height = local_map_size+1;
      local_map.info.origin.orientation.w = 1;
      pub_local_map = n.advertise<nav_msgs::OccupancyGrid>("/local_map", 100);
      
      // NAVIGATION //
      goal_pose.header.frame_id = "map";
      pub_goal = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
      sub_status = n.subscribe("/move_base/status", 10, &Brain::statusCallback, this);
      sub_goal = n.subscribe("/move_base_simple/goal", 100, &Brain::goalCallback, this);
      
      // MEASURE //
      pub_measure_pose = n.advertise<geometry_msgs::PoseStamped>("/measure_pose",1);
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
    
    void scanPoseCallback(const geometry_msgs::PoseStamped p)
    {
      
    }
    
    // MAP //
    void mapCallback(const nav_msgs::OccupancyGrid msg)
    {
      //ROS_INFO("MAP CALLBACK");
      map = msg;
      grid_x = round((scan_pose.pose.position.x - map.info.origin.position.x) / map.info.resolution);
      grid_y = round((scan_pose.pose.position.y - map.info.origin.position.y) / map.info.resolution);
    }
    
    /*void fnReadLocalMap()
    {
      local_map.info.origin.position.x = scan_pose.pose.position.x - (local_map.info.width-1) / 2*local_map.info.resolution;
      local_map.info.origin.position.y = scan_pose.pose.position.y - (local_map.info.height-1)/ 2*local_map.info.resolution;
      
      if((grid_x<map.info.width)&&(grid_y<map.info.height))
      {
        ROS_INFO("READ LOCAL MAP:");
        local_map_file.open ("local_map.txt");
        int k = grid_y-local_map.info.height/2;
        int l = grid_x-local_map.info.width/2;
        for(int i = 0 ; i < local_map.info.height ; i++)
        {
          for(int j = 0 ; j < local_map.info.width; j++)
          {
            local_map.data[i*local_map.info.width + j] = map.data[k*map.info.width + l];
            local_map_file<<int(local_map.data[i*local_map.info.width + j])<<"\t\t";
            l++;
          }
          local_map_file <<"\n";
          l = grid_x-local_map.info.width/2;
          k++;
        }
        local_map_file.close();
        pub_local_map.publish(local_map);
      }
    }*/
    
    // NAVIGATION //
    void fnPubGoal(int mode)
    {
      if (goal_published == false)
      {
        ROS_INFO("IDENTIFYING GOAL");
        if (mode == 0)
        {
          
        }
        else if (mode == 1)
        {
          goal_pose = scan_pose;
          if (flag)
            goal_pose.pose.position.x += 1;
          else
            goal_pose.pose.position.x -= 0.75;
            flag = !flag;
            goal_pose.pose.orientation.x = 0;
            goal_pose.pose.orientation.y = 0;
            goal_pose.pose.orientation.z = 0;
            goal_pose.pose.orientation.w = 1;
            pub_goal.publish(goal_pose);
            goal_published = true;
        }
        else if (mode == 2)
        {
          int r_min = 0;
          int r_max = 0;
          int r_step = 10;
          int r_lim = 200;
          int i;
          int j;
          int d;
          int a;
          int i_min = grid_y;
          int j_min = grid_x;
          int d_min = r_lim + 1;
          int a_min = 0;
          
          while (d_min > r_max)
          {
            r_min = r_max;
            r_max += r_step;
            d = r_min;
            do
            {
              a = 0;
              do
              {
                i = round(grid_y + d*sin(a*PI/180));
                j = round(grid_x + d*cos(a*PI/180));
                if((i > measure_map.info.height) || (j > measure_map.info.width) || (i < 0) || (j < 0))
                {
                  i = grid_y;
                  j = grid_x;
                }
                a++;
                //ROS_INFO("GOAL OPTION:\nX: %d Y: %d DATA: %d", j, i, measure_map.data[i*measure_map.info.width + j]);
              }
              while ((measure_map.data[i*measure_map.info.width + j] != -1) && (a < 360));
              d++;
            }
            while ((measure_map.data[i*measure_map.info.width + j] != -1) && (d < r_max));
            a--;
            d--;
            if (measure_map.data[i*measure_map.info.width + j] == -1)
            {
              i_min = i;
              j_min = j;
              d_min = d;
              a_min = a;
              ROS_INFO("I_MIN: %d J_MIN: %d D_MIN: %d A_MIN: %d ",i_min,j_min,d_min,a_min);
            }
          }
          if((i_min == grid_x) && (j_min == grid_y) && (d_min == r_lim + 1) && (a_min == 0))
          {
            ROS_INFO("EVERYTHING MEASURED");
          }
          else
          {
            goal_pose = scan_pose;
            goal_pose.pose.position.x = j_min * measure_map.info.resolution + measure_map.info.origin.position.x;
            goal_pose.pose.position.y = i_min * measure_map.info.resolution + measure_map.info.origin.position.y;
            tf2::Quaternion goal_angle_tf;
            goal_angle_tf.setRPY(0, 0, a*PI/180);
            geometry_msgs::Quaternion goal_angle;
            goal_angle = tf2::toMsg(goal_angle_tf);
            goal_pose.pose.orientation.x = double(goal_angle.x);
            goal_pose.pose.orientation.y = double(goal_angle.y);
            goal_pose.pose.orientation.z = double(goal_angle.z);
            goal_pose.pose.orientation.w = double(goal_angle.w);
            pub_goal.publish(goal_pose);
            goal_published = true;
          }
        }
      }
      else
        ROS_INFO("GOAL ALREADY PUBLISHED");
    }
    void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& st)
    {
      if(!st->status_list.empty())
      {
        if ((st->status_list[1].status >= 0) && (st->status_list[1].status <= 9) && (st->status_list[0].goal_id.stamp.sec < st->status_list[1].goal_id.stamp.sec) && (st->status_list[0].goal_id.stamp.nsec < st->status_list[1].goal_id.stamp.nsec))
          if ((st->status_list[2].status >= 0) && (st->status_list[2].status <= 9) && (st->status_list[1].goal_id.stamp.sec < st->status_list[2].goal_id.stamp.sec) && (st->status_list[1].goal_id.stamp.nsec < st->status_list[2].goal_id.stamp.nsec))
            goal_status = st->status_list[2].status;
          else
            goal_status = st->status_list[1].status;
        else
          goal_status = st->status_list[0].status;
       if (goal_status < 3)
          goal_published = true;   
       //ROS_INFO("STATUS  : %d",goal_status);
       //ROS_INFO("STATUS 0: %d TIME: %d.%d", st->status_list[0].status, st->status_list[0].goal_id.stamp.sec, st->status_list[0].goal_id.stamp.nsec);
       //ROS_INFO("STATUS 1: %d TIME: %d.%d", st->status_list[1].status, st->status_list[1].goal_id.stamp.sec, st->status_list[1].goal_id.stamp.nsec);
       //ROS_INFO("STATUS 2: %d TIME: %d.%d", st->status_list[2].status, st->status_list[2].goal_id.stamp.sec, st->status_list[2].goal_id.stamp.nsec);
      }
    }
    void goalCallback(const geometry_msgs::PoseStamped msg)
    {
      /*if ((goal_pose.pose.position.x != msg.pose.position.x) || (goal_pose.pose.position.y != msg.pose.position.y))
      {
        goal_pose = msg;
        goal_published = true;
        new_goal = true;
      }
      else
        new_goal = false;*/
    }

    // MEASURE //
    /*void fnMeasure()
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
        string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);
        //fnReadScanPose();
        ROS_INFO("MEASURE DATA:\nx: %f y: %f t: %s", 
          scan_pose.pose.position.x,
          scan_pose.pose.position.y,
          iso_time_str.c_str());
        string f_n = to_string(scan_pose.pose.position.x)+"_"+
                     to_string(scan_pose.pose.position.y)+"_"+
                     iso_time_str+".txt";
        ofstream m_f;
        m_f.open(f_n);
        m_f.close();
        fnUpdateMeasureMap();
        measure_done = true;
      }
    }*/
    
    // CONTROL //
    void control()
    {
      if(sequence == 0)
      {
        ROS_INFO("WAITING");
        //fnPrintPose(scan_pose.pose, "SCAN");
        if(scan_pose.header.stamp.sec == ros::Time::now().sec)
        {
          fnPrintPose(scan_pose.pose, "SCAN");
          mt = ros::Time::now();
          //fnMeasure();
          sequence = 3;
          ROS_INFO("DOING MEASURING");
          fnWait(debug);
        }
      }
      if(sequence == 1) //Reading Goal
      {
        if (goal_published == true)
        {
          ROS_INFO("GOAL PUBLISHED");
          fnPrintPose(goal_pose.pose,"GOAL");
          fnPrintPose(scan_pose.pose,"SCAN");
          sequence = 2;
          ROS_INFO("REACHING GOAL");
          goal_published = false;
          new_goal = false;
          fnWait(debug);
        }
        else
          fnPubGoal(2);
          
      }
      else if (sequence == 2) //Reaching Goal
      {
        if(goal_status == 3)
        {
          ROS_INFO("GOAL REACHED");
          mt = ros::Time::now();
          //fnMeasure();
          sequence = 3;
          ROS_INFO("DOING MEASURING");
          fnWait(debug);
        }
        else if (goal_status == 4)
        {
          ROS_INFO("GOAL IMPOSSIBLE TO REACH");
          mt = ros::Time::now();
          //fnMeasure();
          sequence = 3;
          ROS_INFO("DOING MEASURING");
          fnWait(debug);
        }
      }
      else if (sequence == 3) //Measuring
      {
        if (measure_done)
        {
          ROS_INFO("MEASURE DONE");
          sequence = 1;
          ROS_INFO("SEARCHING NEW GOAL");
          fnWait(debug);
        }
        else
        {
          //fnMeasure();
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
    ros::Subscriber sub_scan_pose;
    
    // MAP //
    ros::Subscriber sub_map;
    nav_msgs::OccupancyGrid map;
    ros::Subscriber sub_safe_map;
    nav_msgs::OccupancyGrid safe_map;
    int grid_x;
    int grid_y;
    nav_msgs::OccupancyGrid local_map;
    float local_map_resolution = 0.05;
    int local_map_size = 60;
    ros::Publisher pub_local_map;

    // NAVIGATION //
    geometry_msgs::PoseStamped goal_pose;
    bool flag = true;
    ros::Publisher pub_goal;
    ros::Subscriber sub_status;
    ros::Subscriber sub_goal;
    bool goal_published = false;

    // MEASURE //
    ros::Time mt;
    ros::Subscriber sub_measure_map;
    nav_msgs::OccupancyGrid measure_map;
    ros::Publisher pub_measure_pose;
    
    // CONTROL //
    int sequence = 0;
    bool new_goal = false;
    int goal_status = false;
    bool measure_done = false;
    bool debug = true;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "brain");
  Brain brain;
  //ros::spin();
  return 0;
}

