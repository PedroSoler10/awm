#include "ros/ros.h"
#include <stdio.h>
#include <fstream>
#include "nav_msgs/OccupancyGrid.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "message_filters/subscriber.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "boost/date_time/posix_time/posix_time.hpp"

class BrainTest
{
  public:
    BrainTest() :
    tfListener(tfBuffer),
    target_frame("map"),
    source_frame("base_scan")
    //tfFilter(sub_odom, tfBuffer, target_frame, 10, 0)    
    {
      sub_map = n.subscribe("/map", 10, &BrainTest::mapCallback, this);
      sub_status = n.subscribe("/move_base/status", 5, &BrainTest::statusCallback, this);
      //sub_odom.subscribe(n,"/odom",10);
      //tfFilter.registerCallback(boost::bind(&BrainTest::filterCallback, this, _1));
      //pub_scan_pose = n.advertise<geometry_msgs::Pose>("/scan_pose", 1000);
      
      sub_goal = n.subscribe("/move_base_simple/goal", 10, &BrainTest::goalCallback, this);
      pub_goal = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
      pub_local_map = n.advertise<nav_msgs::OccupancyGrid>("/local_map", 1);
            
      ros::Duration(2).sleep();
      fnGetMapToScan();
      
      ros::Rate loop_rate(10);
      while (ros::ok())
      {
        control();
        ros::spinOnce();
        loop_rate.sleep();
      }
    }
    void goalCallback(const geometry_msgs::PoseStamped msg)
    {
      /*geometry_msgs::Pose newGoal = msg.pose;
      dif.pose.position.x -= goal.pose.position.x;
      dif.pose.position.y -= goal.pose.position.y;
      dif.pose.position.z -= goal.pose.position.z;
      dif.pose.orientation.x -= goal.pose.orientation.x;
      dif.pose.orientation.y -= goal.pose.orientation.y;
      dif.pose.orientation.z -= goal.pose.orientation.z;
      dif.pose.orientation.w -= goal.pose.orientation.w;
      if ((abs(dif.pose.position.x)<0.1)&&(abs(dif.pose.position.y)<0.1)&&(abs(dif.pose.position.z)<0.1)&&
 (abs(dif.pose.position.x)<0.1)&&(abs(dif.pose.position.y)<0.1)&&(abs(dif.pose.position.z)<0.1)&&(abs(dif.pose.position.w)<0.1)&&)
      newGoal = true;*/
    }
    
    void mapCallback(const nav_msgs::OccupancyGrid msg)
    {
      //ROS_INFO("I heard: [%f]", msg.info.origin.position.x);
      map = msg;
    }
    void fnGetGridPose()
    {
      local_map = map;
      int local_map_size = 40;
      int map_size = map.info.width - 1;
      local_map.info.width = local_map_size + 1;
      local_map.info.height = local_map_size + 1;
      local_map.info.origin.position.x = scan_pose.pose.position.x - local_map_size/2 * local_map.info.resolution;
      local_map.info.origin.position.y = local_map.info.origin.position.x;
             
      grid_x = round((scan_pose.pose.position.x - map.info.origin.position.x)/ map.info.resolution);
      grid_y = round((scan_pose.pose.position.y - map.info.origin.position.y)/ map.info.resolution);
      if((grid_x+grid_y) < 2*map_size)
      {
        ROS_INFO("Grid Cells:\nx: %d y: %d",grid_x, grid_y);
        ROS_INFO("LOCAL MAP:");
        std::ofstream myfile;
        myfile.open ("local_map.txt");
        int k = round(grid_x-local_map_size/2);
        int l = round(grid_y-local_map_size/2);
        for(int i = 0 ; i <= local_map_size ; i++)
        {
          for(int j = 0 ; j <= local_map_size; j++)
          {
            local_map.data[i*(local_map_size+1) + j] = map.data[k*(map_size+1) + l];
            //myfile <<"("<<i<<","<<j<<"): "
            myfile<<int(local_map.data[i*(local_map_size+1) + j])<<"\t";
            l++;
          }
          myfile <<"\n";
          l = round(grid_y-local_map_size/2);
          k++;
        }
        myfile.close();

        pub_local_map.publish(local_map);
      }
    }
    void fnWriteMap()
    {
      std::ofstream myfile;
      myfile.open ("local_map.txt");
      myfile << "\n";
      myfile.close();
    }
    //void filterCallback (nav_msgs::Odometry& msg_evt)
    void fnGetMapToScan()
    {
      try
      {
        map_to_scan = tfBuffer.lookupTransform(target_frame, source_frame,  ros::Time(0));
        //tfBroadcaster.sendTransform(map_to_scan);
        scan_pose.header.frame_id = target_frame;
        scan_pose.header.stamp = ros::Time::now();
        scan_pose.pose.position.x = map_to_scan.transform.translation.x;
        scan_pose.pose.position.y = map_to_scan.transform.translation.y;
        scan_pose.pose.position.z = map_to_scan.transform.translation.z;
        scan_pose.pose.orientation.x = map_to_scan.transform.rotation.x;
        scan_pose.pose.orientation.y = map_to_scan.transform.rotation.y;
        scan_pose.pose.orientation.z = map_to_scan.transform.rotation.z;
        scan_pose.pose.orientation.w = map_to_scan.transform.rotation.w;
        
        ROS_INFO("Scan Pose:\nx:%f y:%f z:%f x:%f y:%f z:%f w:%f",
          scan_pose.pose.position.x,
          scan_pose.pose.position.y,
          scan_pose.pose.position.z,
          scan_pose.pose.orientation.x,
          scan_pose.pose.orientation.y,
          scan_pose.pose.orientation.z,
          scan_pose.pose.orientation.w);
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
      }
      //pub_scan_pose.publish(scan_pose);
      fnGetGridPose();
    }
    
    void fnPubGoal()
    {
      goal = scan_pose;
      if (flag)
        goal.pose.position.x += 1;
      else
        goal.pose.position.x -= 1;
      flag = !flag;
      //goal.pose.orientation->setRPY(0,0,0);
      goal.pose.orientation.x = 0;
      goal.pose.orientation.y = 0;
      goal.pose.orientation.z = 0;
      goal.pose.orientation.w = 1;
      pub_goal.publish(goal);
      ROS_INFO("Goal:\nx:%f y:%f z:%f x:%f y:%f z:%f w:%f",
          goal.pose.position.x,
          goal.pose.position.y,
          goal.pose.position.z,
          goal.pose.orientation.x,
          goal.pose.orientation.y,
          goal.pose.orientation.z,
          goal.pose.orientation.w);
    }
    void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& st)
    {
      if(!st->status_list.empty())
      {
        if ((st->status_list[0].status == 3)&&((st->status_list[1].status < 1)||(st->status_list[1].status > 10)))
          goal_reached = true;
        else
          goal_reached = false;

          ROS_INFO("STATUS 0: %d",st->status_list[0].status);
          ROS_INFO("STATUS 1: %d",st->status_list[1].status);
      }

        /*if((st->status_list[1].status != 3)&&(st->status_list[1].status > 0)&&(st->status_list[1].status < 10))
          status_filter = 0;
        else if (st->status_list[0].status == 3)
          status_filter++;
        else
          status_filter = 0;
        if(status_filter>3)
          goal_reached = true;
        else
          goal_reached = false;
      }
      ROS_INFO("STATUS 0: %d",st->status_list[0].status);
      ROS_INFO("STATUS 1: %d",st->status_list[1].status);
      ROS_INFO("GOAL REACHED: %d",status_filter);
      */
    }
    
    void fnMeasure()
    {
      ros::Duration d(3.0);
      ros::Time n = ros::Time::now();
      
      if ((n - mt) < d)
        measurement = false;
      else
      {
        measurement = true;
        ros::WallDuration two_hours = ros::WallDuration(2*60*60);
        ros::WallTime n = ros::WallTime::now() + two_hours;
        boost::posix_time::ptime my_posix_time = n.toBoost();
        std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);
        ROS_INFO("Measurement:\nx: %f y: %f z: %f t: %s", 
          scan_pose.pose.position.x,
          scan_pose.pose.position.y,
          scan_pose.pose.position.z,
          iso_time_str.c_str());
      }
    }
    
    void control()
    {
      if(sequence == 0) //Pub Goal
      {
        //ros::Duration(2).sleep();
        ROS_INFO("PUBLISHING GOAL");
        fnGetMapToScan();
        fnPubGoal();
        sequence = 1;
        ROS_INFO("GOAL PUBLISHED");
        ros::Duration(0.5).sleep();
      }
      else if (sequence == 1) //Goal reached?
      {
        if(goal_reached == true)
        {
          //ros::Duration(2).sleep();
          ROS_INFO("GOAL REACHED");
          //ros::Duration(2).sleep();
          ROS_INFO("DOING MEASURING");
          fnGetMapToScan();
          mt = ros::Time::now();
          fnMeasure();
          sequence = 2;
        }
      }
      else if (sequence == 2) //Measurement
      {
        fnMeasure();
        if (measurement == true)
        {
          //ros::Duration(2).sleep();
          ROS_INFO("MEASUREMENT DONE");
          sequence = 0;
        }
      }
    }
  private:
    ros::NodeHandle n;
    ros::Subscriber sub_map;
    nav_msgs::OccupancyGrid map;
    ros::Publisher pub_local_map;
    int grid_x;
    int grid_y;
    nav_msgs::OccupancyGrid local_map;
    geometry_msgs::PoseStamped grid_pose;
    ros::Subscriber sub_status;
    std::string target_frame;
    std::string source_frame;
    tf2_ros::TransformListener tfListener;
    tf2_ros::Buffer tfBuffer;
    //message_filters::Subscriber<nav_msgs::Odometry> sub_odom;
    //tf2_ros::MessageFilter<tf2_msgs::TFMessage> tfFilter;
    geometry_msgs::TransformStamped map_to_scan;
    tf2_ros::TransformBroadcaster tfBroadcaster;
    geometry_msgs::PoseStamped scan_pose;
    ros::Publisher pub_scan_pose;
    geometry_msgs::PoseStamped goal;
    ros::Publisher pub_goal;
    ros::Subscriber sub_goal;
    bool flag = true;
    bool goal_reached = false;
    bool measurement = false;
    int sequence = 0;
    int status_filter = 0;
    ros::Time mt;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "brainTest");
  BrainTest brainTest;
  ros::spin();
  return 0;
}





/*
        try
        {
          transformStamped = tfBuffer.lookupTransform("map", "base_scan", ros::Time(0));
          ROS_INFO("x:%f y:%f z:%f\n",
            transformStamped.transform.translation.x,
            transformStamped.transform.translation.y,
            transformStamped.transform.translation.z);
        }
        catch (tf2::TransformException &ex) 
        {
          ROS_WARN("%s",ex.what());
          ros::Duration(1.0).sleep();
          continue;
        }
*/
