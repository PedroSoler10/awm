#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PoseStamped.h"


class MyTest
{
  public:
    MyTest():
      tf2_(buffer_)
    {
      fnInit();
      
      ros::Rate loop_rate(10);
      while(ros::ok)
      {
        fnReadScanPose();
        ros::spinOnce();
        loop_rate.sleep();
      }
    }
    void fnInit()
    {
      base_scan_pose.header.frame_id = "base_scan";
      base_scan_pose.header.stamp = ros::Time::now();
      base_scan_pose.pose.orientation.x = 0;
      base_scan_pose.pose.orientation.y = 0;
      base_scan_pose.pose.orientation.z = 0;
      base_scan_pose.pose.orientation.w = 1;
      pub_scan_pose = n.advertise<geometry_msgs::PoseStamped>("/scan_pose", 1);
    }
    void fnReadScanPose()
    {
      try
      {
        buffer_.transform(base_scan_pose, scan_pose, "map");
        scan_pose.header.stamp = ros::Time::now();
        ROS_INFO("Scan Pose:\nx:%f y:%f z:%f x:%f y:%f z:%f w:%f",
          scan_pose.pose.position.x,
          scan_pose.pose.position.y,
          scan_pose.pose.position.z,
          scan_pose.pose.orientation.x,
          scan_pose.pose.orientation.y,
          scan_pose.pose.orientation.z,
          scan_pose.pose.orientation.w);
        pub_scan_pose.publish(scan_pose);
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
      }
    }
    
  private:
    ros::NodeHandle n;
    
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf2_;
    geometry_msgs::PoseStamped base_scan_pose;
    geometry_msgs::PoseStamped scan_pose;
    ros::Publisher pub_scan_pose;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "myTest");
  MyTest myTest;
  ros::spin();
  return 0;
}
