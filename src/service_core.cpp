#include <string.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "move_base_msgs/MoveBaseActionResult.h"

class ServiceCore
{
public:
  ServiceCore()
  {
    pub_goal = nh_.advertise<std_msgs::String>("/move_base_simple/goal", 1);
    pub_measure = nh_.advertise<std_msgs::String>("/measure/goal", 1);
    
    sub_goal = nh_.subscribe("/move_base/result", 1, &ServiceCore::checkGoal, this);
    sub_measure = nh_.subscribe("/measure/result", 1, &ServiceCore::checkMeasure, this);
    
    ros::Rate loop_rate(5);
    while (ros::ok())
    {
      fnControl();
      
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  void checkGoal(const move_base_msgs::MoveBaseActionResult result)
  {
    if (result.status.status == 3)
    {
      is_robot_reached_target = true;
    }
    else
    {
      ROS_INFO("checkGoal : %d", result.status.status);
    }
  }
  void checkMeasure(const move_base_msgs::MeasureResult result)
  {
    if (result.status.status == 3)
    {
      is_measure_done = true;
    }
    else
    {
      ROS_INFO("checkMeasure : %d", result.status.status);
    }
  }
  
  void fnControl()
  {
    if (is_robot_reached_target)
    {
      if (is_measure_done == false)
      {
        fnMesure();
      }
      else if (is_measure_done == true)
      {
        fnNewGoal();
        is_robot_reached_target = false;
        is_measure_done = false;
      }
    }
  }
      
private:
  ros::NodeHandle nh_;
  ros::Publisher pub_goal;
  ros::Publisher pub_measure;
  ros::Subscriber sub_goal;
  ros::Subscriber sub_measure;
  bool is_robot_reached_target = true;
  bool is_measure_done = false;
};

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "service_core");

  //Create an object of class ServiceCore that will take care of everything
  ServiceCore serviceCore;

  ros::spin();

  return 0;
}
