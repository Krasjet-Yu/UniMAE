#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/replan_fsm.h>

using namespace car_planner;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "car_planner_node");
  ros::NodeHandle nh("~");

  ReplanFSM rebo_replan;

  rebo_replan.init(nh);

  // ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}