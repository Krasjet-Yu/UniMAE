#include <ros/ros.h>
#include <mapping.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "mapping_vis");
  ros::NodeHandle nh("~");

  MappingProcess mapping;
  mapping.init(nh);

  // ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}