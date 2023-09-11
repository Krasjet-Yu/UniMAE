#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

bool finish = false;
std::string file_name_;

void cloudCallback(const sensor_msgs::PointCloud2& msg) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(msg, cloud);
  pcl::io::savePCDFileASCII(file_name_, cloud);

  cout << "map saved." << endl;
  finish = true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_recorder");
  ros::NodeHandle nh("~");
  // nh.param("file_name", file_name_);
  file_name_ = argv[1];

  // Generate map by clicking
  ros::Subscriber cloud_sub = nh.subscribe("/map_generator/global_cloud", 10, cloudCallback);
  ros::Duration(1.0).sleep();

  while (ros::ok()) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
    if (finish) break;
  }

  // cout << "finish record map." << endl;
  ROS_WARN("[Map Recorder]: finish record map.");
  return 0;
}
