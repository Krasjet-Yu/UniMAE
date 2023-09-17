#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <algorithm>
#include <iostream>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "maps.hpp"

void clickCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, mocka::Maps &map) {
  map.addMap(msg);
  return;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "mockamap");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("mock_map", 1);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2 output;

  int seed;

  int sizeX;
  int sizeY;
  int sizeZ;

  double resolution;
  double update_freq;

  int type;

  nh_private.param("seed", seed, 4546);
  nh_private.param("update_frep", update_freq, 1.0);
  nh_private.param("resolution", resolution, 0.38);
  nh_private.param("map_x", sizeX, 100);
  nh_private.param("map_y", sizeY, 100);
  nh_private.param("map_z", sizeZ, 10);
  nh_private.param("type",  type,  1);

  double inv_resolution = 1.0 / resolution;

  sizeX = sizeX * inv_resolution;
  sizeY = sizeY * inv_resolution;
  sizeZ = sizeZ * inv_resolution;

  mocka::Maps::BasicInfo info;
  info.nh_private = &nh_private;
  info.sizeX = sizeX;
  info.sizeY = sizeY;
  info.sizeZ = sizeZ;
  info.seed = seed;
  info.scale = inv_resolution;
  info.output = &output;
  info.cloud = &cloud;

  mocka::Maps map;
  map.setInfo(info);
  map.generate(type);

  ros::Subscriber click_sub = nh.subscribe<geometry_msgs::PoseStamped>("goal", 10, boost::bind(&clickCallback, _1, map));

  ros::Rate loop_rate(update_freq);
  while (ros::ok()) {
    pcl_pub.publish(output);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}