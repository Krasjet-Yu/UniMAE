#include <string.h>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/StdVector>
#include <iostream>

#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/PoseStamped.h"
#include "pose_utils.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <pcl/io/ply_io.h>  
#include <pcl_conversions/pcl_conversions.h> 

using namespace std;

string frame_id;
static string mesh_resource;
static string mesh_ply;
bool cross_config = false;
static double color_r, color_g, color_b, color_a, cov_scale, scale;
bool is_render = false;

visualization_msgs::Marker meshROS;
visualization_msgs::MarkerArray meshesROS;

sensor_msgs::PointCloud2 dst_cloud;
sensor_msgs::PointCloud2 meshesPC;

ros::Publisher meshesPub, meshesPcPub;
ros::Timer state_timer;

void render_callback(const geometry_msgs::PoseStamped& msg) {
  if (msg.header.frame_id == string("null"))
    return;
  is_render = true;

  static int obj_num = 0;

  colvec q(4);
  q(0) = 1.0;
  q(1) = 0.0;
  q(2) = 0.0;
  q(3) = 0.0;

  // Mesh model
  meshROS.header.frame_id = frame_id;
  meshROS.header.stamp = msg.header.stamp;
  meshROS.ns = "mesh";
  meshROS.id = obj_num;
  meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
  meshROS.action = visualization_msgs::Marker::ADD;
  meshROS.mesh_use_embedded_materials = true;
  meshROS.pose.position.x = msg.pose.position.x;
  meshROS.pose.position.y = msg.pose.position.y;
  meshROS.pose.position.z = 0;

  meshROS.pose.orientation.w = q(0);
  meshROS.pose.orientation.x = q(1);
  meshROS.pose.orientation.y = q(2);
  meshROS.pose.orientation.z = q(3);
  meshROS.scale.x = 1.0;
  meshROS.scale.y = 1.0;
  meshROS.scale.z = 1.0;
  meshROS.color.a = color_a;
  meshROS.color.r = color_r;
  meshROS.color.g = color_g;
  meshROS.color.b = color_b;
  meshROS.mesh_resource = mesh_resource;

  meshesROS.markers.push_back(meshROS);

  obj_num++;
  is_render = false;
}

void timer_callback(const ros::TimerEvent& event) {
  if (is_render) {
    return;
  } else {
    meshesPub.publish(meshesROS);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "object_visualization");
  ros::NodeHandle n("~");

  // n.param("mesh_resource", mesh_resource, std::string("package://odom_visualization/meshes/hummingbird.mesh"));
  n.param("mesh_resource", mesh_resource, std::string("package://odom_visualization/meshes/house_1.dae"));
  n.param("mesh_ply", mesh_ply, std::string("package://odom_visualization/meshes/house_1.ply"));

  n.param("color/r", color_r, 0.0);
  n.param("color/g", color_g, 0.0);
  n.param("color/b", color_b, 0.0);
  n.param("color/a", color_a, 0.0);
  n.param("scale", scale, 2.0);
  n.param("frame_id", frame_id, string("world"));

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(mesh_ply, *cloud) == -1)
  {
      ROS_ERROR("Failed to load PLY file");
      return -1;
  }
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud, output);
  output.header.frame_id = "world";

  ros::Subscriber sub_odom = n.subscribe("/goal", 100, render_callback);
  
  meshesPub = n.advertise<visualization_msgs::MarkerArray>("/urban", 1000);  
  ros::Publisher pub_cloud = n.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1000);

  pub_cloud.publish(output);
  state_timer = n.createTimer(ros::Duration(0.1), timer_callback);
  ros::spin();

  return 0;
}
