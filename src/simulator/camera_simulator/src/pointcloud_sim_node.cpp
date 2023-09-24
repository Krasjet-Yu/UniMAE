#include <fstream>
#include <iostream>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

#include <pcl_conversions/pcl_conversions.h>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <Eigen/Eigen>

#include <visualization_msgs/MarkerArray.h>

#include "backward.hpp"
namespace backward {
backward::SignalHandling sh;
}

//#include <cloud_banchmark/cloud_banchmarkConfig.h>
using namespace cv;
using namespace std;
using namespace Eigen;

std::string robot_type;
int robot_id;

cv::Mat depth_mat;

// camera param
int width, height;
double fx, fy, cx, cy;
double sensing_horizon, sensing_rate, estimation_rate;

ros::Publisher _depth_publisher;
ros::Publisher _pose_publisher;
ros::Publisher _fov_publisher;

ros::Subscriber odom_sub;
ros::Subscriber global_map_sub;
ros::Timer local_sensing_timer, estimation_timer;

bool has_global_map(false);
bool has_odom(false);

Matrix4d cam02body;
Matrix4d w_cam_pose;
Eigen::Vector3d w_cam_trans;
Eigen::Quaterniond w_cam_quat;
nav_msgs::Odometry odom_;

ros::Time last_odom_stamp = ros::TIME_MAX;
Eigen::Vector3d last_pose_world;
pcl::PointCloud<pcl::PointXYZ> cloudIn;

// fov visualize
double max_dis_ = 4.0;
double x_max_dis_gain_ = 0.64;
double y_max_dis_gain_ = 0.82;
visualization_msgs::Marker markerNode_fov;
visualization_msgs::Marker markerEdge_fov;
visualization_msgs::Marker marker_line, fast_marker_line;
std::vector<Eigen::Vector3d> fov_node;

void fov_visual_init(std::string msg_frame_id) {
  double x_max_dis = max_dis_ * x_max_dis_gain_;
  double y_max_dis = max_dis_ * y_max_dis_gain_;

  fov_node.resize(5);
  fov_node[0][0] = 0;
  fov_node[0][1] = 0;
  fov_node[0][2] = 0;

  fov_node[1][2] = x_max_dis;
  fov_node[1][1] = y_max_dis;
  fov_node[1][0] = max_dis_;

  fov_node[2][2] = x_max_dis;
  fov_node[2][1] = -y_max_dis;
  fov_node[2][0] = max_dis_;

  fov_node[3][2] = -x_max_dis;
  fov_node[3][1] = -y_max_dis;
  fov_node[3][0] = max_dis_;

  fov_node[4][2] = -x_max_dis;
  fov_node[4][1] = y_max_dis;
  fov_node[4][0] = max_dis_;

  markerNode_fov.header.frame_id = msg_frame_id;
  // markerNode_fov.header.stamp = msg_time;
  markerNode_fov.action = visualization_msgs::Marker::ADD;
  markerNode_fov.type = visualization_msgs::Marker::SPHERE_LIST;
  markerNode_fov.ns = "fov_nodes";
  // markerNode_fov.id = 0;
  markerNode_fov.pose.orientation.w = 1;
  markerNode_fov.scale.x = 0.05;
  markerNode_fov.scale.y = 0.05;
  markerNode_fov.scale.z = 0.05;
  markerNode_fov.color.r = 0;
  markerNode_fov.color.g = 0.8;
  markerNode_fov.color.b = 1;
  markerNode_fov.color.a = 1;

  markerEdge_fov.header.frame_id = msg_frame_id;
  // markerEdge_fov.header.stamp = msg_time;
  markerEdge_fov.action = visualization_msgs::Marker::ADD;
  markerEdge_fov.type = visualization_msgs::Marker::LINE_LIST;
  markerEdge_fov.ns = "fov_edges";
  // markerEdge_fov.id = 0;
  markerEdge_fov.pose.orientation.w = 1;
  markerEdge_fov.scale.x = 0.05;
  markerEdge_fov.color.r = 0.5f;
  markerEdge_fov.color.g = 0.0;
  markerEdge_fov.color.b = 0.0;
  markerEdge_fov.color.a = 1;  // Don't forget to set the alpha!
}

void fov_visual() {
  visualization_msgs::Marker clear_previous_msg;
  clear_previous_msg.action = visualization_msgs::Marker::DELETEALL;

  visualization_msgs::MarkerArray markerArray_fov;
  markerNode_fov.points.clear();
  markerEdge_fov.points.clear();

  std::vector<geometry_msgs::Point> fov_node_marker;
  for (int i = 0; i < (int)fov_node.size(); i++) {
    Eigen::Vector3d vector_temp;
    vector_temp = w_cam_quat * fov_node[i] + w_cam_trans;
    geometry_msgs::Point point_temp;
    point_temp.x = vector_temp[0];
    point_temp.y = vector_temp[1];
    point_temp.z = vector_temp[2];
    fov_node_marker.push_back(point_temp);
  }

  markerNode_fov.points.push_back(fov_node_marker[0]);
  markerNode_fov.points.push_back(fov_node_marker[1]);
  markerNode_fov.points.push_back(fov_node_marker[2]);
  markerNode_fov.points.push_back(fov_node_marker[3]);
  markerNode_fov.points.push_back(fov_node_marker[4]);

  markerEdge_fov.points.push_back(fov_node_marker[0]);
  markerEdge_fov.points.push_back(fov_node_marker[1]);

  markerEdge_fov.points.push_back(fov_node_marker[0]);
  markerEdge_fov.points.push_back(fov_node_marker[2]);

  markerEdge_fov.points.push_back(fov_node_marker[0]);
  markerEdge_fov.points.push_back(fov_node_marker[3]);

  markerEdge_fov.points.push_back(fov_node_marker[0]);
  markerEdge_fov.points.push_back(fov_node_marker[4]);

  markerEdge_fov.points.push_back(fov_node_marker[1]);
  markerEdge_fov.points.push_back(fov_node_marker[2]);

  markerEdge_fov.points.push_back(fov_node_marker[2]);
  markerEdge_fov.points.push_back(fov_node_marker[3]);

  markerEdge_fov.points.push_back(fov_node_marker[3]);
  markerEdge_fov.points.push_back(fov_node_marker[4]);

  markerEdge_fov.points.push_back(fov_node_marker[4]);
  markerEdge_fov.points.push_back(fov_node_marker[1]);

  markerArray_fov.markers.push_back(clear_previous_msg);
  markerArray_fov.markers.push_back(markerNode_fov);
  markerArray_fov.markers.push_back(markerEdge_fov);
  _fov_publisher.publish(markerArray_fov);
}

void odometryCallbck(const nav_msgs::Odometry& odom) {
  /*if(!has_global_map)
    return;*/
  has_odom = true;
  odom_ = odom;
  Matrix4d Pose_receive = Matrix4d::Identity();

  Eigen::Vector3d request_position;
  Eigen::Quaterniond request_pose;
  request_position.x() = odom.pose.pose.position.x;
  request_position.y() = odom.pose.pose.position.y;
  request_position.z() = odom.pose.pose.position.z;
  request_pose.x() = odom.pose.pose.orientation.x;
  request_pose.y() = odom.pose.pose.orientation.y;
  request_pose.z() = odom.pose.pose.orientation.z;
  request_pose.w() = odom.pose.pose.orientation.w;
  Pose_receive.block<3, 3>(0, 0) = request_pose.toRotationMatrix();
  Pose_receive(0, 3) = request_position(0);
  Pose_receive(1, 3) = request_position(1);
  Pose_receive(2, 3) = request_position(2);

  Eigen::Matrix4d world2body = Pose_receive;
  // convert to cam pose
  w_cam_pose = cam02body.transpose() * world2body;
  w_cam_trans = Eigen::Vector3d(w_cam_pose(0, 3), w_cam_pose(1, 3), w_cam_pose(2, 3));
  w_cam_quat = w_cam_pose.block<3, 3>(0, 0);
  last_odom_stamp = odom.header.stamp;

  last_pose_world(0) = odom.pose.pose.position.x;
  last_pose_world(1) = odom.pose.pose.position.y;
  last_pose_world(2) = odom.pose.pose.position.z;

  fov_visual();
}

void pubCameraPose(const ros::TimerEvent& event) {
  // cout<<"pub cam pose"
  geometry_msgs::PoseStamped camera_pose;
  camera_pose.header = odom_.header;
  camera_pose.header.frame_id = "world";
  camera_pose.pose.position.x = w_cam_pose(0, 3);
  camera_pose.pose.position.y = w_cam_pose(1, 3);
  camera_pose.pose.position.z = w_cam_pose(2, 3);
  camera_pose.pose.orientation.w = w_cam_quat.w();
  camera_pose.pose.orientation.x = w_cam_quat.x();
  camera_pose.pose.orientation.y = w_cam_quat.y();
  camera_pose.pose.orientation.z = w_cam_quat.z();
  _pose_publisher.publish(camera_pose);
}

void pointCloudCallBack(const sensor_msgs::PointCloud2& pointcloud_map) {
  if (has_global_map) return;

  ROS_WARN("Global Pointcloud received..");
  // load global map
  // transform map to point cloud format
  pcl::fromROSMsg(pointcloud_map, cloudIn);
  printf("global map has points: %d.\n", cloudIn.points.size());
  has_global_map = true;
}

void renderDepth() {
  double this_time = ros::Time::now().toSec();
  Matrix4d cam_pose = w_cam_pose.inverse();

  // depth_mat = cv::Mat::zeros();
  depth_mat = cv::Mat::zeros(height, width, CV_32FC1);

  Eigen::Matrix4d Tcw = w_cam_pose.inverse();
  Eigen::Matrix3d Rcw = Tcw.block<3, 3>(0, 0);
  Eigen::Vector3d tcw = Tcw.block<3, 1>(0, 3);

  auto t1 = ros::Time::now();

  Eigen::Vector3d pos = w_cam_pose.block<3, 1>(0, 3);
  for (auto pt : cloudIn.points) {
    Eigen::Vector3d pw(pt.x, pt.y, pt.z);
    if ((pos - pw).norm() > 5.0) continue;

    Eigen::Vector3d pc = Rcw * pw + tcw;

    if (pc[2] <= 0.0) continue;

    // std::cout << "pc: " << pc.transpose() << std::endl;

    float projected_x, projected_y;
    projected_x = pc[0] / pc[2] * fx + cx;
    projected_y = pc[1] / pc[2] * fy + cy;
    if (projected_x < 0 || projected_x >= width || projected_y < 0 || projected_y >= height)
      continue;

    // std::cout << "(u,v): " << projected_x << ", " << projected_y << endl;
    float dist = pc[2];
    int r = 0.0573 * fx / dist + 0.5;
    // std::cout << "r: " << r << std::endl;
    int min_x = max(int(projected_x - r), 0);
    int max_x = min(int(projected_x + r), width - 1);
    int min_y = max(int(projected_y - r), 0);
    int max_y = min(int(projected_y + r), height - 1);

    for (int to_x = min_x; to_x <= max_x; to_x++)
      for (int to_y = min_y; to_y <= max_y; to_y++) {
        // std::cout << "(u',v'): " << to_x << ", " << to_y << std::endl;
        float value = depth_mat.at<float>(to_y, to_x);
        if (value < 1e-3) {
          depth_mat.at<float>(to_y, to_x) = dist;
        } else {
          depth_mat.at<float>(to_y, to_x) = min(value, dist);
        }
      }
  }

  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = last_odom_stamp;
  out_msg.header.frame_id = "world";
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  out_msg.image = depth_mat.clone();
  _depth_publisher.publish(out_msg.toImageMsg());
}

void renderSensedPoints(const ros::TimerEvent& event) {
  if (!has_global_map) return;
  renderDepth();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pointcloud_simulator");
  ros::NodeHandle nh("~");

  nh.getParam("robot_type", robot_type);
  nh.getParam("robot_id", robot_id);
  nh.getParam("cam_width", width);
  nh.getParam("cam_height", height);
  nh.getParam("cam_fx", fx);
  nh.getParam("cam_fy", fy);
  nh.getParam("cam_cx", cx);
  nh.getParam("cam_cy", cy);
  nh.getParam("sensing_horizon", sensing_horizon);
  nh.getParam("sensing_rate", sensing_rate);
  nh.getParam("estimation_rate", estimation_rate);

  std::string cam_pose_topic, cam_depth_topic; 
  std::string map_topic, odom_topic, cam_fov_topic;
  nh.getParam("map_topic", map_topic);
  nh.getParam("odom_topic", odom_topic);
  nh.getParam("cam_pose_topic", cam_pose_topic);
  nh.getParam("cam_depth_topic", cam_depth_topic);
  nh.getParam("cam_fov_topic", cam_fov_topic);

  cam02body << 1.0,  0.0,  0.0, 0.0, 
               0.0,  1.0,  0.0, 0.0, 
               0.0,  0.0,  1.0, 0.0, 
               0.0,  0.0,  0.0, 1.0;

  // init w_cam_pose transformation
  w_cam_pose = Matrix4d::Identity();
  // subscribe point cloud
  global_map_sub = nh.subscribe(map_topic, 1, pointCloudCallBack);
  odom_sub = nh.subscribe(odom_topic, 50, odometryCallbck);

  // publisher depth image and color image
  _depth_publisher = nh.advertise<sensor_msgs::Image>(cam_depth_topic, 1000);
  _pose_publisher = nh.advertise<geometry_msgs::PoseStamped>(cam_pose_topic, 1000);
  _fov_publisher = nh.advertise<visualization_msgs::MarkerArray>(cam_fov_topic, 1);

  double sensing_duration = 1.0 / sensing_rate;
  double estimate_duration = 1.0 / estimation_rate;

  local_sensing_timer = nh.createTimer(ros::Duration(sensing_duration), renderSensedPoints);
  estimation_timer = nh.createTimer(ros::Duration(estimate_duration), pubCameraPose);

  fov_visual_init(robot_type+std::to_string(robot_id)+"_vision");

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }
}