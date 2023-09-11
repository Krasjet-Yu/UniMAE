#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <random>

using namespace std;

struct Point {
    double x;
    double y;

    Point(double _x, double _y) : x(_x), y(_y) {}
};

ros::Publisher all_map_pub_;
sensor_msgs::PointCloud2 map_msg_;
pcl::PointCloud<pcl::PointXYZ> map_cloud_;

ros::Subscriber click_sub_;
std::vector<Eigen::Vector3d> points_;
std::vector<Point> points2_;
double len2_;
double resolution_;

int is_point_in_polygon(Point point, Point polygon[], int n) {
  int i, j;
  int is_inside = 0;
  
  for (i = 0, j = n-1; i < n; j = i++) {
      // 判断点是否在多边形的边界上
      if ((polygon[i].y == point.y && polygon[i].x == point.x) || 
          (polygon[j].y == point.y && polygon[j].x == point.x)) {
          return 1;
      }
      // 判断点是否在多边形的边上
      if ((polygon[i].y == point.y && polygon[j].y == point.y && 
          ((polygon[i].x <= point.x && point.x <= polygon[j].x) || 
          (polygon[j].x <= point.x && point.x <= polygon[i].x))) || 
          (polygon[i].x == point.x && polygon[j].x == point.x && 
          ((polygon[i].y <= point.y && point.y <= polygon[j].y) || 
          (polygon[j].y <= point.y && point.y <= polygon[i].y)))) {
          return 1;
      }
      // 判断点是否在多边形内部
      if (((polygon[i].y > point.y) != (polygon[j].y > point.y)) && 
          (point.x < (polygon[j].x - polygon[i].x) * (point.y - polygon[i].y) / (polygon[j].y - polygon[i].y) + polygon[i].x)) {
          is_inside = !is_inside;
      }
  }
  return is_inside;
}

std::vector<double> convertToRectangle(std::vector<Point>& points) {
    // 找到上边和下边中最短的边
    double topEdge = std::abs(points[0].y - points[3].y); 
    double bottomEdge = std::abs(points[1].y - points[2].y);
    double lengthEdge = std::min(topEdge, bottomEdge);

    // 找到左边和右边中最短的边
    double leftEdge = std::abs(points[2].x - points[0].x);
    double rightEdge = std::abs(points[3].x - points[1].x);
    double widthEdge = std::min(leftEdge, rightEdge);

    // 计算中心点
    double centerX = (points[0].x + points[1].x + points[2].x + points[3].x) / 4;
    double centerY = (points[0].y + points[1].y + points[2].y + points[3].y) / 4;

    // 计算矩形的四个点
    // std::vector<Point> rectangle;
    // rectangle.push_back(Point(centerX - widthEdge/2, centerY - lengthEdge/2));   // 左上角
    // rectangle.push_back(Point(centerX + widthEdge/2, centerY - lengthEdge/2));   // 右上角
    // rectangle.push_back(Point(centerX - widthEdge/2, centerY + lengthEdge/2));   // 左下角
    // rectangle.push_back(Point(centerX + widthEdge/2, centerY + lengthEdge/2));   // 右下角
    std::vector<double> rectangle;
    rectangle.push_back(widthEdge);  // 宽长度
    rectangle.push_back(lengthEdge); // 高长度
    rectangle.push_back(centerX);    // 横坐标
    rectangle.push_back(centerY);    // 纵坐标

    return rectangle;
}

void clickCallback(const geometry_msgs::PoseStamped& msg) {
  double x = msg.pose.position.x;
  double y = msg.pose.position.y;
  points_.push_back(Eigen::Vector3d(x, y, 0));
  if (points_.size() < 2) return;

  // Generate wall using two points
  Eigen::Vector3d p1 = points_[0];
  Eigen::Vector3d p2 = points_[1];
  points_.clear();

  Eigen::Vector3d dir1 = (p2 - p1).normalized();
  double len = (p2 - p1).norm();
  Eigen::Vector3d dir2;
  dir2[0] = -dir1[1];
  dir2[1] = dir1[0];

  pcl::PointXYZ pt_random;
  for (double l1 = 0.0; l1 <= len + 1e-3; l1 += 0.1) {
    Eigen::Vector3d tmp1 = p1 + l1 * dir1;
    for (double l2 = -len2_; l2 <= len2_ + 1e-3; l2 += 0.1) {
      Eigen::Vector3d tmp2 = tmp1 + l2 * dir2;
      for (double h = -0.5; h < 1; h += 0.1) {
        pt_random.x = tmp2[0];
        pt_random.y = tmp2[1];
        pt_random.z = h;
        map_cloud_.push_back(pt_random);
      }
    }
  }

  map_cloud_.width = map_cloud_.points.size();
  map_cloud_.height = 1;
  map_cloud_.is_dense = true;
  pcl::toROSMsg(map_cloud_, map_msg_);
  map_msg_.header.frame_id = "world";
  all_map_pub_.publish(map_msg_);
}

void clickCallback2(const geometry_msgs::PoseStamped& msg) {
  double x = msg.pose.position.x;
  double y = msg.pose.position.y;
  points2_.push_back(Point(x, y));
  if (points2_.size() < 4) return;

  // Generate standard rectangle using four points
  std::vector<double> rectangle = convertToRectangle(points2_);
  points2_.clear();

  double cx = rectangle[2];
  double cy = rectangle[3];
  double width = rectangle[0];
  double length = rectangle[1];
  
  int widnum = ceil(width / resolution_);
  int lennum = ceil(length / resolution_);
  int heinum = 150; // 0.1 * 150 = 15m

  int rl, rh, sl, sh;
  rl = -widnum / 2;
  rh = widnum / 2;
  sl = -lennum / 2;
  sh = lennum / 2;

  pcl::PointXYZ pt_random;

  for (int r = rl; r < rh; ++r) {
    for (int s = sl; s < sh; ++s) {
      for (int h = 0; h < heinum; ++h) {
        if ((r-rl) * (r- rh + 1) * (s - sl) * (s - sh + 1) * h * (h - heinum + 1) == 0) {
          pt_random.x = cx + r * resolution_;
          pt_random.y = cy + s * resolution_;
          pt_random.z = h * resolution_;
          map_cloud_.push_back(pt_random);
        }
      }
    }
  }

  map_cloud_.width = map_cloud_.points.size();
  map_cloud_.height = 1;
  map_cloud_.is_dense = true;
  pcl::toROSMsg(map_cloud_, map_msg_);
  map_msg_.header.frame_id = "world";
  all_map_pub_.publish(map_msg_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "click_map");
  ros::NodeHandle nh("~");

  nh.param("map/len2", len2_, 0.15);
  nh.param("map/resolution", resolution_, 0.1);

  all_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);

  click_sub_ = nh.subscribe("/move_base_simple/goal", 10, clickCallback2);

  ros::Duration(0.5).sleep();

  // init random device

  while (ros::ok()) {
    pcl::toROSMsg(map_cloud_, map_msg_);
    map_msg_.header.frame_id = "world";
    all_map_pub_.publish(map_msg_);

    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }
}
