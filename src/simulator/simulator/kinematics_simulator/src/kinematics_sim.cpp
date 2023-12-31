#include <iostream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/Range.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "plan_utils/traj_container.hpp"
#include "pose_utils.h"

#define OMINIDIRECTION 0
#define DIFFERENTIAL   1
#define ACKERMANN 	   2

using namespace arma;
using namespace std;

// ros interface
ros::Subscriber traj_sub;
ros::Publisher  odom_pub;
ros::Publisher  mesh_pub;
ros::Publisher  sensor_pub;
ros::Timer simulate_timer;
visualization_msgs::Marker marker;

// simulator variables
plan_utils::TrajContainer newest_trajectory;
std::default_random_engine generator;
std::normal_distribution<double> distribution{0.0,1.0};

// visualization
sensor_msgs::Range heightROS;
visualization_msgs::Marker sensorROS;

double x = 0.0;
double y = 0.0;
double yaw = 0.0;
double vx = 0.0;
double vy = 0.0;
double w = 0.0;
bool rcv_cmd = false;
bool rcv_traj = false;

// simulator parameters
int car_type = DIFFERENTIAL;
int car_id = 0;
static double car_scale;
double init_x = 0.0;
double init_y = 0.0;
double init_yaw = 0.0;
double time_resolution = 0.01;
double max_longitude_speed = 1.5;
double max_latitude_speed = 1.5;
double max_angular_vel = 2.4;
double max_steer_angle = 0.7;
double time_delay = 0.0;
double wheel_base = 0.5;
double noise_std = 0.1;
Eigen::Quaterniond q_mesh;
Eigen::Vector3d pos_mesh;

// utils
void normYaw(double& th)
{
	while (th > M_PI)
		th -= M_PI * 2;
	while (th < -M_PI)
		th += M_PI * 2;
}

Eigen::Vector2d guassRandom2d(double std)
{
	return std * Eigen::Vector2d(distribution(generator), distribution(generator));
}

Eigen::Vector3d guassRandom3d(double std)
{
	return std * Eigen::Vector3d(distribution(generator), distribution(generator), distribution(generator));
}

double guassRandom(double std)
{
	return std * distribution(generator);
}

void rcvTrajCallBack(const nav_msgs::Odometry traj_msg)
{
    rcv_traj = true;
}

void simCallback(const ros::TimerEvent &e)
{
	nav_msgs::Odometry new_odom;

	double time_now = ros::Time::now().toSec();
	ros::Time ros_time_now = ros::Time::now();
	new_odom.header.stamp    = ros_time_now;
	new_odom.header.frame_id = "world";

	if(rcv_traj)
	{
		plan_utils::Trajectory* cur_segment;
		int segment_Id = newest_trajectory.locateSingulId(time_now);
		double segment_start_time = newest_trajectory.singul_traj[segment_Id].start_time;
		double segment_end_time   = newest_trajectory.singul_traj[segment_Id].end_time;
		double pt_time;
		if(time_now - segment_end_time < 0)
			pt_time = time_now - segment_start_time;
		else
			pt_time = segment_end_time - segment_start_time;
		cur_segment = &newest_trajectory.singul_traj[segment_Id].traj;

		int singul;
		Eigen::Matrix2d B_h;
		B_h << 0, -1,
			   1,  0;
		double cur_yaw, vel, angul_vel;
		Eigen::Vector2d sigma, dsigma, ddsigma;
		sigma   = cur_segment->getPos(pt_time);
		dsigma  = cur_segment->getdSigma(pt_time);
		ddsigma = cur_segment->getddSigma(pt_time);
		singul  = cur_segment->getSingul(pt_time);
		cur_yaw = cur_segment->getAngle(pt_time);
		vel = singul * dsigma.norm();
		angul_vel = (ddsigma.transpose() * B_h * dsigma)(0, 0) / dsigma.squaredNorm();

		x = sigma(0); y = sigma(1);
		yaw = cur_yaw;
		vx = vel; vy = 0;
		w = angul_vel;
	}

	new_odom.pose.pose.position.x  = x;
	new_odom.pose.pose.position.y  = y;
	new_odom.pose.pose.position.z  = 0;
	new_odom.pose.pose.orientation.w  = cos(yaw/2.0);
	new_odom.pose.pose.orientation.x  = 0;
	new_odom.pose.pose.orientation.y  = 0;
	new_odom.pose.pose.orientation.z  = sin(yaw/2.0);
	new_odom.twist.twist.linear.x  = vx;
	new_odom.twist.twist.linear.y  = vy;
	new_odom.twist.twist.linear.z  = 0;
	new_odom.twist.twist.angular.x = 0;
	new_odom.twist.twist.angular.y = 0;
	new_odom.twist.twist.angular.z = w;	

	odom_pub.publish(new_odom);

	Eigen::Quaterniond q_shift(0.0, 0.0, -0.7071, -0.7071);
	Eigen::Quaterniond q_odom;
	q_odom = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
	Eigen::Quaterniond q_marker = q_odom * q_shift;
	marker.pose.orientation.w = q_marker.w();
	marker.pose.orientation.x = q_marker.x();
	marker.pose.orientation.y = q_marker.y();
	marker.pose.orientation.z = q_marker.z();	

	Eigen::Vector3d pos_shift{1.3, 0.0, 0.0};
	Eigen::Vector3d pos(x, y, 0.0);
	pos = q_odom * pos_shift + pos;
	marker.pose.position.x = pos.x();
	marker.pose.position.y = pos.y();
	marker.pose.position.z = pos.z();

	mesh_pub.publish(marker);

	// TF visualize and broadcast
	colvec pose(6);
	colvec q(4);
	pose(0) = x;
  pose(1) = y;
  pose(2) = 0;
	q(0) = cos(yaw/2.0);
  q(1) = 0;
  q(2) = 0;
  q(3) = sin(yaw/2.0);
	bool G = 1;
  bool V = 1;
  bool L = 1;
	// Sensor availability
  sensorROS.header.frame_id = string("world");
  sensorROS.header.stamp = ros_time_now;
  sensorROS.ns = string("sensor");
  sensorROS.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  sensorROS.action = visualization_msgs::Marker::ADD;
  sensorROS.pose.position.x = pose(0);
  sensorROS.pose.position.y = pose(1);
  sensorROS.pose.position.z = pose(2) + 1.0;
  sensorROS.pose.orientation.w = q(0);
  sensorROS.pose.orientation.x = q(1);
  sensorROS.pose.orientation.y = q(2);
  sensorROS.pose.orientation.z = q(3);
  string strG = G ? string(" GPS ") : string("");
  string strV = V ? string(" Vision ") : string("");
  string strL = L ? string(" Laser ") : string("");
  sensorROS.text = "| " + strG + strV + strL + " |";
  sensorROS.color.a = 1.0;
  sensorROS.color.r = 1.0;
  sensorROS.color.g = 1.0;
  sensorROS.color.b = 1.0;
  sensorROS.scale.z = 0.5;
  sensor_pub.publish(sensorROS);

	// TF for raw sensor visualization
  if (1) {
		static tf2_ros::TransformBroadcaster _br;
		geometry_msgs::TransformStamped _transformStamped;

		string base_s   = car_id == -1 ? string("base")   : string("car") + std::to_string(car_id) + string("_base");
    string laser_s  = car_id == -1 ? string("laser")  : string("car") + std::to_string(car_id) + string("_laser");
    string vision_s = car_id == -1 ? string("vision") : string("car") + std::to_string(car_id) + string("_vision");

		_transformStamped.header.stamp = ros_time_now;

		_transformStamped.header.frame_id = "world";
		_transformStamped.child_frame_id = base_s;
		_transformStamped.transform.translation.x = pose(0);
		_transformStamped.transform.translation.y = pose(1);
		_transformStamped.transform.translation.z = pose(2);
		_transformStamped.transform.rotation.x = q(1);
		_transformStamped.transform.rotation.y = q(2);
		_transformStamped.transform.rotation.z = q(3);
		_transformStamped.transform.rotation.w = q(0);
		_br.sendTransform(_transformStamped);

    colvec y45 = zeros<colvec>(3);
    y45(0) = 45.0 * M_PI / 180;
    colvec q45 = R_to_quaternion(ypr_to_R(y45));
		// 0.92388 0 0 0.382683
		// std::cout << q45(0) << " " << q45(1) << " " << q45(2) << " " << q45(3) << std::endl;

		_transformStamped.header.frame_id = base_s;
		_transformStamped.child_frame_id = laser_s;
    _transformStamped.transform.rotation.x = q45(1);
		_transformStamped.transform.rotation.y = q45(2);
		_transformStamped.transform.rotation.z = q45(3);
		_transformStamped.transform.rotation.w = q45(0);
		_br.sendTransform(_transformStamped);

		_transformStamped.header.frame_id = base_s;
		_transformStamped.child_frame_id = vision_s;
		_transformStamped.transform.rotation.x = q45(1);
		_transformStamped.transform.rotation.y = q45(2);
		_transformStamped.transform.rotation.z = q45(3);
		_transformStamped.transform.rotation.w = q45(0);
		_br.sendTransform(_transformStamped);         
  }
}

// main loop
int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "simulator_node");
    ros::NodeHandle nh("~");

	nh.getParam("car_id", car_id);
	nh.getParam("car_type", car_type);
	nh.getParam("car_scale", car_scale);
	nh.getParam("init_x", init_x);
	nh.getParam("init_y", init_y);
	nh.getParam("init_yaw", init_yaw);
	nh.getParam("time_resolution", time_resolution);
	nh.getParam("max_longitude_speed", max_longitude_speed);
	nh.getParam("max_latitude_speed", max_latitude_speed);
	nh.getParam("max_angular_vel", max_angular_vel);
	nh.getParam("max_steer_angle", max_steer_angle);
	nh.getParam("time_delay", time_delay);
	nh.getParam("wheel_base", wheel_base);
	nh.getParam("noise_std", noise_std);
	
	traj_sub = nh.subscribe("traj", 100, rcvTrajCallBack);
  odom_pub  = nh.advertise<nav_msgs::Odometry>("odom", 10);
	mesh_pub = nh.advertise<visualization_msgs::Marker>("mesh", 100);
	sensor_pub = nh.advertise<visualization_msgs::Marker>("sensor", 100, true);
	
	x = init_x;
	y = init_y;
	yaw = init_yaw;


	marker.header.frame_id = "world";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.mesh_resource = "package://kinematics_simulator/meshes/bmw_x5.dae";
	
	marker.mesh_use_embedded_materials = true;
	marker.pose.position.x = init_x;
	marker.pose.position.y = init_y;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.w = 0.5;
	marker.pose.orientation.x = 0.5;
	marker.pose.orientation.y = 0.5;
	marker.pose.orientation.z = 0.5;
	marker.color.a = 1.0;
	marker.color.r = 0.5;
	marker.color.g = 0.5;
	marker.color.b = 0.5;	

	marker.scale.x = car_scale;
	marker.scale.y = car_scale;
	marker.scale.z = car_scale;
	q_mesh = Eigen::Quaterniond(1.0/sqrt(2), 0.0, 0.0, 1.0/sqrt(2));
	pos_mesh = Eigen::Vector3d(-0.75, 0.35, 0.0);
	
  simulate_timer = nh.createTimer(ros::Duration(time_resolution), simCallback);

	ros::spin();

  return 0;
}