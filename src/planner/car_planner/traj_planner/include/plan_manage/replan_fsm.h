#ifndef _CAR_REPLAN_FSM_H_
#define _CAR_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include <mapping.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

using std::vector;
using namespace std;

namespace car_planner{
  const double TIME_BUDGET = 0.1;

  class ReplanFSM
  {
  private:

      enum FSM_EXEC_STATE
      {
          INIT,
          WAIT_TARGET,
          GEN_NEW_TRAJ,
          REPLAN_TRAJ,
          EXEC_TRAJ,
          EMERGENCY_STOP,
          SEQUENTIAL_START
      };

      enum TARGET_TYPE
      {
          MANNUAL_TARGET = 1,
          PRESET_TARGET = 2,
          REFERENCE_PATH = 3
      };
      
      /* planning utils */
      MappingProcess::Ptr mapping_ptr_;
      PlannerManager::Ptr planner_ptr_;
      PlanningVisualization::Ptr visualize_ptr_;

      /* ROS utils */
      ros::NodeHandle nh_;
      ros::Publisher traj_pub_;
      ros::Subscriber odom_sub_;
      ros::Subscriber waypoint_sub_;
      ros::Subscriber deploy_waypoint_sub_;
      ros::Timer exec_timer_, safety_timer_;

      /* fsm threshold parameters */
      double emergency_time_;
      bool enable_faile_safe_;
      double start_world_time_;
      double no_replan_thresh_, replan_thresh_;
      double planning_horizen_, planning_horizen_time_;
      
      /* vehicle parameters */
      int car_id_;
      double car_d_cr_;
      Eigen::Vector4d init_state_;
      Eigen::Vector4d end_pt_;
      Eigen::Vector2d cur_pos_;
      double cur_yaw_, cur_vel_;
      double target_x_, target_y_, target_yaw_;

      /* planning data */
      bool have_odom_, have_target_, have_new_target_;
      FSM_EXEC_STATE exec_state_;
      int continously_called_times_{0};
      
      /* state machine functions */
      void printFSMExecState();
      void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
      std::pair<int, ReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
      void execFSMCallback(const ros::TimerEvent &e);

      /* local planning for in the horizon wps */

      /* global trajectory include out of horizon wps */
      void waypointCallback(const geometry_msgs::PoseStampedPtr &msg);
      void planNextWaypoint(const Eigen::Vector3d next_wp);
      bool planFromGlobalTraj(const int trial_times = 1);
      bool planFromCurrentTraj(const int trial_times = 1);

      /* input-output */
      void OdomCallback(const nav_msgs::OdometryConstPtr &msg);

  public:
      ReplanFSM()
      {
      }
      ~ReplanFSM()
      {
      }

      void init(ros::NodeHandle &nh);
      std::string odom_topic_ = "world";

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif