#ifndef _CAR_PLAN_CONTAINER_H_
#define _CAR_PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>

#include <traj_utils/polynomial_traj.h>

using std::vector;

namespace car_planner
{
  struct LocalTrajData
  {
    PolynomialTraj traj;
    int car_id; // A negative value indicates no received trajectories.
    int traj_id;
    double duration;
    double start_time; // world time 
    double end_time;   // world time
    Eigen::Vector3d start_pos;
    double init_angle;
  };

  class GlobalTrajData
  {
  private:
  public:
    PolynomialTraj global_traj_;

    int car_id_;
    double duration_;
    double start_time_;
    double end_time_;
    Eigen::Vector3d start_pos_;
    Eigen::Vector3d start_vel_;
    Eigen::Vector3d start_acc_;

    GlobalTrajData(){}
    ~GlobalTrajData(){}

    void setPosVelAcc(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Vector3d& acc)
    {
      start_pos_ = pos;
      start_vel_ = vel;
      start_acc_ = acc;
    }

    void setGlobalTraj(const PolynomialTraj &traj, const double &world_time, const int car_id)
    {
      global_traj_ = traj;
      global_traj_.init();
      car_id_ = car_id;
      duration_ = global_traj_.getTimeSum();
      start_time_ = world_time;
      end_time_ = world_time + duration_;
    }
  };

  typedef std::vector<GlobalTrajData> SurroundTrajData;

  class TrajContainer
  {
  public:
    SurroundTrajData surround_traj;
    int traj_id = 0;

    TrajContainer() {}
    ~TrajContainer() {}
  
    void addSingulTraj(const GlobalTrajData &traj_data)
    {
      traj_id++;
      surround_traj.push_back(traj_data);
    }

    int locateSingulId(const double& t)
    {
      int number_of_surround_trajs = surround_traj.size();
      if(t < surround_traj[0].start_time_)
      {
        return 0;
      }
      else if(t >= surround_traj[number_of_surround_trajs - 1].end_time_)
      {
        return number_of_surround_trajs - 1;
      }
      for(int i = 0; i < number_of_surround_trajs; i++)
      {
        if(t >= surround_traj[i].start_time_ && t < surround_traj[i].end_time_)
          return i;
      }
    }
  };

  struct PlanParameters
  {
    /* planning algorithm parameters */
    double max_vel_, max_acc_, max_jerk_; // physical limits
    double ctrl_pt_dist;                  // distance between adjacient B-spline control points
    double feasibility_tolerance_;        // permitted ratio of vel/acc exceeding limits
    double planning_horizen_;
    bool use_distinctive_trajs;
    int car_id; // single car: car_id <= -1, swarm: car_id >= 0

    /* processing time */
    double time_search_ = 0.0;
    double time_optimize_ = 0.0;
    double time_adjust_ = 0.0;
  };

} // namespace ego_planner

#endif