#ifndef _CAR_PLANNER_MANAGER_H_
#define _CAR_PLANNER_MANAGER_H_

#include <stdlib.h>

#include <ros/ros.h>
#include <mapping.h>
#include "traj_utils/plan_container.hpp"
#include "traj_utils/planning_visualization.h"
#include "traj_utils/PolynomialTrajectory.h"

using namespace std;

namespace car_planner
{
  // Key algorithms of planning are called

  class PlannerManager
  {
  public:
    PlannerManager();
    ~PlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */

    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    void initPlanModules(ros::NodeHandle &nh, MappingProcess::Ptr grid_map_2d, PlanningVisualization::Ptr vis = NULL);

    traj_utils::PolynomialTrajectory polyTraj2rosMsg(PolynomialTraj& traj);

    PlanParameters pp_;
    GlobalTrajData global_data_;
    TrajContainer traj_container_;

  private:
    /* main planning algorithms & modules */
    MappingProcess::Ptr mapping_ptr_;
    PlanningVisualization::Ptr visualize_ptr_;
  
  public:
    typedef unique_ptr<PlannerManager> Ptr;
  };
}

#endif