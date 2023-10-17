#include <plan_manage/planner_manager.h>
#include <thread>

namespace car_planner
{
  // SECTION interfaces for setup and query

  PlannerManager::PlannerManager() {}
  PlannerManager::~PlannerManager() {}

  void PlannerManager::initPlanModules(ros::NodeHandle &nh, MappingProcess::Ptr grid_map_2d, PlanningVisualization::Ptr vis)
  {
    /* read algorithm parameters */
    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
    // nh.param("manager/use_distinctive_trajs", pp_.use_distinctive_trajs, false);
    nh.param("manager/car_id", pp_.car_id, -1);

    mapping_ptr_ = grid_map_2d;
    visualize_ptr_ = vis;
  }

  bool PlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                      const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    // TODO: KinoAstar generate path given to points
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    double time_now = ros::Time::now().toSec();
    global_data_.setGlobalTraj(gl_traj, time_now, pp_.car_id);
    return true;
  }

  traj_utils::PolynomialTrajectory PlannerManager::polyTraj2rosMsg(PolynomialTraj &traj){
    static int count = 0;
    ros::Time t = ros::Time::now();
    traj_utils::PolynomialTrajectory traj_msg;
    traj_msg.header.seq = count;
    traj_msg.header.stamp = t;
    traj_msg.header.frame_id = std::string("world");
    traj_msg.trajectory_id = count;
    traj_msg.num_order = traj.getOrder(t.toSec()); // the order of polynomial
    traj_msg.num_segment = traj.getPieceNum();
    traj_msg.start_yaw = 0;
    traj_msg.final_yaw = 0;
    // cout << "p_order:" << poly_number << endl;
    // cout << "traj_msg.num_order:" << traj_msg.num_order << endl;
    // cout << "traj_msg.num_segment:" << traj_msg.num_segment << endl;

    vector<vector<double>> coef_x = traj.getCoef(0);
    vector<vector<double>> coef_y = traj.getCoef(1);
    vector<vector<double>> coef_z = traj.getCoef(2);
    vector<double> time = traj.getTimes();
    vector<int> order = traj.getOrders();
    
    for(unsigned int i=0; i<traj_msg.num_segment; i++)
    {
        for (unsigned int j = 0; j < order[i]; j++)
        {
          traj_msg.coef_x.push_back(coef_x[i][j]);
          traj_msg.coef_y.push_back(coef_y[i][j]);
          traj_msg.coef_z.push_back(coef_z[i][j]);
        }
        traj_msg.time.push_back(time[i]);
        traj_msg.order.push_back(order[i]);
    }
    traj_msg.mag_coeff = 1;
    count++;
    return traj_msg;
  }
} // namespace car_planner
