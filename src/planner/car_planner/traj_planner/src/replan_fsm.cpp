#include <plan_manage/replan_fsm.h>

namespace car_planner
{
  void ReplanFSM::init(ros::NodeHandle &nh)
  {
    nh_ = nh;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;

    /* fsm parameters */
    nh_.param("mapping/odometry_topic", odom_topic_, odom_topic_);
    nh_.param("vehicle/car_d_cr", car_d_cr_, 1.015);
    nh_.param("vehicle/car_id", car_id_, 0);
    
    /* initialize main modules */
    mapping_ptr_.reset(new MappingProcess);
    mapping_ptr_->init(nh);
    visualize_ptr_.reset(new PlanningVisualization(nh));
    planner_ptr_.reset(new PlannerManager);
    planner_ptr_->initPlanModules(nh, mapping_ptr_, visualize_ptr_);

    /* callback */
    traj_pub_ = nh_.advertise<traj_utils::PolynomialTrajectory>("traj", 50);
    odom_sub_ = nh_.subscribe(odom_topic_, 1, &ReplanFSM::OdomCallback, this);
    waypoint_sub_ = nh_.subscribe("waypoints", 1, &ReplanFSM::waypointCallback, this);
    exec_timer_ = nh_.createTimer(ros::Duration(0.02), &ReplanFSM::execFSMCallback, this);
  }

  void ReplanFSM::OdomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    Eigen::Vector3d center_pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Vector3d pos2center(-car_d_cr_, 0, 0);

    Eigen::Quaterniond quaternion(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, 
                                  msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Matrix3d R = quaternion.toRotationMatrix();
    Eigen::Vector3d pos = center_pos /*+ R * pos2center*/;

    cur_pos_ = pos.head(2);
    cur_vel_ = msg->twist.twist.linear.x;
    cur_yaw_ = tf::getYaw(msg->pose.pose.orientation);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(cur_pos_(0), cur_pos_(1), 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, cur_yaw_);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "world", "car_"+to_string(car_id_)+"_pos"));

    have_odom_ = true;
  }

  void ReplanFSM::waypointCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    std::cout << "Triggered parking mode!" << std::endl;

    // trigger_ = true;
    target_x_ = msg->pose.position.x;
    target_y_ = msg->pose.position.y;
    target_yaw_ = tf::getYaw(msg->pose.orientation);

    init_state_ << cur_pos_, cur_yaw_, cur_vel_;
    end_pt_ << target_x_, target_y_, 
               target_yaw_, 1.0e-2;
    double start_time = ros::Time::now().toSec() + TIME_BUDGET;
    start_world_time_ = start_time;

    Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, target_yaw_);

    planNextWaypoint(end_wp);
  }

  void ReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void ReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {
    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, ReplanFSM::FSM_EXEC_STATE> ReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, ReplanFSM::FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void ReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 50)
    {
      // printFSMExecState();
      if (!have_target_)
        cout << "wait for goal or trigger." << endl;
      if (!have_odom_)
        cout << "no odom." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
      case INIT:
      {
        if (!have_odom_)
        {
          goto force_return;
        }
        changeFSMExecState(WAIT_TARGET, "FSM");
        break;
      }

      case WAIT_TARGET:
      {
        if (!have_target_)
          goto force_return;
        else
        {
          changeFSMExecState(SEQUENTIAL_START, "FSM");
        }
        break;
      }

      case SEQUENTIAL_START:
      {
        if (have_odom_ && have_target_)
        {
          // TODO: Slicing into multiple local trajectories and optimising it.
          bool success = planFromGlobalTraj(10);
          if (success)
          {
            changeFSMExecState(EXEC_TRAJ, "FSM");
            // publishTrajs(true);
          }
          else
          {
            ROS_ERROR("Failed to generate the first trahectory!!!");
            changeFSMExecState(SEQUENTIAL_START, "FSM");
          }
        }
        else
        {
          ROS_ERROR("No odom or no target! have_odom_=%d, have_target_=%d", have_odom_, have_target_);
        }
        break;
      }

      case GEN_NEW_TRAJ:
      {
        bool success = planFromGlobalTraj(10);
        if (success)
        {
          changeFSMExecState(EXEC_TRAJ, "FSM");
        }
        else
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
        break;
      }

      case REPLAN_TRAJ:
      {
        ros::Time t_now = ros::Time::now();
        double replan_start_time = t_now.toSec() + TIME_BUDGET;
        start_world_time_ = replan_start_time;
        if (planFromCurrentTraj(1))
        {
          changeFSMExecState(EXEC_TRAJ, "FSM");
          // publishSwarmTrajs(false);
        }
        else
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }

        break;
      }

      case EXEC_TRAJ:
      {
        /* determine if need to replan */
        ros::Time t_now = ros::Time::now();
        // TODO: only for deploy task
        if ((cur_pos_ - init_state_.head(2)).norm() > 10.0 || (t_now.toSec() - start_world_time_) > 2.5 && 
            (cur_pos_ - end_pt_.head(2)).norm() > 5.0 /*&& !collision_with_othercars_*/)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
        if ((cur_pos_ - end_pt_.head(2)).norm() < 0.5) // close to the global target
        {
          have_target_ = false;
          changeFSMExecState(WAIT_TARGET, "FSM");
          goto force_return; 
        }
        break;
      }

      case EMERGENCY_STOP:
      {
        break;
      }
    }

    force_return:;
    exec_timer_.start();
  }

  bool ReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/)
  {
    bool flag_random_poly_init;
    if (timesOfConsecutiveStateCalls().first == 1)
      flag_random_poly_init = false;
    else
      flag_random_poly_init = true;
    for (int i = 0; i < trial_times; i++)
    {
      // if (callReboundReplan(true, flag_random_poly_init))
      // {
      //   return true;
      // }
      return true;
    }
    return false;
  }

  bool ReplanFSM::planFromCurrentTraj(const int trial_times /*=1*/)
  {
    return true;
  }

  void ReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    bool success = false;
    // success = planner_ptr_->planGlobalTraj(cur_pos_, cur_yaw_, cur_vel_, next_wp.head(2), next_wp(2), Eigen::Vector3d::Zero());

    if (success)
    {
      end_pt_.head(3) = next_wp;

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_ptr_->global_data_.duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_ptr_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      have_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
      {
        changeFSMExecState(SEQUENTIAL_START, "TRIG");
      }
      else
      {
        while (exec_state_ != EXEC_TRAJ)
        {
          ros::spinOnce();
          ros::Duration(0.001).sleep();
        }
        // TODO: because of global planning, here no need to replan
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      traj_pub_.publish(planner_ptr_->polyTraj2rosMsg(planner_ptr_->global_data_.global_traj_));
      
      // visualize_ptr_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualize_ptr_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }
}