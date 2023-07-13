#ifndef DAWN_IK_H
#define DAWN_IK_H

#include <random>
#include <mutex>
#include <deque>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h> // TODO

#include <dawn_ik/robot_configuration/robot_configuration.h>
#include <dawn_ik/joint_trajectory_control_interface.h>
#include <dawn_ik/utils.h>
#include <dawn_ik/robot_monitor.h>
#include <dawn_ik/goals.h>

#include <dawn_ik/Constraint.h>
#include <dawn_ik/IKGoal.h>
#include <dawn_ik/SolverSummary.h>

#include <dawn_ik/experimental.h> // TODO: EXPERIMENTAL STUFF!

#include <control_msgs/JointTrajectoryControllerState.h>

#include <eigen3/Eigen/Core>

namespace dawn_ik
{

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

struct IKSolution
{
  std::vector<double> target_positions;
  ceres::Solver::Summary solver_summary;
};
class DawnIK
{

public:
  DawnIK(ros::NodeHandle &nh, ros::NodeHandle &priv_nh);
  ~DawnIK();
  void loopThread();
  IKSolution update(const dawn_ik::IKGoalPtr &ik_goal, bool noisy_initialization);

private: // Parameters
  void readParameters();
  double p_update_rate;
  double p_init_noise;
  double p_max_step_size;

private:
  std::random_device rand_dev;
  std::mt19937 rand_gen;
  boost::thread *loop_thread;

  ros::NodeHandle nh;
  ros::NodeHandle priv_nh;

  std::map<std::string, int> joint_name_to_joint_idx;

  std::shared_ptr<RobotMonitor> robot_monitor;                        // solver environment input
  ros::Subscriber ik_goal_sub;                                        // solver command input
  std::shared_ptr<JointTrajectoryControlInterface> joint_controller;  // solver control output
  ros::Publisher solver_summary_pub;                                  // solver log
  std::deque< std::vector<double> > solver_history;                   // solver history

  void goalCallback(const dawn_ik::IKGoalPtr &msg);
  std::mutex ik_goal_mutex;
  dawn_ik::IKGoalPtr ik_goal_msg;

  // TODO
  void subscriberCallback(const visualization_msgs::InteractiveMarkerFeedbackPtr &msg);
  ros::Subscriber endpoint_sub; 
  Eigen::Vector3d endpoint;
  Eigen::Quaterniond direction;

  double acc_loss_weight;

  ros::Publisher debug_pub;



};


} // namespace dawn_ik


#endif // DAWN_IK_H
