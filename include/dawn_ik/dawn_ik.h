#ifndef DAWN_IK_H
#define DAWN_IK_H

#include <random>

#include <ros/ros.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h> // TODO

#include <dawn_ik/robot_configuration/robot_configuration.h>
#include <dawn_ik/joint_trajectory_control_interface.h>
#include <dawn_ik/utils.h>
#include <dawn_ik/robot_monitor.h>
#include <dawn_ik/goals.h>

/////////////////////////////////////// EXPERIMENTAL /////////////////////////////////////////////////////////////
namespace ceres {
class RelaxedIKLoss final : public LossFunction {
 public:
  explicit RelaxedIKLoss(double a);
  void Evaluate(double, double*) const override;

 private:
  const double a_;
};
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace dawn_ik
{

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

class DawnIK
{
// my classes
public:
  std::shared_ptr<RobotMonitor> robot_monitor;
public:
  std::random_device rand_dev;
  std::mt19937 rand_gen;
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh;
  std::shared_ptr<JointTrajectoryControlInterface> joint_controller;

  // TODO
  void subscriberCallback(const visualization_msgs::InteractiveMarkerFeedbackPtr &msg);
  ros::Subscriber endpoint_sub; 
  Eigen::Vector3d endpoint;
  bool endpoint_received = false;
  Eigen::Quaterniond direction;

  DawnIK(ros::NodeHandle &nh, ros::NodeHandle &priv_nh);
  void loop();
  bool update();

};


} // namespace dawn_ik


#endif // DAWN_IK_H