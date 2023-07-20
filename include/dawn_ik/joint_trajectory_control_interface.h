#ifndef DAWN_IK_JOINT_TRAJECTORY_CONTROL_INTERFACE
#define DAWN_IK_JOINT_TRAJECTORY_CONTROL_INTERFACE

#include <ros/ros.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <dawn_ik/robot_configuration/robot_configuration.h>
#include <ruckig/ruckig.hpp>
#include <sensor_msgs/JointState.h>
#include <map>

// http://wiki.ros.org/joint_trajectory_controller
// http://wiki.ros.org/joint_trajectory_controller/UnderstandingTrajectoryReplacement

namespace dawn_ik
{

class JointTrajectoryControlInterface
{
public:
  JointTrajectoryControlInterface(ros::NodeHandle &nh, const std::string& controller_topic = "joint_trajectory_command");
  void setJointPositions(const std::vector<std::string> joint_names, const double *target_positions, const double *current_positions=nullptr, const double* current_velocities=nullptr, const double *current_accelerations=nullptr);
  void setJointPositionsWithOTG(const std::vector<std::string> joint_names, const double *target_positions, std::vector<double> current_positions, std::vector<double> current_velocities);
private:
  ros::NodeHandle nh_;
  ros::Publisher command_pub_;
  ruckig::Ruckig<robot::num_targets> ruckig_;
  ruckig::InputParameter<robot::num_targets> input_;
  ruckig::OutputParameter<robot::num_targets> output_;
};

} // namespace dawn_ik

#endif // DAWN_IK_JOINT_TRAJECTORY_CONTROL_INTERFACE
