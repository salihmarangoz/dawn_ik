#ifndef DAWN_IK_JOINT_TRAJECTORY_CONTROL_INTERFACE
#define DAWN_IK_JOINT_TRAJECTORY_CONTROL_INTERFACE

#include <ros/ros.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// http://wiki.ros.org/joint_trajectory_controller
// http://wiki.ros.org/joint_trajectory_controller/UnderstandingTrajectoryReplacement

namespace dawn_ik
{

class JointTrajectoryControlInterface
{
public:
  JointTrajectoryControlInterface(ros::NodeHandle &nh, const std::string& controller_topic = "joint"); // TODO: mimic rqt_joint_trajectory_controller ? 
  bool start(const std::string& controller);
  bool stop();
  void setSpeedScaling(double scale); // scale: [0.0-1.0]
  const std::vector<std::string> getJointNames();
  void setJointPositions(const double *positions);
  control_msgs::JointTrajectoryControllerStateConstPtr getState();

private:
  control_msgs::JointTrajectoryControllerStateConstPtr state_;
  trajectory_msgs::JointTrajectory command_;
  ros::Subscriber state_sub_;
  ros::Publisher  command_pub_;
  ros::NodeHandle nh_;
  double speed_scale_;
  bool is_started_;
  std::string command_topic_;
  std::string state_topic_;
  void stateCallback_(const control_msgs::JointTrajectoryControllerStateConstPtr& msg);
};




} // namespace dawn_ik

#endif // DAWN_IK_JOINT_TRAJECTORY_CONTROL_INTERFACE
