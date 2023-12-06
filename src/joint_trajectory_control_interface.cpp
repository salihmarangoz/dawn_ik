#include <dawn_ik/joint_trajectory_control_interface.h>
#include <dawn_ik/utils.h>
namespace dawn_ik
{

JointTrajectoryControlInterface::JointTrajectoryControlInterface(ros::NodeHandle &nh, const std::string& command_topic) : nh_(nh)
{
  command_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(command_topic, 2);
  ROS_WARN("JointTrajectoryControlInterface");
}


void 
JointTrajectoryControlInterface::setJointPositions(const std::vector<std::string> joint_names, const double *target_positions, const double *current_positions, const double* current_velocities, const double *current_accelerations)
{
  if (command_pub_.getNumSubscribers() <= 0)
  {
    ROS_ERROR_THROTTLE(1.0, "Joint targets are published but there are no subscribers!");
  }

  trajectory_msgs::JointTrajectory command;
  trajectory_msgs::JointTrajectoryPoint point;
  for (int i=0; i<joint_names.size(); i++)
  {
    command.joint_names.push_back(joint_names[i]);
    point.positions.push_back(target_positions[i]);
  }
  point.time_from_start = ros::Duration(0.02);
  command.points.push_back(point);
  command.header.stamp = ros::Time::now();
  command_pub_.publish(command);
}


} // namespace dawn_ik
