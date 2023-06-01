#include <dawn_ik/joint_trajectory_control_interface.h>

namespace dawn_ik
{

JointTrajectoryControlInterface::JointTrajectoryControlInterface(ros::NodeHandle &nh, const std::string& command_topic) : nh_(nh), ruckig_(0.01)
{
  command_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(command_topic, 2);
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
  point.time_from_start = ros::Duration(0.1);
  command.points.push_back(point);
  command.header.stamp = ros::Time::now();
  command_pub_.publish(command);

  /*
  ruckig::InputParameter<robot::num_targets> input;
  for (int target_idx=0; target_idx<robot::num_targets; target_idx++)
  {
    int joint_idx = robot::target_idx_to_joint_idx[target_idx];

    // Current
    input.current_position[target_idx] = current_positions[target_idx];
    //if (current_velocities != nullptr)
    //  input.current_velocity[target_idx] = current_velocities[target_idx];
    //if (current_accelerations != nullptr)
    //  input.current_acceleration[target_idx] = current_accelerations[target_idx];

    // Target
    input.target_position[target_idx] = target_positions[target_idx];
    //input.target_velocity = {-0.1, ...};
    //input.target_acceleration = {0.2, ...};

    // Limits
    //if (robot::joint_is_velocity_bounded[joint_idx])
    //  input.max_velocity[target_idx] = robot::joint_max_velocity[joint_idx];
    //else
    //  input.max_velocity[target_idx] = 1.0; // safe option?

    //if (robot::joint_is_acceleration_bounded[joint_idx])
    //  input.max_acceleration[target_idx] = robot::joint_max_acceleration[joint_idx];
    //else
    //  input.max_acceleration[target_idx] = 1.0; // safe option?

    //input.max_jerk[target_idx] = 0.1;
  }

  command_.points.clear();
  ruckig::OutputParameter<robot::num_targets> output;
  ruckig::Result update_result = ruckig_.update(input, output);
  while (update_result == ruckig::Result::Working)
  {
    ROS_WARN("working!!!");
    trajectory_msgs::JointTrajectoryPoint point;
    for (int i=0; i<robot::num_targets; i++)
    {
      ROS_INFO("%f", output.new_position[i]);
      point.positions.push_back( output.new_position[i] );
      point.velocities.push_back( output.new_velocity[i] );
      //point.accelerations.push_back( output.new_acceleration[i] );
      point.time_from_start = ros::Duration(output.time);
    }
    command_.points.push_back(point);
  }
  ROS_WARN("aaaaaaaaaaaa %d", update_result);
  */
}

} // namespace dawn_ik