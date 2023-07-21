#include <dawn_ik/joint_trajectory_control_interface.h>
#include <dawn_ik/utils.h>
namespace dawn_ik
{

JointTrajectoryControlInterface::JointTrajectoryControlInterface(ros::NodeHandle &nh, const std::string& command_topic) : nh_(nh), ruckig_(0.004)
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
//    if(current_positions !=nullptr)
//    {
//      double vel_pos_based = (target_positions[i] - current_positions[i])/0.03;
//      double vel_bounded = current_velocities[i];
//      if(fabs(vel_pos_based) < 0.005)
//      {
//        vel_bounded = 0.0;
//      }
//      double vel_smoothed = 0.1 * vel_pos_based + 0.9 * current_velocities[i];
//      vel_bounded = utils::getBoundedValueMinMax(utils::getBoundedValueMinMax(vel_smoothed, current_velocities[i]-1.57*0.03, current_velocities[i]+1.57*0.03), -1.57, 1.57);
//      point.velocities.push_back(0.5);
//    }
//    else
//    {
//      if(current_velocities[i] > 0.001)
//      {
//        point.velocities.push_back(0.5);
//      }
//      else
//      {
//        point.velocities.push_back(0.0);
//      }
//    }
  }
  point.time_from_start = ros::Duration(0.02);
  command.points.push_back(point);
  command.header.stamp = ros::Time::now();
  command_pub_.publish(command);


//  ROS_WARN("setJointPositionsWithOTG");
//  for (int target_idx=0; target_idx<robot::num_targets; target_idx++)
//  {
//    int joint_idx = robot::target_idx_to_joint_idx[target_idx];

//    input_.current_position[target_idx] = current_positions[target_idx];
//    input_.current_velocity[target_idx] = current_velocities[target_idx];

//    input_.max_velocity[target_idx] = robot::joint_max_velocity[joint_idx];
//    input_.max_acceleration[target_idx] = robot::joint_max_acceleration[joint_idx];
//    input_.max_jerk[target_idx] = 10;

//    input_.target_position[target_idx] = target_positions[target_idx];
//    input_.target_velocity[target_idx] = robot::joint_max_velocity[joint_idx];
//  }
//  trajectory_msgs::JointTrajectory command;
//  trajectory_msgs::JointTrajectoryPoint point;
//  for (int i=0; i<joint_names.size(); i++)
//  {
//    command.joint_names.push_back(joint_names[i]);
//  }
//  ROS_WARN("Before While");
//  while (ruckig_.update(input_, output_) == ruckig::Result::Working)
//  {
//    ROS_WARN("working!!!");
//    trajectory_msgs::JointTrajectoryPoint point;
//    for (int i=0; i<robot::num_targets; i++)
//    {
//      ROS_INFO("%f", output_.new_position[i]);
//      point.positions.push_back( output_.new_position[i] );
//      point.velocities.push_back( output_.new_velocity[i] );
//      //point.accelerations.push_back( output.new_acceleration[i] );
//      point.time_from_start = ros::Duration(output_.time);
//    }
//    command.points.push_back(point);
//    output_.pass_to_input(input_);
//  }
//  command.header.stamp = ros::Time::now();
//  command_pub_.publish(command);
}

void JointTrajectoryControlInterface::setJointPositionsWithOTG(const std::vector<std::string> joint_names, const double *target_positions, std::vector<double> current_positions,std::vector<double> current_velocities)
{
    ROS_WARN_THROTTLE(1.0, "setJointPositionsWithOTG");
    for (int target_idx=0; target_idx<robot::num_targets; target_idx++)
    {
      int joint_idx = robot::target_idx_to_joint_idx[target_idx];

      input_.current_position[target_idx] = current_positions[target_idx];
      input_.current_velocity[target_idx] = current_velocities[target_idx];

      input_.max_velocity[target_idx] = 6.28*10; //robot::joint_max_velocity[joint_idx];
      input_.max_acceleration[target_idx] = 6.28*10; //robot::joint_max_acceleration[joint_idx];
      input_.max_jerk[target_idx] = 2.0;

      input_.target_position[target_idx] = target_positions[target_idx];
      //input_.target_velocity[target_idx] = 0.9 * robot::joint_max_velocity[joint_idx];
    }
    trajectory_msgs::JointTrajectory command;
    trajectory_msgs::JointTrajectoryPoint point;
    for (int i=0; i<joint_names.size(); i++)
    {
      command.joint_names.push_back(joint_names[i]);
    }
    ROS_WARN_THROTTLE(1.0, "Before While");
    auto result = ruckig_.update(input_, output_);
    ROS_WARN_STREAM("Ruckig result: "<<result);
    int i = 0;
    do
    {

      trajectory_msgs::JointTrajectoryPoint point;
      for (int i=0; i<robot::num_targets; i++)
      {
        //ROS_INFO("%f", output_.new_position[i]);
        point.positions.push_back( output_.new_position[i] );
        point.velocities.push_back( output_.new_velocity[i] );
        point.accelerations.push_back(output_.new_acceleration[i] );
        point.time_from_start = ros::Duration(output_.time);
      }
      command.points.push_back(point);
      output_.pass_to_input(input_);
      result = ruckig_.update(input_, output_);
      i++;

    }
    while (result== ruckig::Result::Working);

    ROS_WARN_STREAM("After While: "<<i);
    command.header.stamp = ros::Time::now();
    command_pub_.publish(command);
}


} // namespace dawn_ik
