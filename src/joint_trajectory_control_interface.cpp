#include <salih_marangoz_thesis/joint_trajectory_control_interface.h>

namespace salih_marangoz_thesis
{

JointTrajectoryControlInterface::JointTrajectoryControlInterface(ros::NodeHandle &nh, const std::string& controller_topic) : nh_(nh), state_(nullptr), speed_scale_(0.5), is_started_(false)
{
  command_topic_ = controller_topic + "/command";
  state_topic_ = controller_topic + "/state";

  state_sub_ = nh_.subscribe(state_topic_, 2, &JointTrajectoryControlInterface::stateCallback_, this);
}

bool
JointTrajectoryControlInterface::start(const std::string& controller)
{
  if (is_started_)
  {
    ROS_ERROR("JointTrajectoryControlInterface: already started!");
    return is_started_;
  }

  if (controller.size() <= 0)
  {
    // TODO: automatically find a suitable controller
  }
  // TODO: switch the controller
  ROS_WARN("TODO: switch the controller");

  command_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(command_topic_, 2);
  is_started_ = true;
  return true;
}

bool
JointTrajectoryControlInterface::stop()
{
  if (!is_started_) return false;

  // TODO: switch the controller?
  ROS_WARN("TODO: switch the controller");

  command_pub_.shutdown();
  is_started_ = false;
  return true;
}

void 
JointTrajectoryControlInterface::setSpeedScaling(double scale)
{
  speed_scale_ = scale;
}

const std::vector<std::string> 
JointTrajectoryControlInterface::getJointNames()
{
  if (state_ == nullptr) return std::vector<std::string>();
  std::vector<std::string> copy_arr = state_->joint_names;
  return copy_arr;
}

void 
JointTrajectoryControlInterface::setJointPositions(const double *positions)
{
  if (!is_started_)
  {
    ROS_ERROR("JointTrajectoryControlInterface: not started yet!");
    return;
  }

  if (state_ == nullptr)
  {
    ROS_WARN_THROTTLE(1.0, "JointTrajectoryControlInterface: not initialized yet!");
    return;
  }

  if (command_pub_.getNumSubscribers() <= 0)
  {
    ROS_ERROR("JointTrajectoryControlInterface: no subscribers on the command topic. something is wrong.");
    return;
  }

  for (int i=0; i<command_.points[0].positions.size(); i++)
    command_.points[0].positions[i] = positions[i];

  // TODO: use scaling_
  ROS_WARN_ONCE("TODO: use scaling_");
  command_.points[0].time_from_start = ros::Duration(0.1);

  command_pub_.publish(command_);
}

control_msgs::JointTrajectoryControllerStateConstPtr 
JointTrajectoryControlInterface::getState()
{
  return state_;
}

void 
JointTrajectoryControlInterface::stateCallback_(const control_msgs::JointTrajectoryControllerStateConstPtr& msg)
{
  ROS_INFO_ONCE("JointTrajectoryControlInterface: initialized!");
  // init command_ (... mostly)
  //if (state_ == nullptr)
  //{
    //command_.header.frame_id = msg->header.frame_id; // not used
    command_.joint_names = msg->joint_names;
    trajectory_msgs::JointTrajectoryPoint p;
    if (msg->desired.positions.size() == msg->joint_names.size())
    {
      for (int i=0; i<msg->joint_names.size(); i++) p.positions.push_back(msg->desired.positions[i]);
    }
    else if (msg->actual.positions.size() == msg->joint_names.size())
    {
      for (int i=0; i<msg->joint_names.size(); i++) p.positions.push_back(msg->actual.positions[i]);
    }
    else
    {
      for (int i=0; i<msg->joint_names.size(); i++) p.positions.push_back(0.0);
    }
    command_.points.push_back(p);
  //}

  state_ = msg;
}

} // namespace salih_marangoz_thesis