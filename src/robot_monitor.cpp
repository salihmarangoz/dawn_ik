#include <salih_marangoz_thesis/robot_monitor.h>

namespace salih_marangoz_thesis
{

RobotMonitor::RobotMonitor(ros::NodeHandle &nh, 
                           const std::string joint_states_topic,
                           int num_joints,
                           int num_links,
                           const std::string* joint_names,
                           const int* joint_child_link_idx,
                           const int* joint_parent_link_idx,
                           const double* link_transform_translation_only,
                           const double* link_transform_quaternion_only
                           ): nh_(nh), 
                              joint_states_topic_(joint_states_topic), 
                              num_joints_(num_joints), 
                              num_links_(num_links), 
                              joint_names_(joint_names),
                              joint_child_link_idx_(joint_child_link_idx),
                              joint_parent_link_idx_(joint_parent_link_idx),
                              link_transform_translation_only_(link_transform_translation_only),
                              link_transform_quaternion_only_(link_transform_quaternion_only)
{
  last_joint_state_msg_ = nullptr;
  last_joint_state_is_dirty_ = true;
  global_link_transformations_ = nullptr;
  joint_idx_to_msg_idx_ = nullptr;

  joint_states_sub_ = nh_.subscribe(joint_states_topic, 2, &RobotMonitor::jointStateCallback_, this);
  ROS_WARN("init!");
}

RobotMonitor::~RobotMonitor()
{
  if (global_link_transformations_ != nullptr) delete[] global_link_transformations_;
  if (joint_idx_to_msg_idx_ != nullptr) delete[] joint_idx_to_msg_idx_;
}

void 
RobotMonitor::jointStateCallback_(const sensor_msgs::JointStateConstPtr& msg)
{
  ROS_INFO_ONCE("RobotMonitor: first jointStateCallback_");
  last_joint_state_msg_ = msg;
  last_joint_state_is_dirty_ = true;
}

const double* 
RobotMonitor::getGlobalLinkTransformations(bool return_cached_transformations)
{
  // Return cached results if possible (probably not possible at 500hz...)
  if (global_link_transformations_ != nullptr)
    if (!last_joint_state_is_dirty_ || return_cached_transformations)
      return global_link_transformations_;

  // If the joint state message is not received
  if (last_joint_state_msg_ == nullptr) return nullptr;

  // Process the latest joint state msg
  sensor_msgs::JointStateConstPtr current_joint_state_msg = last_joint_state_msg_; // hold the joint states msg shared ptr
  if (joint_idx_to_msg_idx_ == nullptr)
  {
    joint_idx_to_msg_idx_ = new int[num_joints_];
    for (int i=0; i<num_joints_; i++)
    { 
      ptrdiff_t pos = find(current_joint_state_msg->name.begin(), current_joint_state_msg->name.end(), joint_names_[i]) - current_joint_state_msg->name.begin();
      if(pos < current_joint_state_msg->name.size())
      {
        joint_idx_to_msg_idx_[i] = pos; // i->joint_idx, pos->msg_idx
      }
      else
      {
        joint_idx_to_msg_idx_[i] = -1;
      }
    }
  }

  // recompute kinematic chain
  if (global_link_transformations_ == nullptr) global_link_transformations_ = new double[num_links_*7];

  for (int i=0; i<num_joints_; i++)
  {
    int child_link_idx = joint_child_link_idx_[i];
    int parent_link_idx = joint_parent_link_idx_[i];
    int msg_idx = joint_idx_to_msg_idx_[i];

    // init
    if (parent_link_idx == -1)
    {
      global_link_transformations_[7*child_link_idx+0] = 0.0;
      global_link_transformations_[7*child_link_idx+1] = 0.0;
      global_link_transformations_[7*child_link_idx+2] = 0.0;
      global_link_transformations_[7*child_link_idx+3] = 1.0;
      global_link_transformations_[7*child_link_idx+4] = 0.0;
      global_link_transformations_[7*child_link_idx+5] = 0.0;
      global_link_transformations_[7*child_link_idx+6] = 0.0;
      continue;
    }

    // Translation
    if (robot::link_can_skip_translation[child_link_idx])
    {
        global_link_transformations_[7*child_link_idx+0] = global_link_transformations_[7*parent_link_idx+0];
        global_link_transformations_[7*child_link_idx+1] = global_link_transformations_[7*parent_link_idx+1];
        global_link_transformations_[7*child_link_idx+2] = global_link_transformations_[7*parent_link_idx+2];
    }
    else
    {
      utils::computeLinkTranslation(&(global_link_transformations_[7*parent_link_idx]), // translation
                                    &(global_link_transformations_[7*parent_link_idx+3]),  // rotation
                                    &(link_transform_translation_only_[3*child_link_idx]), 
                                    &(global_link_transformations_[7*child_link_idx]));
    }

    if (msg_idx!=-1) // if joint can move
    { 
      double joint_val = current_joint_state_msg->position[msg_idx];
      
      if (robot::link_can_skip_rotation[child_link_idx]) // if can skip the rotation then only rotate using the joint position
      {
        utils::computeLinkRotation(&(global_link_transformations_[7*parent_link_idx+3]),
                                  joint_val, 
                                  &(global_link_transformations_[7*child_link_idx+3]));
      }
      else // if link has rotation and joint has rotation, then we need to rotate using both
      {
        utils::computeLinkRotation(&(global_link_transformations_[7*parent_link_idx+3]), 
                                  &(link_transform_quaternion_only_[4*child_link_idx]), 
                                  joint_val, 
                                  &(global_link_transformations_[7*child_link_idx+3]));
      }
    }
    else // if joint is static
    {
      if (robot::link_can_skip_rotation[child_link_idx]) // if can skip the rotation then no need to do anything
      {
        global_link_transformations_[7*child_link_idx+3] = global_link_transformations_[7*parent_link_idx+3];
        global_link_transformations_[7*child_link_idx+4] = global_link_transformations_[7*parent_link_idx+4];
        global_link_transformations_[7*child_link_idx+5] = global_link_transformations_[7*parent_link_idx+5];
        global_link_transformations_[7*child_link_idx+6] = global_link_transformations_[7*parent_link_idx+6];
      }
      else // if link has a rotation, only compute that
      {
        utils::computeLinkRotation(&(global_link_transformations_[7*parent_link_idx+3]), // global parent rotation
                                  &(link_transform_quaternion_only_[4*child_link_idx]), // local child rotation
                                  &(global_link_transformations_[7*child_link_idx+3]));  // global child rotation
      }

    }
  }

  last_joint_state_is_dirty_ = false;
  return global_link_transformations_;
}



} // namespace salih_marangoz_thesis