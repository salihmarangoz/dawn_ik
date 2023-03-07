#include <salih_marangoz_thesis/robot_monitor.h>

namespace salih_marangoz_thesis
{

RobotMonitor::RobotMonitor(ros::NodeHandle &nh,
                           const std::string joint_states_topic,
                           double async_thread_rate_limit,
                           int num_joints,
                           int num_links,
                           const std::string* joint_names,
                           const int* joint_child_link_idx,
                           const int* joint_parent_link_idx,
                           const double* link_transform_translation_only,
                           const double* link_transform_quaternion_only,
                           const int* link_can_skip_translation,
                           const int* link_can_skip_rotation
                           ): nh_(nh), 
                              joint_states_topic_(joint_states_topic),
                              async_thread_rate_limit_(async_thread_rate_limit),
                              num_joints_(num_joints), 
                              num_links_(num_links), 
                              joint_names_(joint_names),
                              joint_child_link_idx_(joint_child_link_idx),
                              joint_parent_link_idx_(joint_parent_link_idx),
                              link_transform_translation_only_(link_transform_translation_only),
                              link_transform_quaternion_only_(link_transform_quaternion_only),
                              link_can_skip_translation_(link_can_skip_translation),
                              link_can_skip_rotation_(link_can_skip_rotation)
{
  last_joint_state_msg_ = nullptr;
  joint_state_is_dirty_ = true;
  joint_idx_to_msg_idx_ = nullptr;
  async_thread_ = nullptr;
  if (async_thread_rate_limit > 0) 
    async_thread_ = new boost::thread(boost::bind(&RobotMonitor::asyncThread_, this));
  joint_states_sub_ = nh_.subscribe(joint_states_topic, 2, &RobotMonitor::jointStateCallback_, this);
}

RobotMonitor::~RobotMonitor()
{
  if (joint_idx_to_msg_idx_ != nullptr) delete[] joint_idx_to_msg_idx_;

  if (async_thread_ != nullptr)
  {
    async_thread_->join();
    delete async_thread_;
  }
}

void
RobotMonitor::jointStateCallback_(const sensor_msgs::JointStateConstPtr& msg)
{
  // sanity checks
  ROS_INFO_ONCE("RobotMonitor: Received joint state!");
  if (last_joint_state_msg_ != nullptr)
  {
    double update_freq = 1. / (msg->header.stamp - last_joint_state_msg_->header.stamp).toSec();
    if (update_freq < 50) ROS_WARN_THROTTLE(5.0, "RobotMonitor: Joint state update freq is low: %f", update_freq); // fixed 50hz threshold is good, I think
  }

  // cache joint_idx_to_msg_idx_
  if (joint_idx_to_msg_idx_ == nullptr)
  {
    joint_idx_to_msg_idx_ = new int[num_joints_];
    for (int i=0; i<num_joints_; i++)
    { 
      ptrdiff_t pos = find(msg->name.begin(), msg->name.end(), joint_names_[i]) - msg->name.begin();
      if(pos < msg->name.size())
      {
        joint_idx_to_msg_idx_[i] = pos; // i->joint_idx, pos->msg_idx
      }
      else
      {
        joint_idx_to_msg_idx_[i] = -1;
      }
    }
  }

  // save the message
  msg_mtx_.lock();
  last_joint_state_msg_ = msg;
  joint_state_is_dirty_ = true;
  msg_mtx_.unlock();

  // process the message here if async thread is disabled
  if (async_thread_rate_limit_ <= 0)
  {
    // Update internal state
    JointLinkStatePtr new_state = computeJointLinkState_();
    if (new_state != nullptr)
    {
      joint_state_is_dirty_ = false; // assumed that locking msg_mtx_ is not necessary
      async_mtx_.lock();
      async_state_ = new_state;
      async_mtx_.unlock();
    }
  }
}

const JointLinkStateConstPtr
RobotMonitor::getJointLinkState()
{
  async_mtx_.lock();
  JointLinkStatePtr tmp = async_state_;
  async_mtx_.unlock();
  return tmp;
}

void
RobotMonitor::asyncThread_()
{
  ros::Rate r(async_thread_rate_limit_);
  while (ros::ok())
  {
    if (joint_state_is_dirty_)
    {
      // Update internal state
      JointLinkStatePtr new_state = computeJointLinkState_();
      if (new_state != nullptr)
      {
        joint_state_is_dirty_ = false; // assumed that locking msg_mtx_ is not necessary
        async_mtx_.lock();
        async_state_ = new_state;
        async_mtx_.unlock();
      }
    }
    r.sleep();
  }
}

JointLinkStatePtr
RobotMonitor::computeJointLinkState_()
{
  // If the joint state message is not received
  if (last_joint_state_msg_ == nullptr) return nullptr;

  JointLinkStatePtr state = boost::make_shared<JointLinkState>();

  // Process the latest joint state msg
  msg_mtx_.lock();
  state->joint_state = *last_joint_state_msg_; // copy
  msg_mtx_.unlock();

  // recompute kinematic chain
  std::vector<double> &global_link_transformations = state->link_state.transformations; // copy reference
  global_link_transformations.resize(num_links_*7);
  for (int i=0; i<num_joints_; i++)
  {
    int child_link_idx = joint_child_link_idx_[i];
    int parent_link_idx = joint_parent_link_idx_[i];
    int msg_idx = joint_idx_to_msg_idx_[i];

    // init
    if (parent_link_idx == -1)
    {
      global_link_transformations[7*child_link_idx+0] = 0.0;
      global_link_transformations[7*child_link_idx+1] = 0.0;
      global_link_transformations[7*child_link_idx+2] = 0.0;
      global_link_transformations[7*child_link_idx+3] = 1.0;
      global_link_transformations[7*child_link_idx+4] = 0.0;
      global_link_transformations[7*child_link_idx+5] = 0.0;
      global_link_transformations[7*child_link_idx+6] = 0.0;
      continue;
    }

    // Translation
    if (link_can_skip_translation_[child_link_idx])
    {
        global_link_transformations[7*child_link_idx+0] = global_link_transformations[7*parent_link_idx+0];
        global_link_transformations[7*child_link_idx+1] = global_link_transformations[7*parent_link_idx+1];
        global_link_transformations[7*child_link_idx+2] = global_link_transformations[7*parent_link_idx+2];
    }
    else
    {
      utils::computeLinkTranslation(&(global_link_transformations[7*parent_link_idx]), // translation
                                    &(global_link_transformations[7*parent_link_idx+3]),  // rotation
                                    &(link_transform_translation_only_[3*child_link_idx]), 
                                    &(global_link_transformations[7*child_link_idx]));
    }

    if (msg_idx!=-1) // if joint can move
    { 
      double joint_val = state->joint_state.position[msg_idx];
      
      if (link_can_skip_rotation_[child_link_idx]) // if can skip the rotation then only rotate using the joint position
      {
        utils::computeLinkRotation(&(global_link_transformations[7*parent_link_idx+3]),
                                  joint_val, 
                                  &(global_link_transformations[7*child_link_idx+3]));
      }
      else // if link has rotation and joint has rotation, then we need to rotate using both
      {
        utils::computeLinkRotation(&(global_link_transformations[7*parent_link_idx+3]), 
                                  &(link_transform_quaternion_only_[4*child_link_idx]), 
                                  joint_val, 
                                  &(global_link_transformations[7*child_link_idx+3]));
      }
    }
    else // if joint is static
    {
      if (link_can_skip_rotation_[child_link_idx]) // if can skip the rotation then no need to do anything
      {
        global_link_transformations[7*child_link_idx+3] = global_link_transformations[7*parent_link_idx+3];
        global_link_transformations[7*child_link_idx+4] = global_link_transformations[7*parent_link_idx+4];
        global_link_transformations[7*child_link_idx+5] = global_link_transformations[7*parent_link_idx+5];
        global_link_transformations[7*child_link_idx+6] = global_link_transformations[7*parent_link_idx+6];
      }
      else // if link has a rotation, only compute that
      {
        utils::computeLinkRotation(&(global_link_transformations[7*parent_link_idx+3]), // global parent rotation
                                  &(link_transform_quaternion_only_[4*child_link_idx]), // local child rotation
                                  &(global_link_transformations[7*child_link_idx+3]));  // global child rotation
      }

    }
  }

  return state;
}



} // namespace salih_marangoz_thesis