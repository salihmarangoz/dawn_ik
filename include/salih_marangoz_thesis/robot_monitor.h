#ifndef __ROBOT_MONITOR_H__
#define __ROBOT_MONITOR_H__

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <algorithm> // std::find
#include <vector>
#include <string>
#include <boost/thread.hpp>
#include <mutex>

#include <salih_marangoz_thesis/utils.h> // computeLinkTranslation, computeLinkRotation
#include <salih_marangoz_thesis/JointLinkState.h>


#ifndef ROBOT_MONITOR_NO_DEFAULTS
  #include <salih_marangoz_thesis/robot_configuration/robot_configuration.h>
  #define ROBOT_MON_DEF_joint_states_topic = std::string("/joint_states")
  #define ROBOT_MON_DEF_joint_link_state_topic = std::string("int_joint_link_state")
  #define ROBOT_MON_DEF_async_thread_rate_limit = -1 // non-positive: kinematics processed in the callback. positive: kinematics processed in a separate thread with a fixed rate
  #define ROBOT_MON_DEF_num_joints = robot::num_joints
  #define ROBOT_MON_DEF_num_links = robot::num_links
  #define ROBOT_MON_DEF_joint_names = robot::joint_names
  #define ROBOT_MON_DEF_joint_child_link_idx = robot::joint_child_link_idx
  #define ROBOT_MON_DEF_joint_parent_link_idx = robot::joint_parent_link_idx
  #define ROBOT_MON_DEF_link_transform_translation_only = (double*)robot::link_transform_translation_only // knowing that 2D C arrays are contiguous
  #define ROBOT_MON_DEF_link_transform_quaternion_only = (double*)robot::link_transform_quaternion_only // knowing that 2D C arrays are contiguous
  #define ROBOT_MON_DEF_link_can_skip_translation = robot::link_can_skip_translation
  #define ROBOT_MON_DEF_link_can_skip_rotation = robot::link_can_skip_rotation
#endif

namespace salih_marangoz_thesis
{

class RobotMonitor
{
public:
  RobotMonitor(ros::NodeHandle &nh,
               ros::NodeHandle &priv_nh,
               const std::string joint_states_topic ROBOT_MON_DEF_joint_states_topic,
               const std::string joint_link_state_topic ROBOT_MON_DEF_joint_link_state_topic,
               double async_thread_rate_limit ROBOT_MON_DEF_async_thread_rate_limit,
               int num_joints ROBOT_MON_DEF_num_joints, 
               int num_links ROBOT_MON_DEF_num_links,
               const std::string* joint_names ROBOT_MON_DEF_joint_names,
               const int* joint_child_link_idx ROBOT_MON_DEF_joint_child_link_idx,
               const int* joint_parent_link_idx ROBOT_MON_DEF_joint_parent_link_idx,
               const double* link_transform_translation_only ROBOT_MON_DEF_link_transform_translation_only,
               const double* link_transform_quaternion_only ROBOT_MON_DEF_link_transform_quaternion_only,
               const int* link_can_skip_translation ROBOT_MON_DEF_link_can_skip_translation,
               const int* link_can_skip_rotation ROBOT_MON_DEF_link_can_skip_rotation
               );
  ~RobotMonitor();

  const JointLinkStateConstPtr getJointLinkState();

  // setter/getter
  int getNumJoints(){return num_joints_;}
  int getNumLinks(){return num_links_;}
  const std::string* getJointNames(){return joint_names_;}
  int getChildLinkIdx(int joint_idx){return joint_child_link_idx_[joint_idx];}
  int getParentLinkIdx(int joint_idx){return joint_parent_link_idx_[joint_idx];}
  int getMessageIdx(int joint_idx){return joint_idx_to_msg_idx_[joint_idx];}
  // const sensor_msgs::JointStateConstPtr getJointState()
  // {
  //   msg_mtx_.lock();
  //   const sensor_msgs::JointStateConstPtr tmp = last_joint_state_msg_;
  //   msg_mtx_.unlock();
  //   return tmp;
  // }
  

private:
  JointLinkStatePtr computeJointLinkState_();
  void jointStateCallback_(const sensor_msgs::JointStateConstPtr& msg);
  void asyncThread_();

private:
  // from the constructor
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;
  const std::string joint_states_topic_;
  const std::string joint_link_state_topic_;
  double async_thread_rate_limit_;
  int num_joints_;
  int num_links_;
  const std::string* joint_names_; // len: num_joints_
  const int* joint_child_link_idx_; // len: num_joints_
  const int* joint_parent_link_idx_; // len: num_joints_
  const double* link_transform_translation_only_; // len: num_links_*3
  const double* link_transform_quaternion_only_; // len: num_links_*4
  const int* link_can_skip_translation_; // len: num_links_
  const int* link_can_skip_rotation_; // len: num_links_
  // internal
  ros::Publisher joint_link_state_pub_;
  std::mutex msg_mtx_;
  sensor_msgs::JointStateConstPtr last_joint_state_msg_;
  bool joint_state_is_dirty_;
  std::mutex async_mtx_;
  JointLinkStatePtr async_state_;
  boost::thread* async_thread_;
  ros::Subscriber joint_states_sub_;
  int* joint_idx_to_msg_idx_; // len: num_joints_
};

} // namespace salih_marangoz_thesis


#endif // __ROBOT_MONITOR_H__