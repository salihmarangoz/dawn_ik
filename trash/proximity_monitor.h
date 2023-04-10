#ifndef __PROXIMITY_MONITOR_H__
#define __PROXIMITY_MONITOR_H__

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <algorithm> // std::find
#include <vector>
#include <string>
#include <boost/thread.hpp>
#include <mutex>

#include <salih_marangoz_thesis/utils.h> // computeLinkTranslation, computeLinkRotation
#include <salih_marangoz_thesis/JointLinkState.h>
#include <salih_marangoz_thesis/JointLinkProximityState.h>



#ifndef PROXIMITY_MONITOR_NO_DEFAULTS
  #include <salih_marangoz_thesis/robot_configuration/robot_configuration.h>
  #define ROBOT_MON_DEF_joint_link_state_topic = std::string("int_joint_link_state")
  #define ROBOT_MON_DEF_joint_link_proximity_state_topic = std::string("int_joint_link_proximity_state")
  #define ROBOT_MON_DEF_async_thread_rate_limit = -1 // non-positive: collision objects processed in the callback. positive: collision objects processed in a separate thread with a fixed rate


  #define ROBOT_MON_DEF_num_links = robot::num_links
#endif

namespace salih_marangoz_thesis
{

class ProximityMonitor
{
public:
  ProximityMonitor(ros::NodeHandle &nh,
               ros::NodeHandle &priv_nh,
               const std::string joint_link_state_topic ROBOT_MON_DEF_joint_link_state_topic,
               const std::string joint_link_proximity_state_topic ROBOT_MON_DEF_joint_link_proximity_state_topic,
               double async_thread_rate_limit ROBOT_MON_DEF_async_thread_rate_limit,
               int num_links ROBOT_MON_DEF_num_links,
               );
  ~ProximityMonitor();

  const JointLinkProximityStateConstPtr getJointLinkProximityState();

private:
  JointLinkStatePtr computeJointLinkState_();
  void jointStateCallback_(const sensor_msgs::JointStateConstPtr& msg);
  void asyncThread_();

private:
  // from the constructor
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;
  const std::string joint_link_state_topic_;
  const std::string joint_link_proximity_state_topic_;






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


#endif // __PROXIMITY_MONITOR_H__