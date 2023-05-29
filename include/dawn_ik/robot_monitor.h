#ifndef DAWN_IK_ROBOT_MONITOR_H
#define DAWN_IK_ROBOT_MONITOR_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <dawn_ik/JointLinkState.h>
#include <dawn_ik/JointLinkCollisionState.h>

#include <hpp/fcl/collision.h>
#include <hpp/fcl/broadphase/broadphase.h>
using hpp::fcl::DynamicAABBTreeCollisionManager;
using hpp::fcl::FCL_REAL;
using hpp::fcl::Transform3f;
using hpp::fcl::Vec3f;
using hpp::fcl::CollisionObject;
using hpp::fcl::Matrix3f;

#include <algorithm> // std::find
#include <vector>
#include <string>
#include <boost/thread.hpp>
#include <mutex>
#include <chrono>
using namespace std::chrono;

#include <dawn_ik/utils.h> // computeLinkTranslation, computeLinkRotation
#include <dawn_ik/robot_configuration/robot_configuration.h>


namespace dawn_ik
{

struct CollisionCallBackCollect : hpp::fcl::CollisionCallBackBase {
  typedef std::pair<CollisionObject*, CollisionObject*> CollisionPair;
  CollisionCallBackCollect();
  bool collide(CollisionObject* o1, CollisionObject* o2);
  size_t numCollisionPairs() const;
  const std::vector<CollisionPair>& getCollisionPairs() const;
  void init();
  bool exist(const CollisionPair& pair) const;
  virtual ~CollisionCallBackCollect(){};

 protected:
  std::vector<CollisionPair> collision_pairs;
  size_t max_size;
};

class RobotMonitor
{
public:
  RobotMonitor(ros::NodeHandle &nh, ros::NodeHandle &priv_nh);
  ~RobotMonitor();
  const JointLinkCollisionStateConstPtr getState();
  const std::vector<CollisionObject*> getInternalObjects(){ return int_collision_objects; }

private:
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);

  void updateLinkThread();
  JointLinkStatePtr computeJointLinkState(const sensor_msgs::JointStateConstPtr& msg);

  void updateCollisionThread();
  JointLinkCollisionStatePtr computeJointLinkCollisionState(const JointLinkStateConstPtr& msg);

  void updateVisualizationThread();
  void computeAndPublishVisualization(const JointLinkCollisionStateConstPtr& msg);

private:
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh;
  ros::Subscriber joint_state_sub;
  ros::Publisher visualization_pub;
  boost::thread* link_state_thread;
  boost::thread* collision_state_thread;
  boost::thread* visualization_thread;

  std::mutex int_collision_manager_mtx;
  DynamicAABBTreeCollisionManager int_collision_manager;
  std::vector<CollisionObject*> int_collision_objects;

  std::mutex ext_collision_manager_mtx;
  DynamicAABBTreeCollisionManager ext_collision_manager;
  std::vector<CollisionObject*> ext_collision_objects;

  std::mutex last_joint_state_mtx;
  sensor_msgs::JointStateConstPtr last_joint_state_msg;
  int joint_idx_to_msg_idx[robot::num_joints];

  std::mutex last_joint_link_state_mtx;
  JointLinkStatePtr last_joint_link_state_msg;

  std::mutex last_joint_link_collision_state_mtx;
  JointLinkCollisionStatePtr last_joint_link_collision_state_msg;

};

} // namespace dawn_ik


#endif // DAWN_IK_ROBOT_MONITOR_H