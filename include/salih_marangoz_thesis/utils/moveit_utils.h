#ifndef __SALIH_MARANGOZ_MOVEIT_UTILS_H__
#define __SALIH_MARANGOZ_MOVEIT_UTILS_H__

#include <geometry_msgs/Pose.h>
#include <moveit/robot_state/robot_state.h>
#include "ceres_utils.h"
#include <salih_marangoz_thesis/robot_configuration/robot_configuration.h>

namespace utils
{

// TODO: implement ceres functor using forward kinematics and jacobians of moveit
// see: http://ceres-solver.org/interfacing_with_autodiff.html



/*
// Test forwardKinematics, computeLinkTranslation, computeLinkRotation
planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
planning_scene_monitor->startSceneMonitor();
planning_scene_monitor->startStateMonitor();
ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("endpoint_pose", 1, true);
ros::Rate r(1.0);
while (ros::ok())
{
  planning_scene_monitor->waitForCurrentRobotState(ros::Time::now());
  planning_scene_monitor::LockedPlanningSceneRO lps(planning_scene_monitor);
  moveit::core::RobotState current_state = lps->getCurrentState(); // copy the current state

  geometry_msgs::PoseStamped endpoint = forwardKinematics(current_state);
  pub.publish(endpoint);
  r.sleep();
}
*/
geometry_msgs::PoseStamped forwardKinematics(const moveit::core::RobotState& robot_state)
{
  moveit::core::RobotModelConstPtr robot_model = robot_state.getRobotModel();

  // Follow the kinematic tree
  const moveit::core::JointModel* current_joint = robot_model->getRootJoint();
  ROS_INFO("Joint: %s", current_joint->getName().c_str());

  double pos[3] = {0,0,0};
  double rot[4] = {1,0,0,0};

  while (true)
  {
    // One child link is possible
    const moveit::core::LinkModel* current_link = current_joint->getChildLinkModel();
    ROS_INFO("Link: %s", current_link->getName().c_str());

    // Get link transform
    const Eigen::Isometry3d current_link_transform = current_link->getJointOriginTransform();
    if (!current_link_transform.translation().isZero())
    {
      ROS_INFO_STREAM("translation: " << current_link_transform.translation());

      double link_translation[3];
      link_translation[0] = current_link_transform.translation().x();
      link_translation[1] = current_link_transform.translation().y();
      link_translation[2] = current_link_transform.translation().z();

      double out[3];
      computeLinkTranslation(pos, rot, link_translation, out);
      pos[0] = out[0];
      pos[1] = out[1];
      pos[2] = out[2];
    }

    if (current_link_transform.rotation().isIdentity())
    {
      ROS_INFO_STREAM("rotation: " << current_link_transform.rotation() << " without joint rotation");
    }
    else
    {
      ROS_INFO_STREAM("rotation: " << current_link_transform.rotation() << " with joint rotation");
    }

    const double* pos_ = robot_state.getJointPositions(current_joint->getName());
    double pos = *pos_;

    Eigen::Quaterniond link_rot(current_link_transform.rotation());
    double link_rot_r[4];
    link_rot_r[0] = link_rot.w();
    link_rot_r[1] = link_rot.x();
    link_rot_r[2] = link_rot.y();
    link_rot_r[3] = link_rot.z();

    double out[4];
    computeLinkRotation(rot, link_rot_r, pos, out);
    rot[0] = out[0];
    rot[1] = out[1];
    rot[2] = out[2];
    rot[3] = out[3];

    // Multiple joints are possible. But here we will pick the first one.
    const std::vector<const moveit::core::JointModel*> next_joints = current_link->getChildJointModels();
    if (next_joints.size() <= 0) break; // end of the kinematic tree
    current_joint = next_joints[0];
    ROS_INFO("Joint: %s", current_joint->getName().c_str());
  }

  geometry_msgs::PoseStamped p;
  p.pose.position.x = pos[0];
  p.pose.position.y = pos[1];
  p.pose.position.z = pos[2];
  p.pose.orientation.w = rot[0];
  p.pose.orientation.x = rot[1];
  p.pose.orientation.y = rot[2];
  p.pose.orientation.z = rot[3];
  p.header.frame_id = "world";
  p.header.stamp = ros::Time::now();
  return p;
}

template <typename T>
geometry_msgs::PoseStamped forwardKinematicsDebug(const moveit::core::RobotState& robot_state, int (&joint_idx_to_target_idx)[robot::num_joints], double (&target_values)[robot::num_targets], double* variable_positions)
{
  T global_link_translations[3*robot::num_links];
  T global_link_rotations[4*robot::num_links];

  T link_translations[3*robot::num_links];
  T link_rotations[4*robot::num_links];
  for (int i=0; i<robot::num_links; i++)
  {
    link_translations[3*i+0] = T(robot::link_transform_translation_only[i][0]);
    link_translations[3*i+1] = T(robot::link_transform_translation_only[i][1]);
    link_translations[3*i+2] = T(robot::link_transform_translation_only[i][2]);
    link_rotations[4*i+0] = T(robot::link_transform_quaternion_only[i][0]);
    link_rotations[4*i+1] = T(robot::link_transform_quaternion_only[i][1]);
    link_rotations[4*i+2] = T(robot::link_transform_quaternion_only[i][2]);
    link_rotations[4*i+3] = T(robot::link_transform_quaternion_only[i][3]);
  }

  // TODO: To a separate function
  for (int i=0; i<robot::num_joints; i++)
  {
    int child_link_idx = robot::joint_child_link_idx[i];
    int parent_link_idx = robot::joint_parent_link_idx[i];
    int target_idx = joint_idx_to_target_idx[i];
    int variable_idx = robot::joint_idx_to_variable_idx[i];

    // init
    if (parent_link_idx == -1)
    {
      // TODO
      global_link_translations[3*child_link_idx+0] = T(0.0);
      global_link_translations[3*child_link_idx+1] = T(0.0);
      global_link_translations[3*child_link_idx+2] = T(0.0);
      global_link_rotations[4*child_link_idx+0] = T(1.0);
      global_link_rotations[4*child_link_idx+1] = T(0.0);
      global_link_rotations[4*child_link_idx+2] = T(0.0);
      global_link_rotations[4*child_link_idx+3] = T(0.0);
      continue;
    }

    // Translation
    utils::computeLinkTranslation(&(global_link_translations[3*parent_link_idx]), 
                                  &(global_link_rotations[4*parent_link_idx]), 
                                  &(link_translations[3*child_link_idx]), 
                                  &(global_link_translations[3*child_link_idx]));

    
    if (variable_idx!=-1) // if joint can move
    { 
      T joint_val = T(variable_positions[variable_idx]);
      utils::computeLinkRotation(&(global_link_rotations[4*parent_link_idx]), 
                                 &(link_rotations[4*child_link_idx]), 
                                 joint_val, 
                                 &(global_link_rotations[4*child_link_idx]));
    }
    else
    {
      utils::computeLinkRotation(&(global_link_rotations[4*parent_link_idx]), 
                                 &(link_rotations[4*child_link_idx]),
                                 &(global_link_rotations[4*child_link_idx]));
    }

    
  }

  geometry_msgs::PoseStamped p;
  p.pose.position.x = global_link_translations[3*robot::endpoint_link_idx+0];;
  p.pose.position.y = global_link_translations[3*robot::endpoint_link_idx+1];;
  p.pose.position.z = global_link_translations[3*robot::endpoint_link_idx+2];;
  p.pose.orientation.w = global_link_rotations[4*robot::endpoint_link_idx+0];;
  p.pose.orientation.x = global_link_rotations[4*robot::endpoint_link_idx+1];;
  p.pose.orientation.y = global_link_rotations[4*robot::endpoint_link_idx+2];;
  p.pose.orientation.z = global_link_rotations[4*robot::endpoint_link_idx+3];;
  p.header.frame_id = "world";
  p.header.stamp = ros::Time::now();
  return p;
}


} // namespace utils

#endif // __SALIH_MARANGOZ_MOVEIT_UTILS_H__