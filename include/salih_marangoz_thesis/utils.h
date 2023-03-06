#ifndef __SALIH_MARANGOZ_UTILS_H__
#define __SALIH_MARANGOZ_UTILS_H__

// COMPILE TIME COMPUTATIONS AND CONSTEXPR UTILS
#include <stddef.h> // ptrdiff_t

// CERES UTILS
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// VISUALIZATION UTILS
#include <salih_marangoz_thesis/robot_configuration/robot_configuration.h> // TODO: this shouldn't be used here
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <moveit/robot_state/robot_state.h>

namespace utils
{

////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////// COMPILE TIME COMPUTATIONS AND CONSTEXPR UTILS ////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

// util: find number of elements in an static array

template< class Type, ptrdiff_t n > ptrdiff_t countOf( Type (&)[n] ) { return n; }

// Try to use countOf instead, use this if other option doesn't compile
#define COUNTOF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))



////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////// CERES UTILS //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
// TODO: Eigen::Map<const Eigen::Matrix<T, 4, 1> > point(input_point);
// https://groups.google.com/g/ceres-solver/c/7ZH21XX6HWU/m/kX-2n4vbAwAJ

// Computes translation of the link
// Refrain from using this function if link_translation is all zero.
template <typename T>
inline void computeLinkTranslation(const T current_translation[3], const T current_rotation[4], const T link_translation[3], T result[3])
{
  T rotated_pt[3];
  ceres::QuaternionRotatePoint(current_rotation, link_translation, rotated_pt); // maybe safe option with UnitQuaternionRotatePoint ???

  result[0] = rotated_pt[0] + current_translation[0];
  result[1] = rotated_pt[1] + current_translation[1];
  result[2] = rotated_pt[2] + current_translation[2];
}


// Computes rotation of the link using the joint (excluding rotation of the link)
// ASSUMES THAT ROTATION OCCURS ON THE Z ANGLE AXIS
// WORKS WELL WITH THE FIRST ORDER OF GRADIENTS
template <typename T>
inline void computeLinkRotation(const T current_rotation[4], const T& joint_value, T result[4])
{
  T angle_axis[3];// = {0.0, 0.0, joint_value}; // z axis!
  angle_axis[0] = T(0.0);
  angle_axis[1] = T(0.0);
  angle_axis[2] = joint_value;
  T joint_rotation[4];
  ceres::AngleAxisToQuaternion(angle_axis, joint_rotation);
  ceres::QuaternionProduct(current_rotation, joint_rotation, result);
}


// Computes rotation of the link using the joint (including rotation of the link)
// ASSUMES THAT ROTATION OCCURS ON THE Z ANGLE AXIS
// WORKS WELL WITH THE FIRST ORDER OF GRADIENTS
template <typename T>
inline void computeLinkRotation(const T current_rotation[4], const T link_rotation[4], const T& joint_value, T result[4])
{
  T angle_axis[3];// = {0.0, 0.0, joint_value}; // z axis!
  angle_axis[0] = T(0.0);
  angle_axis[1] = T(0.0);
  angle_axis[2] = joint_value;
  T joint_rotation[4];
  ceres::AngleAxisToQuaternion(angle_axis, joint_rotation);
  T link_and_joint_rotation[4];
  ceres::QuaternionProduct(link_rotation, joint_rotation, link_and_joint_rotation);
  ceres::QuaternionProduct(current_rotation, link_and_joint_rotation, result);
}

// Computes rotation of the link without the joint (including rotation of the link)
// WORKS WELL WITH THE FIRST ORDER OF GRADIENTS
template <typename T>
inline void computeLinkRotation(const T current_rotation[4], const T link_rotation[4], T result[4])
{
  ceres::QuaternionProduct(current_rotation, link_rotation, result);
}

/*
  while(ros::ok())
  {
    double fs[3] = {0,0,0};
    double ss[3] = {1,1,1};
    double dist = utils::distSphere2Sphere(fs, 0.1, ss, 0.2);
    ROS_WARN("dist: %f", dist);
  }
*/
template <typename T>
inline T distSphere2Sphere(const T first_sphere_pos[3], double first_sphere_radius, const T second_sphere_pos[3], double second_sphere_radius)
{
  Eigen::Map<const Eigen::Matrix<T, 3, 1>> first_sphere_pos_e(first_sphere_pos);
  Eigen::Map<const Eigen::Matrix<T, 3, 1>> second_sphere_pos_e(second_sphere_pos);
  return (first_sphere_pos_e - second_sphere_pos_e).norm() - first_sphere_radius - second_sphere_radius;
}

template <typename T>
inline T distPoint2Sphere(const T point_pos[3], const T sphere_pos[3], double sphere_radius)
{
  Eigen::Map<const Eigen::Matrix<T, 3, 1>> point_pos_e(point_pos);
  Eigen::Map<const Eigen::Matrix<T, 3, 1>> sphere_pos_e(sphere_pos);
  return (point_pos_e - sphere_pos_e).norm() - sphere_radius;
}

template <typename T>
inline T distPoint2Point(const T first_point_pos[3], const T second_point_pos[3])
{
  Eigen::Map<const Eigen::Matrix<T, 3, 1>> first_point_pos_e(first_point_pos);
  Eigen::Map<const Eigen::Matrix<T, 3, 1>> second_point_pos_e(second_point_pos);
  return (first_point_pos_e - second_point_pos_e).norm();
}

////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////// VISUALIZATION UTILS //////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

// TODO
template <typename T>
visualization_msgs::MarkerArray visualizeCollisions(const moveit::core::RobotState& robot_state, int (&joint_idx_to_target_idx)[robot::num_joints], double (&target_values)[robot::num_targets], double* variable_positions)
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

  visualization_msgs::MarkerArray arr;
  

  // Put collisions

  


  for (int i=0; i<countOf(robot::collisions); i++)
  {
    visualization_msgs::Marker marker;
    const robot::Collision &obj = robot::collisions[i];

    T obj_pos[3];
    T result[3];
    obj_pos[0] = obj.x;
    obj_pos[1] = obj.y;
    obj_pos[2] = obj.z;
    computeLinkTranslation(&(global_link_translations[3*obj.link_idx]),
                           &(global_link_rotations[4*obj.link_idx]),
                           obj_pos,
                           result);

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time(0);
    marker.ns = "collisions";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = result[0];
    marker.pose.position.y = result[1];
    marker.pose.position.z = result[2];
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = obj.radius;
    marker.scale.y = obj.radius;
    marker.scale.z = obj.radius;
    marker.color.a = 0.75; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    arr.markers.push_back(marker);
  }

  return arr;
}

} // namespace utils

#endif // __SALIH_MARANGOZ_UTILS_H__