#ifndef __SALIH_MARANGOZ_CERES_UTILS_H__
#define __SALIH_MARANGOZ_CERES_UTILS_H__

#include <ceres/ceres.h>
#include <ceres/rotation.h>

// TODO: Eigen::Map<const Eigen::Matrix<T, 4, 1> > point(input_point);
// https://groups.google.com/g/ceres-solver/c/7ZH21XX6HWU/m/kX-2n4vbAwAJ

namespace utils
{

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
inline T distSphere2Sphere(const T first_sphere_pos[3], float first_sphere_radius, const T second_sphere_pos[3], float second_sphere_radius)
{
  Eigen::Map<const Eigen::Matrix<T, 3, 1>> first_sphere_pos_e(first_sphere_pos);
  Eigen::Map<const Eigen::Matrix<T, 3, 1>> second_sphere_pos_e(second_sphere_pos);
  return (first_sphere_pos_e - second_sphere_pos_e).norm() - first_sphere_radius - second_sphere_radius;
}

template <typename T>
inline T distPoint2Sphere(const T point_pos[3], const T sphere_pos[3], float sphere_radius)
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




} // namespace utils

#endif // __SALIH_MARANGOZ_CERES_UTILS_H__