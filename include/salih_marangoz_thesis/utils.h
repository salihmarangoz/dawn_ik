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


// TODO: Eigen::Map<const Eigen::Matrix<T, 4, 1> > point(input_point);
// https://groups.google.com/g/ceres-solver/c/7ZH21XX6HWU/m/kX-2n4vbAwAJ

// Computes translation of the link
// Refrain from using this function if link_translation is all zero.
template <typename T>
inline void computeLinkTranslation(const T* current_translation, const T* current_rotation, const T* link_translation, T* result)
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
inline void computeLinkRotation(const T* current_rotation, const T& joint_value, T* result)
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
inline void computeLinkRotation(const T* current_rotation, const T* link_rotation, const T& joint_value, T* result)
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
inline void computeLinkRotation(const T* current_rotation, const T* link_rotation, T* result)
{
  ceres::QuaternionProduct(current_rotation, link_rotation, result);
}

// distSphere2Sphere for automatic differentiation
// double fs[3] = {0,0,0};
// double ss[3] = {1,1,1};
// double dist = utils::distSphere2Sphere(fs, 0.1, ss, 0.2);
template <typename T>
inline T distSphere2Sphere(const T* first_sphere_pos, double first_sphere_radius, const T* second_sphere_pos, double second_sphere_radius)
{
  Eigen::Map<const Eigen::Matrix<T, 3, 1>> first_sphere_pos_e(first_sphere_pos);
  Eigen::Map<const Eigen::Matrix<T, 3, 1>> second_sphere_pos_e(second_sphere_pos);
  return (first_sphere_pos_e - second_sphere_pos_e).norm() - first_sphere_radius - second_sphere_radius;
}

template<typename T>
void eulerToMatrix(double a, double b, double c, Eigen::Matrix3d& R) {
  double c1 = cos(a);
  double c2 = cos(b);
  double c3 = cos(c);
  double s1 = sin(a);
  double s2 = sin(b);
  double s3 = sin(c);

  R << c1 * c2, -c2 * s1, s2, c3 * s1 + c1 * s2 * s3, c1 * c3 - s1 * s2 * s3,
      -c2 * s3, s1 * s3 - c1 * c3 * s2, c3 * s1 * s2 + c1 * s3, c2 * c3;
}

} // namespace utils

#endif // __SALIH_MARANGOZ_UTILS_H__