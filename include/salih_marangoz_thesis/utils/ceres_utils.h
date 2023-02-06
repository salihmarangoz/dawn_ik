#ifndef __SALIH_MARANGOZ_CERES_UTILS_H__
#define __SALIH_MARANGOZ_CERES_UTILS_H__

#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace salih_marangoz_thesis
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
template <typename T>
inline void computeLinkRotation(const T current_rotation[4], const T& joint_value, T result[4])
{
  const T angle_axis[3] = {0.0, 0.0, joint_value}; // z axis!
  T joint_rotation[4];
  ceres::AngleAxisToQuaternion(angle_axis, joint_rotation);
  ceres::QuaternionProduct(current_rotation, joint_rotation, result);
}


// Computes rotation of the link using the joint (including rotation of the link)
// ASSUMES THAT ROTATION OCCURS ON THE Z ANGLE AXIS
template <typename T>
inline void computeLinkRotation(const T current_rotation[4], const T link_rotation[4], const T& joint_value, T result[4])
{
  const T angle_axis[3] = {0.0, 0.0, joint_value}; // z axis!
  T joint_rotation[4];
  ceres::AngleAxisToQuaternion(angle_axis, joint_rotation);
  T link_and_joint_rotation[4];
  ceres::QuaternionProduct(link_rotation, joint_rotation, link_and_joint_rotation);
  ceres::QuaternionProduct(current_rotation, link_and_joint_rotation, result);
}


} // namespace salih_marangoz_thesis

#endif // __SALIH_MARANGOZ_CERES_UTILS_H__