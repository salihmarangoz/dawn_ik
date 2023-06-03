#ifndef DAWN_IK_UTILS_H
#define DAWN_IK_UTILS_H

// COMPILE TIME COMPUTATIONS AND CONSTEXPR UTILS
#include <stddef.h> // ptrdiff_t

// CERES UTILS
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// VISUALIZATION UTILS
#include <dawn_ik/robot_configuration/robot_configuration.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <moveit/robot_state/robot_state.h>

namespace utils
{

// TODO: Eigen::Map<const Eigen::Matrix<T, 4, 1> > point(input_point);
// https://groups.google.com/g/ceres-solver/c/7ZH21XX6HWU/m/kX-2n4vbAwAJ

// desmos: https://www.desmos.com/calculator/mlpskpxmuj
// do not use this function if min_value >= max_value
template <typename T>
inline T computeToleranceCost(const T value, double min_value, double max_value)
{
  double center_value = (min_value+max_value)/2.0;
  double diff = max_value - min_value;
  return ceres::tanh(ceres::pow((value-center_value)/diff, 2));
}

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

// doesnt support autodiff!!!
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

// USAGE:
// T global_link_translations[3*robot::num_links];
// T global_link_rotations[4*robot::num_links];
// utils::computeGlobalLinkTransforms(target_values, variable_positions, global_link_translations, global_link_rotations);
//
// Set target_positions=nullptr to use without autograd
template<typename JetT, typename ConstT>
inline void computeGlobalLinkTransforms(const JetT* target_positions, 
                                        const ConstT* variable_positions, 
                                        JetT* global_link_translations, 
                                        JetT* global_link_rotations)
{
  // Get link transformations from robot configuration
  JetT link_translations[3*robot::num_links];
  JetT link_rotations[4*robot::num_links];
  for (int i=0; i<robot::num_links; i++)
  {
    link_translations[3*i+0] = JetT(robot::link_transform_translation_only[i][0]);
    link_translations[3*i+1] = JetT(robot::link_transform_translation_only[i][1]);
    link_translations[3*i+2] = JetT(robot::link_transform_translation_only[i][2]);
    link_rotations[4*i+0] = JetT(robot::link_transform_quaternion_only[i][0]);
    link_rotations[4*i+1] = JetT(robot::link_transform_quaternion_only[i][1]);
    link_rotations[4*i+2] = JetT(robot::link_transform_quaternion_only[i][2]);
    link_rotations[4*i+3] = JetT(robot::link_transform_quaternion_only[i][3]);
  }

  for (int i=0; i<robot::num_joints; i++)
  {
    int child_link_idx = robot::joint_child_link_idx[i];
    int parent_link_idx = robot::joint_parent_link_idx[i];
    int target_idx = robot::joint_idx_to_target_idx[i];
    int variable_idx = robot::joint_idx_to_variable_idx[i];

    // init root link
    if (parent_link_idx == -1)
    {
      global_link_translations[3*child_link_idx+0] = JetT(0.0);
      global_link_translations[3*child_link_idx+1] = JetT(0.0);
      global_link_translations[3*child_link_idx+2] = JetT(0.0);
      global_link_rotations[4*child_link_idx+0] = JetT(1.0);
      global_link_rotations[4*child_link_idx+1] = JetT(0.0);
      global_link_rotations[4*child_link_idx+2] = JetT(0.0);
      global_link_rotations[4*child_link_idx+3] = JetT(0.0);
      continue;
    }

    // Translation
    if (robot::link_can_skip_translation[child_link_idx])
    {
      global_link_translations[3*child_link_idx+0] = global_link_translations[3*parent_link_idx+0];
      global_link_translations[3*child_link_idx+1] = global_link_translations[3*parent_link_idx+1];
      global_link_translations[3*child_link_idx+2] = global_link_translations[3*parent_link_idx+2];
    }
    else
    {
      utils::computeLinkTranslation(&(global_link_translations[3*parent_link_idx]), 
                                    &(global_link_rotations[4*parent_link_idx]), 
                                    &(link_translations[3*child_link_idx]), 
                                    &(global_link_translations[3*child_link_idx]));
    }

    // Rotation
    if (variable_idx!=-1) // if joint can move
    { 
      JetT joint_val;
      if (target_idx!=-1 && target_positions!=nullptr)
      { // this is an optimization target
        joint_val = target_positions[target_idx];
      }
      else
      { // this is a joint value but not an optimization target. But we need to keep building the computation graph
        joint_val = JetT(variable_positions[variable_idx]);
      }

      if (robot::link_can_skip_rotation[child_link_idx]) // if can skip the rotation then only rotate using the joint position
      {
        utils::computeLinkRotation(&(global_link_rotations[4*parent_link_idx]),
                                  joint_val,
                                  &(global_link_rotations[4*child_link_idx]));
      }
      else // if link has rotation and joint has rotation, then we need to rotate using both
      {
        utils::computeLinkRotation(&(global_link_rotations[4*parent_link_idx]), 
                                  &(link_rotations[4*child_link_idx]), 
                                  joint_val,
                                  &(global_link_rotations[4*child_link_idx]));
      }
    }
    else // if joint is static
    {
      if (robot::link_can_skip_rotation[child_link_idx]) // if can skip the rotation then no need to do anything
      {
        global_link_rotations[4*child_link_idx+0] = global_link_rotations[4*parent_link_idx+0];
        global_link_rotations[4*child_link_idx+1] = global_link_rotations[4*parent_link_idx+1];
        global_link_rotations[4*child_link_idx+2] = global_link_rotations[4*parent_link_idx+2];
        global_link_rotations[4*child_link_idx+3] = global_link_rotations[4*parent_link_idx+3];
      }
      else // if link has a rotation, only compute that
      {
        utils::computeLinkRotation(&(global_link_rotations[4*parent_link_idx]), 
                                  &(link_rotations[4*child_link_idx]),
                                  &(global_link_rotations[4*child_link_idx]));
      }
    }
  }
}

// can be used for internal and external collisions
template<typename JetT, typename ConstT>
inline void computeCollisionTransforms(const JetT* global_link_translation, 
                                       const JetT* global_link_rotation, 
                                       const ConstT* local_object_translation,
                                       const ConstT* local_object_rotation,
                                       const int* object_can_skip_translation,
                                       const int* object_can_skip_rotation,
                                       const int* object_idx_to_link_idx,
                                       const int num_objects,
                                       JetT* global_object_translation,
                                       JetT* global_object_rotation)
{
  for (int object_idx=0; object_idx<num_objects; object_idx++)
  {
    int link_idx = object_idx_to_link_idx[object_idx];

    // compute translation
    if (object_can_skip_translation!= nullptr && object_can_skip_translation[object_idx])
    {
      global_object_translation[object_idx*3+0] = global_link_translation[link_idx*3+0];
      global_object_translation[object_idx*3+1] = global_link_translation[link_idx*3+1];
      global_object_translation[object_idx*3+2] = global_link_translation[link_idx*3+2];
    }
    else
    {
      utils::computeLinkTranslation(&(global_link_translation[link_idx*3]), 
                                    &(global_link_rotation[link_idx*4]), 
                                    &(local_object_translation[object_idx*3]), 
                                    &(global_object_translation[object_idx*3]));
    }

    // compute rotation
    if (object_can_skip_rotation != nullptr && object_can_skip_rotation[object_idx])
    {
      global_object_rotation[object_idx*4+0] = global_link_rotation[link_idx*4+0];
      global_object_rotation[object_idx*4+1] = global_link_rotation[link_idx*4+1];
      global_object_rotation[object_idx*4+2] = global_link_rotation[link_idx*4+2];
      global_object_rotation[object_idx*4+3] = global_link_rotation[link_idx*4+3];
    }
    else
    {
      utils::computeLinkRotation(&(global_link_rotation[link_idx*4]), 
                                 &(local_object_rotation[object_idx*4]), 
                                 &(global_object_rotation[object_idx*4]));
    }
  }
}

template<typename T_in, typename T_out>
void translationRotationToTransform(const T_in* translations, const T_in* rotations, T_out* transforms, const int num_elements)
{
  for (int i=0; i<num_elements; i++)
  {
    transforms[i*7+0] = T_out(translations[i*3+0]);
    transforms[i*7+1] = T_out(translations[i*3+1]);
    transforms[i*7+2] = T_out(translations[i*3+2]);
    transforms[i*7+3] = T_out(rotations[i*4+0]);
    transforms[i*7+4] = T_out(rotations[i*4+1]);
    transforms[i*7+5] = T_out(rotations[i*4+2]);
    transforms[i*7+6] = T_out(rotations[i*4+3]);
  }
}

template<typename T_in, typename T_out>
void transformToTranslationRotation(const T_in* transforms, T_in* translations, T_out* rotations, const int num_elements)
{
  for (int i=0; i<num_elements; i++)
  {
    translations[i*3+0] = T_out(transforms[i*7+0]);
    translations[i*3+1] = T_out(transforms[i*7+1]);
    translations[i*3+2] = T_out(transforms[i*7+2]);
    rotations[i*4+0] = T_out(transforms[i*7+3]);
    rotations[i*4+1] = T_out(transforms[i*7+4]);
    rotations[i*4+2] = T_out(transforms[i*7+5]);
    rotations[i*4+3] = T_out(transforms[i*7+6]);
  }
}


//=======================================================================================================================

static inline 
double computeSquaredDistancePointToLineSegment(double p_[3], double a_[3], double b_[3])
{
  Eigen::Vector3d p(p_);
  Eigen::Vector3d a(a_);
  Eigen::Vector3d b(b_);
  auto ba = b - a;
  auto ap = a - p;
  auto c = ba.dot(ap);
  if (c>0) return ap.dot(ap);
  auto pb = p - b;
  if (ba.dot(pb)>0) return pb.dot(pb);
  auto e = ap - ba * (c / ba.dot(ba));
  return e.dot(e);
}

} // namespace utils

#endif // DAWN_IK_UTILS_H