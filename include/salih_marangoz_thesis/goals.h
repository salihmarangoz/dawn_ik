#ifndef __SALIH_MARANGOZ_THESIS_GOALS__
#define __SALIH_MARANGOZ_THESIS_GOALS__

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <salih_marangoz_thesis/robot_configuration/robot_configuration.h>
#include <salih_marangoz_thesis/utils.h>

namespace salih_marangoz_thesis
{

/**
 * MinimalJointDisplacementGoal
*/
struct MinimalJointDisplacementGoal {
  MinimalJointDisplacementGoal(const double *init_target_positions)
  :init_target_positions(init_target_positions) {}

  template <typename T>
  bool operator()(const T* target_values, T* residuals) const // param_x, param_y, residuals
  {
    for (int i=0; i<robot::num_targets; i++)
    {
      residuals[i] = target_values[i] - init_target_positions[i];
    }
    return true;
  }

   // Factory to hide the construction of the CostFunction object from the client code.
   static ceres::CostFunction* Create(const double *init_target_positions)
   {
     return (new ceres::AutoDiffCostFunction<MinimalJointDisplacementGoal, robot::num_targets, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
                 new MinimalJointDisplacementGoal(init_target_positions)));
   }
   
   const double* init_target_positions;
};


/**
 * CenterJointsGoal
*/
struct CenterJointsGoal {
  CenterJointsGoal(const double* target_centers)
  : target_centers(target_centers){}

  template <typename T>
  bool operator()(const T* target_values, T* residuals) const // param_x, param_y, residuals
  {
    for (int i=0; i<robot::num_targets; i++)
    {
      residuals[i] = target_values[i] - target_centers[i];
    }
    return true;
  }

   // Factory to hide the construction of the CostFunction object from the client code.
   static ceres::CostFunction* Create(const double* target_centers)
   {
     return (new ceres::AutoDiffCostFunction<CenterJointsGoal, robot::num_targets, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
                 new CenterJointsGoal(target_centers)));
   }
   
   const double* target_centers;
};

struct EndpointGoal {
  EndpointGoal(const Eigen::Vector3d &endpoint, const Eigen::Quaterniond &direction, const double* variable_positions) : endpoint(endpoint), direction(direction), variable_positions(variable_positions) {}

  template <typename T>
  bool operator()(const T* target_values, T* residuals) const // param_x, param_y, residuals
  {
    T global_link_translations[3*robot::num_links];
    T global_link_rotations[4*robot::num_links];
    utils::computeGlobalLinkTransforms(target_values, variable_positions, global_link_translations, global_link_rotations);

    // Position cost
    residuals[0] = ceres::hypot(global_link_translations[3*robot::endpoint_link_idx+0] - endpoint[0],
                                global_link_translations[3*robot::endpoint_link_idx+1] - endpoint[1],
                                global_link_translations[3*robot::endpoint_link_idx+2] - endpoint[2]);

    // Orientation cost
    residuals[1] = global_link_rotations[4*robot::endpoint_link_idx+0] - direction.w();
    residuals[2] = global_link_rotations[4*robot::endpoint_link_idx+1] - direction.x();
    residuals[3] = global_link_rotations[4*robot::endpoint_link_idx+2] - direction.y();
    residuals[4] = global_link_rotations[4*robot::endpoint_link_idx+3] - direction.z();

    return true;
  }

   // Factory to hide the construction of the CostFunction object from the client code.
   static ceres::CostFunction* Create(const Eigen::Vector3d &endpoint, const Eigen::Quaterniond &direction, const double* variable_positions)
   {
     return (new ceres::AutoDiffCostFunction<EndpointGoal, 5, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
                 new EndpointGoal(endpoint, direction, variable_positions)));
   }

  const Eigen::Vector3d endpoint;
  const Eigen::Quaterniond direction;
  const double* variable_positions;
};


struct CollisionAvoidanceGoal {
  CollisionAvoidanceGoal(const double* variable_positions) : variable_positions(variable_positions) {}

  template <typename T>
  bool operator()(const T* target_values, T* residuals) const // param_x, param_y, residuals
  {
    // Jet conversion is needed for some variables because we have unknown number of residuals in this structure
    T variable_positions_JET[robot::num_variables];
    for (int i=0; i<robot::num_variables; i++) variable_positions_JET[i] = T(variable_positions[i]);
    T object_transform_translation_only_JET[robot::num_objects*3]; 
    T object_transform_quaternion_only_JET[robot::num_objects*4]; 
    for (int i=0; i<robot::num_objects; i++)
    {
      object_transform_translation_only_JET[i*3+0] = T(robot::object_transform_translation_only[i][0]);
      object_transform_translation_only_JET[i*3+1] = T(robot::object_transform_translation_only[i][1]);
      object_transform_translation_only_JET[i*3+2] = T(robot::object_transform_translation_only[i][2]);
      object_transform_quaternion_only_JET[i*4+0] = T(robot::object_transform_quaternion_only[i][0]);
      object_transform_quaternion_only_JET[i*4+1] = T(robot::object_transform_quaternion_only[i][1]);
      object_transform_quaternion_only_JET[i*4+2] = T(robot::object_transform_quaternion_only[i][2]);
      object_transform_quaternion_only_JET[i*4+3] = T(robot::object_transform_quaternion_only[i][3]);
    }

    // Compute link transforms
    T global_link_translations[robot::num_links*3];
    T global_link_rotations[robot::num_links*4];
    utils::computeGlobalLinkTransforms(target_values, variable_positions_JET, global_link_translations, global_link_rotations);

    // Compute internal object positions
    T global_object_translations[robot::num_objects*3];
    T global_object_rotations[robot::num_objects*4];
    utils::computeCollisionTransforms((const T*)global_link_translations,
                                      (const T*)global_link_rotations,
                                      (const T*)object_transform_translation_only_JET,
                                      (const T*)object_transform_quaternion_only_JET,
                                      (const int*)robot::object_can_skip_translation,
                                      (const int*)robot::object_can_skip_rotation,
                                      (const int*)robot::object_idx_to_link_idx,
                                      robot::num_objects,
                                      (T*)global_object_translations,
                                      (T*)global_object_rotations);

    // Compute collision pairs
    // TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO 
    // TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO 
    // TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO 
    // TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO

    return true; // TODO: maybe return false if there is a collision?
  }

  
  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(const int num_pairs, const double* variable_positions)
  {
    //return (new ceres::NumericDiffCostFunction<CollisionAvoidanceGoal, ceres::CENTRAL, ceres::DYNAMIC, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
    return (new ceres::AutoDiffCostFunction<CollisionAvoidanceGoal, ceres::DYNAMIC, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
                new CollisionAvoidanceGoal(variable_positions), num_pairs));
    // THANK GOD CERES AutoDiffCostFunction SUPPORTS DYNAMIC NUMBER OF RESIDUALS!!!
  }

  const double* variable_positions;
};







} // namespace salih_marangoz_thesis

#endif // __SALIH_MARANGOZ_THESIS_GOALS__