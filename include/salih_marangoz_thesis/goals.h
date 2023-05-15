#ifndef __SALIH_MARANGOZ_THESIS_GOALS__
#define __SALIH_MARANGOZ_THESIS_GOALS__

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <salih_marangoz_thesis/robot_configuration/robot_configuration.h>
#include <salih_marangoz_thesis/utils.h>

namespace salih_marangoz_thesis
{

struct PreferredJointPositionGoal {
  PreferredJointPositionGoal(){}

  template <typename T>
  bool operator()(const T* target_values, T* residuals) const // param_x, param_y, residuals
  {
    for (int target_idx=0; target_idx<robot::num_targets; target_idx++)
    {
      int joint_idx = robot::target_idx_to_joint_idx[target_idx];
      residuals[target_idx] = robot::weight_preferred_joint_position_goal[joint_idx] * (target_values[target_idx] - robot::joint_preferred_position[joint_idx]);
    }
    return true;
  }

   // Factory to hide the construction of the CostFunction object from the client code.
   static ceres::CostFunction* Create()
   {
     return (new ceres::AutoDiffCostFunction<PreferredJointPositionGoal, robot::num_targets, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
                 new PreferredJointPositionGoal()));
   }
};

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
    
    // TEST: SLOW!
    /*
    Eigen::Quaterniond endpoint_dir;
    endpoint_dir.w() = global_link_rotations[4*robot::endpoint_link_idx+0];
    endpoint_dir.x() = global_link_rotations[4*robot::endpoint_link_idx+1];
    endpoint_dir.y() = global_link_rotations[4*robot::endpoint_link_idx+2];
    endpoint_dir.z() = global_link_rotations[4*robot::endpoint_link_idx+3];
    residuals[1] = endpoint_dir.angularDistance(direction);
    */

    // TEST: SLOW!
    /*
    residuals[1] = 1.0-global_link_rotations[4*robot::endpoint_link_idx+0]*direction.w() -
                   global_link_rotations[4*robot::endpoint_link_idx+1]*direction.x() -
                   global_link_rotations[4*robot::endpoint_link_idx+2]*direction.y() -
                   global_link_rotations[4*robot::endpoint_link_idx+3]*direction.z();
    */

    // couldn't implement this method
    /*
    T inverse_quad[4];
    inverse_quad[0] = global_link_rotations[4*robot::endpoint_link_idx+0];
    inverse_quad[1] = -global_link_rotations[4*robot::endpoint_link_idx+1];
    inverse_quad[2] = -global_link_rotations[4*robot::endpoint_link_idx+2];
    inverse_quad[3] = -global_link_rotations[4*robot::endpoint_link_idx+3];
    T endpoint[4];
    endpoint[0] = direction.w();
    endpoint[1] = direction.x();
    endpoint[2] = direction.y();
    endpoint[3] = direction.z();
    T tmp[4];
    ceres::QuaternionProduct(endpoint, inverse_quad, tmp);
    */

    return true;
  }

   // Factory to hide the construction of the CostFunction object from the client code.
   static ceres::CostFunction* Create(const Eigen::Vector3d &endpoint, const Eigen::Quaterniond &direction, const double* variable_positions)
   {
     //return (new ceres::NumericDiffCostFunction<EndpointGoal, ceres::FORWARD, 5, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
     //            new EndpointGoal(endpoint, direction, variable_positions)));
     return (new ceres::AutoDiffCostFunction<EndpointGoal, 5, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
                 new EndpointGoal(endpoint, direction, variable_positions)));
   }

  const Eigen::Vector3d endpoint;
  const Eigen::Quaterniond direction;
  const double* variable_positions;
};


struct CollisionAvoidanceGoal {
  CollisionAvoidanceGoal(const double* variable_positions, 
                         const int* int_collision_pair_a, 
                         const int* int_collision_pair_b, 
                         const int num_int_pairs, 
                         const std::vector<CollisionObject*>& int_collision_objects)
  :  variable_positions(variable_positions),
     int_collision_pair_a(int_collision_pair_a),
     int_collision_pair_b(int_collision_pair_b),
     num_int_pairs(num_int_pairs),
     int_collision_objects(int_collision_objects)
  {}

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

    // TODO: supports only spheres!!!
    for (int i=0; i<num_int_pairs; i++)
    {
      int object_idx_a = int_collision_pair_a[i];
      int object_idx_b = int_collision_pair_b[i];
      int link_idx_a = robot::object_idx_to_link_idx[object_idx_a];
      int link_idx_b = robot::object_idx_to_link_idx[object_idx_b];
      const CollisionObject* object_a = int_collision_objects[object_idx_a];
      const CollisionObject* object_b = int_collision_objects[object_idx_b];
      const Sphere& shape_a = static_cast<const Sphere&>(*(object_a->collisionGeometry()));
      const Sphere& shape_b = static_cast<const Sphere&>(*(object_b->collisionGeometry()));
      const T* pos_a = &(global_object_translations[object_idx_a*3]);
      const T* pos_b = &(global_object_translations[object_idx_b*3]);

      // TODO: write a function for this!
      // TODO: maybe use not inflated objects? inflated ones only for the broadphase collision detection.
      const T distance = utils::distSphere2Sphere(pos_a, 
                                                  shape_a.radius-robot::default_inflation, 
                                                  pos_b, 
                                                  shape_b.radius-robot::default_inflation);
      residuals[i] = 0.001 / (distance);
      
      if (distance < 0) return false; // TODO: maybe return false if there is a collision?
    }

    return true; 
  }

  
  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(const double* variable_positions, 
                                     const int* int_collision_pair_a, 
                                     const int* int_collision_pair_b, 
                                     const int num_int_pairs,
                                     const std::vector<CollisionObject*>& int_collision_objects)
  {
    // TODO: which one to select?
    return (new ceres::NumericDiffCostFunction<CollisionAvoidanceGoal, ceres::FORWARD, ceres::DYNAMIC, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
                new CollisionAvoidanceGoal(variable_positions, int_collision_pair_a, int_collision_pair_b, num_int_pairs, int_collision_objects), ceres::TAKE_OWNERSHIP, num_int_pairs));
    //return (new ceres::AutoDiffCostFunction<CollisionAvoidanceGoal, ceres::DYNAMIC, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
    //            new CollisionAvoidanceGoal(variable_positions, int_collision_pair_a, int_collision_pair_b, num_int_pairs, int_collision_objects), num_int_pairs));

  }

  const double* variable_positions;
  const int* int_collision_pair_a;
  const int* int_collision_pair_b;
  int num_int_pairs;
  const std::vector<CollisionObject*>& int_collision_objects;
};







} // namespace salih_marangoz_thesis

#endif // __SALIH_MARANGOZ_THESIS_GOALS__